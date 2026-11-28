#!/usr/bin/env python3
"""Bateria do TCC com UM NOVO TERMINAL GRAFICO por experimento.

O terminal principal só organiza o plano e observa arquivos de estado.
Cada novo terminal: source ROS + workspace -> launch -> aguarda Gazebo/RViz
e lifecycle ativo -> executor -> fecha e limpa. Não altera ROS_DOMAIN_ID.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
import uuid

@dataclass(frozen=True)
class Teste:
    id: int
    metodo: str
    mapa: int
    movimentacao: str
    repeticao: int
    repeticoes: int
    duracao: float | None


class FalhaTentativa(RuntimeError):
    """Falha operacional do terminal com categoria para tentativas seguras."""
    def __init__(self, mensagem, categoria="execucao"):
        super().__init__(mensagem)
        self.categoria = categoria


def aviso(s):
    print(f"[{datetime.now():%H:%M:%S}] {s}", flush=True)


def criar_plano(inicio, metodos, mapas, modos, n_front, n_rota, duracao):
    planos=[]
    for modo in modos:
        for metodo in metodos:
            for mapa in mapas:
                n=n_front if modo=="fronteiras" else n_rota
                for repeticao in range(1,n+1):
                    planos.append(Teste(inicio+len(planos),metodo,mapa,modo,
                                        repeticao,n,duracao if modo=="fronteiras" else None))
    return planos


def grupos_repeticao(planos):
    grupos={}
    for t in planos:
        grupos.setdefault((t.movimentacao,t.metodo,t.mapa,t.duracao),[]).append(t)
    return grupos


def pastas_do_id(ws, id_):
    padrao=re.compile(rf"^(?:exp_{id_}_|experimento_0*{id_}$)")
    pastas=[]
    for raiz in (ws/"bags",ws/"results"):
        if raiz.is_dir():
            pastas.extend(p for p in raiz.iterdir() if p.is_dir() and padrao.match(p.name))
    return pastas


def booleano(v):
    return v is True or str(v).lower() in ("true","1","sim","yes")


def completo(ws,t):
    pastas=pastas_do_id(ws,t.id)
    bags=[p for p in pastas if p.parent.name=="bags"]
    res=[p for p in pastas if p.parent.name=="results"]
    if len(bags)!=1 or len(res)!=1:return False
    try:
        man=json.loads((bags[0]/"manifest.json").read_text())
        with (res[0]/"metricas.csv").open(newline="",encoding="utf-8") as f:
            row=next(csv.DictReader(f))
        if str(man.get("slam_method"))!=t.metodo or int(man.get("map_id",-1))!=t.mapa:
            return False
        if man.get("modo_experimento","fronteiras")!=t.movimentacao:
            return False
        if man.get("estado_execucao")!="analisado" or not booleano(row.get("experimento_intervalo_completo")):
            return False
        if t.movimentacao=="rota_fixa":
            rota=ws/"src"/"tcc_slam_exploration"/"routes"/f"mapa_{t.mapa}.json"
            return (rota.is_file() and
                    man.get("rota_sha256")==hashlib.sha256(rota.read_bytes()).hexdigest() and
                    man.get("rota_concluida") is True)
        return (abs(float(man.get("duracao_configurada_s"))-t.duracao)<.01 and
                abs(float(row.get("experimento_duracao_analisada_s"))-t.duracao)<=1)
    except (OSError,ValueError,TypeError,StopIteration,KeyError):
        return False


def localizar_terminal(preferido):
    candidatos={
        "gnome-terminal":["gnome-terminal","--","bash","-lc"],
        "xfce4-terminal":["xfce4-terminal","--disable-server","--command"],
        "konsole":["konsole","--noclose","-e","bash","-lc"],
        "xterm":["xterm","-e","bash","-lc"],
    }
    if preferido=="auto":
        for nome in ("gnome-terminal","xfce4-terminal","konsole","xterm"):
            if shutil.which(nome):
                return nome
        raise RuntimeError("Nenhum terminal gráfico encontrado. Instale gnome-terminal, xfce4-terminal, konsole ou xterm.")
    if not shutil.which(preferido):
        raise RuntimeError(f"Terminal gráfico não encontrado: {preferido}")
    return preferido


def comando_terminal(tipo, script):
    # script já está citado e escapado com shlex.quote
    if tipo=="gnome-terminal":
        return ["gnome-terminal","--","bash","-lc",script]
    if tipo=="xfce4-terminal":
        return ["xfce4-terminal","--disable-server","--command",f"bash -lc {__import__('shlex').quote(script)}"]
    if tipo=="konsole":
        return ["konsole","-e","bash","-lc",script]
    return ["xterm","-e","bash","-lc",script]


def arquivar_parciais_de_inicializacao(ws, t, pasta_tentativa):
    """Preserva dados vazios/incompletos antes de uma nova tentativa do MESMO ID.

    So chamar quando worker reportou falha de infraestrutura ANTES de GOAL_SENT.
    Nenhum arquivo e removido; os nomes completos continuam rastreaveis.
    """
    pastas = pastas_do_id(ws, t.id)
    for p in pastas:
        man = p / "manifest.json"
        if man.is_file():
            try:
                dados = json.loads(man.read_text(encoding="utf-8"))
                if dados.get("estado_execucao") in ("analisado", "gravacao_concluida"):
                    raise RuntimeError(f"Pasta nao pode ser movida pois ha gravacao concluida: {p}")
            except json.JSONDecodeError as ex:
                raise RuntimeError(f"Manifest ilegivel, arquivamento manual necessario: {p}") from ex
        destino = pasta_tentativa / "dados_parciais" / p.parent.name / p.name
        destino.parent.mkdir(parents=True, exist_ok=True)
        if destino.exists():
            raise RuntimeError(f"Destino de arquivos parciais ja existe: {destino}")
        shutil.move(str(p), str(destino))
        aviso(f"Dados parciais preservados em {destino}")


def solicitar_encerramento_worker(status_file, config_file):
    """Se o supervisor falhar, interrompe o worker do proprio teste."""
    try:
        estado_atual = json.loads(Path(status_file).read_text())
        pid = int(estado_atual.get("pid", 0))
        if pid <= 0:
            return
        cmdline = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
        if "worker_experimento_terminal.py" not in cmdline or str(config_file) not in cmdline:
            return  # Evita sinalizar PID reutilizado por outro processo.
        aviso(f"Solicitando encerramento controlado ao worker do teste (PID {pid})")
        os.kill(pid, signal.SIGINT)
        fim = time.monotonic() + 30
        while Path(f"/proc/{pid}").exists() and time.monotonic() < fim:
            time.sleep(0.2)
        if Path(f"/proc/{pid}").exists():
            aviso(f"AVISO: worker {pid} nao encerrou em 30s; nao inicie outra bateria ate verificar.")
    except (FileNotFoundError, OSError, ValueError, json.JSONDecodeError):
        pass


def executar_um(ws,t,logdir,terminal,timeout_prontidao,timeout_experimento,sem_janelas, tentativa=1):
    import shlex
    pasta=logdir/f"exp_{t.id:03d}_{t.metodo}_m{t.mapa}_{t.movimentacao}_r{t.repeticao}_tentativa_{tentativa:02d}"
    pasta.mkdir()
    cfg={
        "id":t.id,"metodo":t.metodo,"mapa":t.mapa,
        "movimentacao":t.movimentacao,"duracao":t.duracao,"ws":str(ws),
        "status":str(pasta/"estado.json"),"navlog":str(pasta/"navigation.log"),
        "explog":str(pasta/"experimento.log"),"token":uuid.uuid4().hex,
        "timeout_prontidao":timeout_prontidao,
        "timeout_experimento":timeout_experimento,
        "sem_verificar_janelas":sem_janelas,
    }
    config=pasta/"config.json"
    config.write_text(json.dumps(cfg,ensure_ascii=False,indent=2))
    worker=Path(__file__).with_name("worker_experimento_terminal.py")
    if not worker.is_file():
        raise RuntimeError(f"Worker não encontrado: {worker}")
    # Comandos source são executados efetivamente dentro de cada novo terminal.
    texto=(f"source /opt/ros/lyrical/setup.bash && "
           f"source {shlex.quote(str(ws/'install/setup.bash'))} && "
           f"python3 -u {shlex.quote(str(worker))} --config {shlex.quote(str(config))}")
    cmd=comando_terminal(terminal,texto)
    aviso(f"Experimento {t.id}: abrindo terminal {terminal}.")
    # O terminal pode reutilizar um processo servidor existente. O resultado
    # definitivo vem de estado.json, nao do codigo de saida do gnome-terminal.
    client=subprocess.Popen(cmd,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True)
    init=time.monotonic()
    prev=None
    try:
      while time.monotonic()-init < timeout_prontidao+timeout_experimento+180:
        status=cfg["status"]
        if Path(status).is_file():
            try:
                obj=json.loads(Path(status).read_text())
            except (OSError,json.JSONDecodeError):
                time.sleep(.5);continue
            fase=obj["fase"]
            if fase!=prev:
                aviso(f"Experimento {t.id}: {fase} — {obj.get('detalhe','')}")
                prev=fase
            if fase in ("concluido","falha"):
                client.poll()
                # Aguardar a saída efetiva do worker antes de abrir outra janela.
                worker_pid=obj.get("pid")
                if worker_pid:
                    limite_saida=time.monotonic()+20
                    while Path(f"/proc/{worker_pid}").exists() and time.monotonic()<limite_saida:
                        time.sleep(.25)
                    if Path(f"/proc/{worker_pid}").exists():
                        raise RuntimeError(f"Worker do experimento {t.id} ainda está ativo "
                                           f"(PID {worker_pid}); não iniciar outro terminal.")
                if fase=="falha":
                    raise FalhaTentativa(f"Experimento {t.id}: {obj.get('detalhe','')} "
                                       f"(logs: {pasta})", obj.get("categoria", "execucao"))
                if not completo(ws,t):
                    raise RuntimeError(f"Experimento {t.id} terminou, mas não passou na "
                                       f"verificação de manifest/metricas (logs: {pasta})")
                return
        elif client.poll() is not None:
            # GNOME terminal pode retornar 0 enquanto o servidor cria a janela.
            if time.monotonic()-init > 45:
                stderr=client.stderr.read() if client.stderr else ""
                raise FalhaTentativa(f"Terminal não iniciou worker em 45s: {stderr[:1500]}", "infraestrutura")
        time.sleep(2)
      raise FalhaTentativa(f"Experimento {t.id}: supervisão excedeu o tempo limite. "
                         f"Verifique a janela do teste e {pasta}.")
    except BaseException:
      # Em qualquer erro do supervisor, nao deixar uma janela/worker executando
      # sem controle antes de encerrar o processo principal da bateria.
      solicitar_encerramento_worker(cfg["status"], config)
      raise


def comparar(ws,ids,logdir,nome):
    if len(ids)<2:return
    cmd=[sys.executable,str(ws/"comparar_experimento.py"),*map(str,ids)]
    arquivo=logdir/f"comparacao_{nome}.log"
    aviso(f"Comparando {nome}: {ids}")
    with arquivo.open("w") as log:
        r=subprocess.run(cmd,cwd=ws,stdout=log,stderr=subprocess.STDOUT,timeout=900)
    if r.returncode:
        aviso(f"AVISO: comparação {nome} falhou (log {arquivo})")


def main():
    ap=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--workspace",type=Path,default=Path.home()/"tcc_slam_ws")
    ap.add_argument("--inicio",type=int,default=100)
    ap.add_argument("--metodos",nargs="+",choices=["graph","rbpf","ekf"],default=["graph","rbpf","ekf"])
    ap.add_argument("--mapas",nargs="+",type=int,default=[1,2,3,4])
    ap.add_argument("--movimentacao",choices=["fronteiras","rota_fixa","ambas"],default="ambas")
    ap.add_argument("--repeticoes",type=int,default=1)
    ap.add_argument("--repeticoes-fronteiras",type=int)
    ap.add_argument("--repeticoes-rota-fixa",type=int)
    ap.add_argument("--duracao",type=float,default=300)
    ap.add_argument("--terminal",choices=["auto","gnome-terminal","xfce4-terminal","konsole","xterm"],default="auto")
    ap.add_argument("--timeout-prontidao",type=float,default=240,help="Limite real para Gazebo/RViz + Nav2 ativos")
    ap.add_argument("--timeout-fronteiras",type=float,default=3600)
    ap.add_argument("--timeout-rota",type=float,default=7200)
    ap.add_argument("--pausa",type=float,default=12,help="Pausa real após fechar cada terminal")
    ap.add_argument("--tentativas-inicializacao", type=int, default=2,
                    help="Tentativas em outro terminal quando launch/Nav2 nao iniciar (padrao 2)")
    ap.add_argument("--sem-verificar-janelas",action="store_true",help="Desativa teste por processo de Gazebo/RViz (mantém lifecycle)")
    ap.add_argument("--retomar",action="store_true")
    ap.add_argument("--somente-comparar",action="store_true")
    ap.add_argument("--sem-comparacoes",action="store_true")
    ap.add_argument("--comparar-metodos",action="store_true")
    ap.add_argument("--simular",action="store_true")
    args=ap.parse_args()
    nf=args.repeticoes_fronteiras or args.repeticoes
    nr=args.repeticoes_rota_fixa or args.repeticoes
    if min(nf,nr,args.inicio,args.tentativas_inicializacao)<1 or args.duracao<=0 or args.pausa<0 or min(args.timeout_prontidao,args.timeout_rota,args.timeout_fronteiras)<=0:
        ap.error("Repetições/IDs/duração/timeout devem ser positivos; pausa >= 0")
    if len(set(args.metodos))!=len(args.metodos) or len(set(args.mapas))!=len(args.mapas) or any(m<1 for m in args.mapas):
        ap.error("Mapas e métodos não podem se repetir; IDs dos mapas positivos")
    modos=["fronteiras","rota_fixa"] if args.movimentacao=="ambas" else [args.movimentacao]
    testes=criar_plano(args.inicio,args.metodos,args.mapas,modos,nf,nr,args.duracao)
    aviso(f"Plano: {len(testes)} experimentos.")
    for t in testes:
        print(f"{t.id:03d} | {t.metodo} mapa {t.mapa} | {t.movimentacao} | rep {t.repeticao}/{t.repeticoes}")
    if args.simular:return
    ws=args.workspace.expanduser().resolve()
    if not (ws/"executar_experimento.py").is_file() or not (ws/"comparar_experimento.py").is_file():
        raise FileNotFoundError("Workspace sem executar_experimento.py ou comparar_experimento.py")
    for t in testes:
        existentes=pastas_do_id(ws,t.id)
        if existentes and not (args.retomar or args.somente_comparar):
            raise RuntimeError(f"ID {t.id} já utilizado: {existentes}")
        if existentes and args.retomar and not completo(ws,t) and not args.somente_comparar:
            raise RuntimeError(f"ID {t.id} contém resultados incompletos; arquive-os antes da retomada: {existentes}")
    terminal=None
    if not args.somente_comparar:
        if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            raise RuntimeError("Nenhuma sessão gráfica detectada (DISPLAY/WAYLAND_DISPLAY).")
        terminal=localizar_terminal(args.terminal)
    stamp=datetime.now().strftime("%Y%m%d_%H%M%S")
    logdir=ws/"logs"/"baterias_terminais"/f"bateria_{stamp}"
    logdir.mkdir(parents=True,exist_ok=False)
    (logdir/"plano.json").write_text(json.dumps([asdict(t) for t in testes],ensure_ascii=False,indent=2))
    aviso(f"Logs: {logdir}")
    if not args.somente_comparar:
        for t in testes:
            if args.retomar and completo(ws,t):
                aviso(f"Experimento {t.id} já concluído: pulando.")
                continue
            aviso(f"INÍCIO {t.id} — método {t.metodo}, mapa {t.mapa}, {t.movimentacao}")
            timeout=args.timeout_rota if t.movimentacao=="rota_fixa" else args.timeout_fronteiras
            for tentativa in range(1, args.tentativas_inicializacao+1):
                try:
                    executar_um(ws,t,logdir,terminal,args.timeout_prontidao,timeout,
                                args.sem_verificar_janelas,tentativa)
                    break
                except FalhaTentativa as erro:
                    # Somente falhas ANTES do executor, sem bag/resultado parcial,
                    # podem ser reiniciadas sem alterar a amostra experimental.
                    if (erro.categoria != "infraestrutura" or tentativa >= args.tentativas_inicializacao):
                        raise RuntimeError(str(erro)) from erro
                    atual = logdir/f"exp_{t.id:03d}_{t.metodo}_m{t.mapa}_{t.movimentacao}_r{t.repeticao}_tentativa_{tentativa:02d}"
                    arquivar_parciais_de_inicializacao(ws, t, atual)
                    aviso(f"Falha de inicialização na tentativa {tentativa}: {erro}")
                    aviso("Abrindo novo terminal para repetir APENAS a inicialização...")
                    time.sleep(max(12.0,args.pausa))
            aviso(f"FIM {t.id}: dados verificados.")
            aviso(f"Pausa de {args.pausa:g}s antes do próximo terminal.")
            time.sleep(args.pausa)
    if not args.sem_comparacoes:
        faltando=[t.id for t in testes if not completo(ws,t)]
        if faltando:raise RuntimeError(f"Comparações suspensas. IDs incompletos: {faltando}")
        for (modo,metodo,mapa,_),grupo in grupos_repeticao(testes).items():
            if len(grupo)>=2:
                comparar(ws,[t.id for t in grupo],logdir,f"consistencia_{modo}_{metodo}_map{mapa}")
        if args.comparar_metodos:
            for modo in modos:
                for mapa in args.mapas:
                    reps = nf if modo == "fronteiras" else nr
                    for repeticao in range(1,reps+1):
                        ids=[t.id for t in testes if t.movimentacao==modo and
                             t.mapa==mapa and t.repeticao==repeticao]
                        if len(ids)>=2:
                            comparar(ws,ids,logdir,f"metodos_{modo}_map{mapa}_r{repeticao}")
    aviso("BATERIA CONCLUÍDA.")
    return 0


if __name__=="__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        aviso("Interrompida pelo usuário; se houver um terminal de teste aberto, encerre-o com Ctrl+C.")
        sys.exit(130)
    except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as e:
        aviso(f"ERRO: {e}")
        sys.exit(1)
