#!/usr/bin/env python3
"""Executa UM experimento no terminal gráfico dedicado, com cleanup seguro."""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
import uuid

MARCADOR = "TCC_TESTE_TERMINAL_ID"
ATIVOS = ("planner_server", "controller_server", "bt_navigator")


def estado(path, fase, **dados):
    path = Path(path)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(dict(fase=fase, pid=os.getpid(), instante=time.time(), **dados),
                              ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(path)
    print(f"[{time.strftime('%H:%M:%S')}] {fase}: {dados.get('detalhe','')}", flush=True)


def alive(proc):
    return proc is not None and proc.poll() is None


def identificados(token):
    """Busca SOMENTE processos do usuário que herdaram o marcador do teste."""
    encontrados = {}
    uid = os.getuid()
    for item in Path("/proc").iterdir():
        if not item.name.isdecimal():
            continue
        try:
            if item.stat().st_uid != uid:
                continue
            env = (item / "environ").read_bytes().split(b"\0")
            if (MARCADOR + "=" + token).encode() not in env:
                continue
            st = (item / "stat").read_text()
            campos = st[st.rfind(")")+2:].split()
            if campos[0] != "Z":
                encontrados[int(item.name)] = campos[19]
        except (PermissionError, OSError, IndexError):
            pass
    return encontrados


def sinal_grupo(p, sig):
    if not alive(p):
        return
    try:
        os.killpg(p.pid, sig)
    except ProcessLookupError:
        pass


def limpar(proc_list, token):
    """Encerra processos iniciados neste teste; nunca usa pkill global."""
    for p in proc_list:
        sinal_grupo(p, signal.SIGINT)
    time.sleep(2)
    for sig, pausa in ((signal.SIGTERM, 4), (signal.SIGKILL, 2)):
        for p in proc_list:
            sinal_grupo(p, sig)
        existentes = identificados(token)
        for pid, start in existentes.items():
            try:
                st = Path(f"/proc/{pid}/stat").read_text()
                atual = st[st.rfind(")")+2:].split()[19]
                if atual == start:
                    os.kill(pid, sig)
            except (ProcessLookupError, PermissionError, OSError, IndexError):
                pass
        time.sleep(pausa)
    for p in proc_list:
        if p:
            try: p.wait(timeout=2)
            except subprocess.TimeoutExpired: pass
    sobras = identificados(token)
    if sobras:
        raise RuntimeError(f"Restaram processos do teste: {list(sobras)}")


def log_drain(pipe, destino):
    with Path(destino).open("w", encoding="utf-8") as f:
        for linha in iter(pipe.readline, ""):
            f.write(linha)
            f.flush()
            print(linha, end="", flush=True)


def rodar_clang(cmd, env, max_s=5):
    try:
        return subprocess.run(cmd, env=env, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True,
                              timeout=max_s).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ""


def processos_visiveis(token):
    linhas = []
    for pid in identificados(token):
        try:
            cmd = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
            linhas.append(cmd.lower())
        except OSError: pass
    return linhas


def prontidao(nav, env, token, timeout, exigir_gui, status):
    fim = time.monotonic() + timeout
    ultimo = None
    while time.monotonic() < fim:
        if nav.poll() is not None:
            raise RuntimeError(f"navigation.launch.py encerrou antes da prontidão: código {nav.returncode}")
        comandos = processos_visiveis(token)
        rviz = any(re.search(r'(^|[/\s])rviz2(\s|$)', s) for s in comandos)
        gazebo = any(re.search(r'(^|[/\s])gz\s+sim(\s|$)', s) or
                     "gz-sim-server" in s or "gz-sim-gui" in s for s in comandos)
        if not exigir_gui or (rviz and gazebo):
            estados = {}
            for nome in ATIVOS:
                out = rodar_clang(["ros2","lifecycle","get",f"/{nome}"], env)
                estados[nome] = "active [3]" in out
            if all(estados.values()):
                actions = rodar_clang(["ros2","action","list"], env)
                if "/navigate_to_pose" in actions and "/compute_path_to_pose" in actions:
                    return
            detalhe = f"GUI gazebo={gazebo} rviz2={rviz}; lifecycle={estados}"
        else:
            detalhe = f"Esperando GUI: gazebo={gazebo}, rviz2={rviz}"
        if detalhe != ultimo:
            estado(status, "aguardando_prontidao", detalhe=detalhe)
            ultimo = detalhe
        time.sleep(4)
    raise RuntimeError(f"Prontidão não confirmada em {timeout:g}s reais. Último estado: {ultimo}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    args = ap.parse_args()
    cfg = json.loads(Path(args.config).read_text())
    ws = Path(cfg["ws"])
    status = Path(cfg["status"])
    navlog = Path(cfg["navlog"])
    explog = Path(cfg["explog"])
    token = cfg["token"]
    env = os.environ.copy()
    env[MARCADOR] = token
    env["PYTHONUNBUFFERED"] = "1"
    procs = []
    sucesso = False
    erro = ""
    fase_erro = "inicializacao"
    estado(status,"iniciando",detalhe=f"Experimento {cfg['id']}")
    try:
        navcmd = ["ros2","launch","tcc_slam_bringup","navigation.launch.py",
                  f"map:={cfg['mapa']}", f"slam_method:={cfg['metodo']}"]
        print("+ " + " ".join(navcmd),flush=True)
        with navlog.open("w",encoding="utf-8") as f:
            nav = subprocess.Popen(navcmd, cwd=ws, env=env, stdout=f,
                                   stderr=subprocess.STDOUT, start_new_session=True)
            procs.append(nav)
            estado(status,"iniciando_simulacao",detalhe=f"PID do launch={nav.pid}")
            prontidao(nav, env, token, cfg["timeout_prontidao"],
                      not cfg["sem_verificar_janelas"], status)
            estado(status,"executando_experimento",detalhe="Nav2 ativo; iniciando executor")
            fase_erro = "execucao"
            cmd = [sys.executable,"-u",str(ws/"executar_experimento.py"),str(cfg["id"]),
                   "--metodo", cfg["metodo"],"--mapa",str(cfg["mapa"]),
                   "--movimentacao", cfg["movimentacao"]]
            if cfg["movimentacao"]=="fronteiras":
                cmd.extend(["--duracao",str(cfg["duracao"])])
            print("+ " + " ".join(cmd),flush=True)
            exp = subprocess.Popen(cmd, cwd=ws, env=env, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True,
                                   bufsize=1, start_new_session=True)
            procs.append(exp)
            th = threading.Thread(target=log_drain, args=(exp.stdout,explog),daemon=True)
            th.start()
            deadline=time.monotonic()+cfg["timeout_experimento"]
            while exp.poll() is None:
                if nav.poll() is not None:
                    raise RuntimeError("Launcher encerrou antes do executor.")
                if time.monotonic()>deadline:
                    raise RuntimeError("Tempo operacional máximo excedido (tempo real).")
                time.sleep(2)
            th.join(timeout=5)
            if exp.returncode:
                # O launch pode estar ativo, mas /navigate_to_pose desaparecer
                # antes do ActionClient do executor ficar pronto. Essa e uma
                # falha DE INICIALIZACAO, nao uma falha de navegação.
                # Nao repetir falhas depois do primeiro GOAL_SENT.
                try:
                    relato = explog.read_text(encoding="utf-8", errors="replace")
                except OSError:
                    relato = ""
                sem_meta = "GOAL_SENT" not in relato and "CRONÔMETRO INICIADO" not in relato
                if sem_meta and any(erro_antes_inicio in relato for erro_antes_inicio in (
                    "Nav2 não ficou disponível.",
                    "Não foi possível receber /clock.",
                )):
                    fase_erro = "inicializacao"
                raise RuntimeError(f"Executor retornou {exp.returncode}; verifique {explog}")
            sucesso=True
    except BaseException as exc:
        erro=f"{type(exc).__name__}: {exc}"
        print("ERRO:", erro,flush=True)
    finally:
        estado(status,"encerrando",detalhe="Finalizando subprocessos deste teste")
        try:
            limpar(procs,token)
        except Exception as exc:
            sucesso=False
            fase_erro = "execucao"  # Nunca repetir se nao encerrou tudo.
            erro += f"; falha na limpeza: {exc}"
        estado(status,"concluido" if sucesso else "falha",
               detalhe=erro if erro else "Executor terminou sem erros",
               categoria="infraestrutura" if not sucesso and fase_erro == "inicializacao" else "execucao",
               codigo=0 if sucesso else 1)
    return 0 if sucesso else 1


if __name__=="__main__":
    sys.exit(main())
