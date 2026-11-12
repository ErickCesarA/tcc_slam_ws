#!/usr/bin/env python3
"""Compara resultados existentes do TCC-SLAM sem ROS, Gazebo ou rosbag.

Uso: python3 ~/tcc_slam_ws/comparar_experimento.py 21 22 23 24
     python3 ~/tcc_slam_ws/comparar_experimento.py 21 22 --saida nome
"""
from __future__ import annotations

import argparse
from datetime import datetime
from html import escape
import json
import math
from pathlib import Path
import re
import shutil
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

METODOS = {"ekf": "MRPT EKF-SLAM", "rbpf": "MRPT RBPF-SLAM",
           "graph": "MRPT GraphSLAM", "toolbox": "SLAM Toolbox"}
METRICAS = [
    ("slam_ATE_m_RMSE", "ATE posição — RMSE", "m", "menor"),
    ("slam_ATE_angular_graus_RMSE", "ATE angular — RMSE", "°", "menor"),
    ("slam_RPE_1s_translacao_m_RMSE", "RPE translação (1 s)", "m", "menor"),
    ("slam_RPE_5s_translacao_m_RMSE", "RPE translação (5 s)", "m", "menor"),
    ("slam_RPE_10s_translacao_m_RMSE", "RPE translação (10 s)", "m", "menor"),
    ("mapa_variacao_area_conhecida_m2", "Expansão da área conhecida", "m²", "maior"),
    ("mapa_taxa_media_m2_s", "Taxa média de mapeamento", "m²/s", "maior"),
    ("exploracao_eficiencia_m2_por_m", "Expansão por distância", "m²/m", "maior"),
    ("exploracao_objetivos_enviados", "Metas enviadas", "", "informativo"),
    ("exploracao_objetivos_alcancados", "Metas alcançadas", "", "informativo"),
    ("exploracao_objetivos_falhos", "Metas falhas", "", "informativo"),
    ("exploracao_objetivos_sem_resultado", "Metas sem resultado", "", "informativo"),
    ("experimento_duracao_configurada_s", "Duração solicitada", "s", "informativo"),
    ("experimento_duracao_analisada_s", "Duração analisada", "s", "informativo"),
]


def numero(x, casas=3):
    try:
        x = float(x)
        return f"{x:.{casas}f}" if math.isfinite(x) else "—"
    except (TypeError, ValueError):
        return "—"


def val(x):
    try:
        z = float(x)
        return z if math.isfinite(z) else np.nan
    except (TypeError, ValueError):
        return np.nan


def booleano(v):
    if isinstance(v, bool):
        return v
    return str(v).strip().lower() in ("true", "1", "sim", "yes")


def ler_csv(arquivo):
    if not arquivo.is_file():
        return pd.DataFrame()
    try:
        return pd.read_csv(arquivo)
    except (OSError, ValueError, pd.errors.EmptyDataError) as e:
        raise RuntimeError(f"Erro ao ler {arquivo}: {e}") from e


def carregar(ws, numero_id):
    ident = f"{numero_id:03d}"
    pasta = ws / "results" / f"experimento_{ident}"
    csv = pasta / "metricas.csv"
    if not csv.is_file():
        raise FileNotFoundError(f"Experimento {ident}: metricas.csv não encontrado: {csv}\n"
                                f"Gere primeiro: ~/tcc_slam_ws/analisar_experimento.sh {numero_id}")
    df = ler_csv(csv)
    if len(df) != 1:
        raise ValueError(f"{csv}: esperado exatamente 1 registro, encontrado {len(df)}")
    m = df.iloc[0].to_dict()
    manifesto = ws / "bags" / f"experimento_{ident}" / "manifest.json"
    if manifesto.is_file():
        try:
            dado = json.loads(manifesto.read_text(encoding="utf-8"))
            metodo_manifesto = dado.get("slam_method")
            if metodo_manifesto and m.get("experimento_metodo_slam") and str(metodo_manifesto) != str(m.get("experimento_metodo_slam")):
                print(f"AVISO: método no manifest do experimento {ident} difere de metricas.csv", file=sys.stderr)
        except (ValueError, OSError) as e:
            print(f"AVISO: manifest.json do experimento {ident}: {e}", file=sys.stderr)
    metodo = str(m.get("experimento_metodo_slam", "não informado"))
    label = f"{METODOS.get(metodo, metodo)} · {ident}"
    return {"id": ident, "num": numero_id, "pasta": pasta, "m": m,
            "metodo": metodo, "label": label,
            "traj": ler_csv(pasta / "trajetoria_slam.csv"),
            "odom": ler_csv(pasta / "trajetoria_odom.csv"),
            "mapa": ler_csv(pasta / "mapa_evolucao.csv"),
            "events": ler_csv(pasta / "eventos_exploracao.csv"),
            "image": pasta / "mapa_final.png"}


def salvar(fig, destino):
    fig.tight_layout()
    fig.savefig(destino, dpi=155, bbox_inches="tight")
    plt.close(fig)


def graficos(experimentos, imgdir, limite_tempo):
    registros = []
    def registrar(arquivo, titulo, explicacao):
        registros.append((f"graficos/{arquivo}", titulo, explicacao))

    # Barras para métricas comparáveis, sem classificar metas enviadas como qualidade.
    selecoes = [
        ("slam_ATE_m_RMSE", "ATE translacional — RMSE (menor é melhor)", "m", "ate_posicao.png"),
        ("slam_ATE_angular_graus_RMSE", "ATE angular — RMSE (menor é melhor)", "graus", "ate_angular.png"),
        ("mapa_variacao_area_conhecida_m2", "Expansão de área conhecida", "m²", "area_conhecida.png"),
        ("mapa_taxa_media_m2_s", "Taxa de expansão do mapa", "m²/s", "taxa_mapeamento.png"),
        ("exploracao_eficiencia_m2_por_m", "Expansão de área por distância", "m²/m", "eficiencia.png"),
    ]
    for chave, titulo, unidade, arquivo in selecoes:
        valores = np.array([val(e["m"].get(chave)) for e in experimentos])
        if not np.isfinite(valores).any():
            continue
        fig, ax = plt.subplots(figsize=(max(8, len(experimentos) * 1.3), 4.6))
        positions = np.arange(len(experimentos))
        bars = ax.bar(positions[np.isfinite(valores)], valores[np.isfinite(valores)])
        ax.set_xticks(positions, [e["label"] for e in experimentos], rotation=18, ha="right")
        ax.set_ylabel(unidade); ax.set_title(titulo); ax.grid(axis="y", alpha=.25)
        ax.bar_label(bars, fmt="%.3f", padding=3, fontsize=8)
        salvar(fig, imgdir / arquivo)
        registrar(arquivo, titulo, "Comparação descritiva entre execuções. ATE/RPE menores indicam menor erro; mais área conhecida não garante melhor qualidade geométrica.")

    # RPE por janela, com diferentes barras por método/experimento.
    janelas = (1, 5, 10)
    if any(np.isfinite(val(e["m"].get(f"slam_RPE_{j}s_translacao_m_RMSE"))) for e in experimentos for j in janelas):
        fig, ax = plt.subplots(figsize=(9, 4.5))
        centro = np.arange(3)
        largura = min(.75 / len(experimentos), .25)
        for i, e in enumerate(experimentos):
            valores = [val(e["m"].get(f"slam_RPE_{j}s_translacao_m_RMSE")) for j in janelas]
            ax.bar(centro + (i-(len(experimentos)-1)/2)*largura, valores, largura, label=e["label"])
        ax.set_xticks(centro, [f"{j} s" for j in janelas]); ax.set_ylabel("RMSE RPE (m)")
        ax.set_title("Erro relativo translacional por janela"); ax.grid(axis="y", alpha=.25); ax.legend(fontsize=8)
        salvar(fig, imgdir / "rpe_janelas.png")
        registrar("rpe_janelas.png", "Consistência local — RPE", "Erro relativo calculado em intervalos de 1, 5 e 10 segundos. Menor RMSE indica maior consistência local.")

    # Curvas temporais: sempre usam o tempo relativo ao início experimental.
    series = [
        ("traj", "erro_posicao_m", "Evolução do erro absoluto de posição", "Erro (m)", "erro_posicao_tempo.png"),
        ("traj", "erro_angular_graus", "Evolução do erro angular", "Erro (graus)", "erro_angular_tempo.png"),
        ("mapa", "area_conhecida_m2", "Evolução da área conhecida", "Área conhecida (m²)", "area_tempo.png"),
    ]
    for tipo, coluna, titulo, ylabel, arquivo in series:
        fig, ax = plt.subplots(figsize=(9.5, 4.4))
        desenhou = False
        for e in experimentos:
            df = e[tipo]
            if df.empty or coluna not in df.columns:
                continue
            if "tempo_relativo_s" in df.columns:
                t = pd.to_numeric(df["tempo_relativo_s"], errors="coerce")
            elif "tempo_s" in df.columns:
                t = pd.to_numeric(df["tempo_s"], errors="coerce") - val(e["m"].get("experimento_inicio_simulado_s"))
            else:
                continue
            y = pd.to_numeric(df[coluna], errors="coerce")
            mask = np.isfinite(t) & np.isfinite(y) & (t >= -1e-4) & (t <= limite_tempo + 1e-4)
            if mask.any():
                ax.plot(t[mask], y[mask], label=e["label"], linewidth=1.4)
                desenhou = True
        if desenhou:
            ax.set(xlabel="Tempo desde o início experimental (s)", ylabel=ylabel, title=titulo)
            ax.set_xlim(left=0, right=max(limite_tempo, .1)); ax.grid(alpha=.25); ax.legend(fontsize=8)
            salvar(fig, imgdir / arquivo)
            registrar(arquivo, titulo, "Curvas alinhadas por tempo relativo; a área conhecida não representa diretamente precisão cartográfica.")
        else:
            plt.close(fig)

    # Resultados da exploração: separar concluidos, falhas, sem resultado.
    fig, ax = plt.subplots(figsize=(max(8, len(experimentos)*1.4), 4.6))
    x = np.arange(len(experimentos))
    reached = np.array([max(0,val(e["m"].get("exploracao_objetivos_alcancados"))) if np.isfinite(val(e["m"].get("exploracao_objetivos_alcancados"))) else 0 for e in experimentos])
    failed = np.array([max(0,val(e["m"].get("exploracao_objetivos_falhos"))) if np.isfinite(val(e["m"].get("exploracao_objetivos_falhos"))) else 0 for e in experimentos])
    pending = np.array([max(0,val(e["m"].get("exploracao_objetivos_sem_resultado"))) if np.isfinite(val(e["m"].get("exploracao_objetivos_sem_resultado"))) else 0 for e in experimentos])
    ax.bar(x, reached, label="Alcançadas")
    ax.bar(x, failed, bottom=reached, label="Falhas")
    ax.bar(x, pending, bottom=reached+failed, label="Sem resultado")
    ax.set_xticks(x,[e["label"] for e in experimentos], rotation=18, ha="right")
    ax.set_ylabel("Número de metas"); ax.set_title("Resultados das metas de exploração"); ax.grid(axis="y", alpha=.25); ax.legend()
    salvar(fig, imgdir / "exploracao.png")
    registrar("exploracao.png", "Exploração autônoma", "Metas sem resultado não devem ser contadas como sucesso. Contagens refletem a duração real de cada experimento.")
    return registros


def comparar(ws, ids, saida=None):
    exp = [carregar(ws, i) for i in ids]
    tempo = [val(e["m"].get("experimento_duracao_analisada_s")) for e in exp]
    duracoes = [val(e["m"].get("experimento_duracao_configurada_s")) for e in exp]
    validos = [x for x in tempo if np.isfinite(x) and x > 0]
    limite = min(validos) if validos else 0
    avisos = []
    if len(validos) != len(exp):
        avisos.append("Há experimentos sem duração analisada válida; algumas curvas podem estar ausentes.")
    if len(validos) > 1 and max(validos)-min(validos) > 1.0:
        avisos.append("DURAÇÕES ANALISADAS DIFERENTES: indicadores finais como área e metas não estão diretamente comparáveis. Curvas foram limitadas ao menor intervalo comum; métricas finais permanecem as originais de cada execução.")
    if all(np.isfinite(x) for x in duracoes) and max(duracoes)-min(duracoes) > 1:
        avisos.append("Durações solicitadas diferentes entre experimentos.")
    incompletos = [e["id"] for e in exp if not booleano(e["m"].get("experimento_intervalo_completo"))]
    if incompletos:
        avisos.append("Experimentos marcados como incompletos: " + ", ".join(incompletos) + ".")
    alinhamentos = set(str(e["m"].get("experimento_alinhamento", "não informado")) for e in exp)
    if len(alinhamentos) > 1:
        avisos.append("DIFERENTES MÉTODOS DE ALINHAMENTO DE TRAJETÓRIA: comparação de ATE requer cautela.")
    if len(set(e["metodo"] for e in exp)) != len(exp):
        avisos.append("Há mais de um experimento para o mesmo método: cada execução é mostrada separadamente, sem inferência estatística entre repetições.")
    avisos.append("Comparação descritiva: resultados de execuções isoladas não demonstram superioridade estatística. Para comparar desempenho, utilize o mesmo ambiente, robô, duração e condições iniciais.")
    avisos.append("Metas e área conhecida dependem do percurso explorado; diferença de cobertura não é sinônimo de precisão do mapa.")
    if saida:
        if not re.fullmatch(r"[\w-]+", saida, re.UNICODE):
            raise ValueError("--saida aceita apenas letras, números, _ e -")
        destino = ws / "results" / "comparacoes" / saida
        if destino.exists():
            raise FileExistsError(f"Diretório já existe: {destino}; escolha outro --saida")
    else:
        nome = "comparacao_" + "_".join(e["id"] for e in exp) + "_" + datetime.now().strftime("%Y%m%d_%H%M%S")
        destino = ws / "results" / "comparacoes" / nome
        n = 1
        base = destino
        while destino.exists():
            destino = Path(str(base) + f"_{n}")
            n += 1
    imgdir = destino / "graficos"
    imgdir.mkdir(parents=True)
    figmeta = graficos(exp, imgdir, limite if limite > 0 else 1)

    # Exporta uma linha por experimento com os números originais.
    todas_chaves = list(dict.fromkeys([chave for e in exp for chave in e["m"]]))
    registros = [{"experimento":e["id"], "metodo":e["metodo"], **{c:e["m"].get(c) for c in todas_chaves}} for e in exp]
    pd.DataFrame(registros).to_csv(destino / "metricas_comparadas.csv", index=False)

    tabela_long = []
    tabela_html = []
    for chave, titulo, unidade, objetivo in METRICAS:
        valores = [val(e["m"].get(chave)) for e in exp]
        valid = np.array(valores, dtype=float)
        best = -1
        # Não classificar resultados quando duração varia, exceto erros de trajetória;
        # mesmo os erros exigem condições semelhantes.
        duracao_sensivel = chave.startswith("mapa_") or chave.startswith("exploracao_")
        if objetivo != "informativo" and np.isfinite(valid).any() and not (duracao_sensivel and len(validos)>1 and max(validos)-min(validos)>1.0):
            best = int(np.nanargmin(valid) if objetivo=="menor" else np.nanargmax(valid))
        celulas = []
        for i, (e, v) in enumerate(zip(exp, valores)):
            tabela_long.append({"metrica":chave,"descricao":titulo,"unidade":unidade,"experimento":e["id"],"metodo":e["metodo"],"valor":v})
            classe = ' class="melhor"' if i==best and np.isfinite(v) else ""
            celulas.append(f"<td{classe}>{escape(numero(v))}</td>")
        tabela_html.append(f"<tr><th>{escape(titulo)}{(' ('+unidade+')') if unidade else ''}</th>{''.join(celulas)}</tr>")
    pd.DataFrame(tabela_long).to_csv(destino / "metricas_por_indicador.csv", index=False)

    cards = []
    for e in exp:
        if e["image"].is_file():
            nome = f"mapa_{e['id']}.png"
            shutil.copy2(e["image"], imgdir / nome)
            bloco = f'<img loading="lazy" src="graficos/{nome}" alt="Mapa do experimento {e["id"]}">'
        else:
            bloco = '<p>Imagem do mapa indisponível.</p>'
        cards.append(f'<article class="mapa"><h3>{escape(e["label"])}</h3>{bloco}<p>Duração analisada: {numero(e["m"].get("experimento_duracao_analisada_s"),1)} s</p></article>')
    imagens_html = "\n".join(f'<section class="grafico"><h2>{escape(t)}</h2><p>{escape(d)}</p><img loading="lazy" src="{escape(p,quote=True)}" alt="{escape(t,quote=True)}"></section>' for p,t,d in figmeta)
    header = "".join(f"<th>{escape(e['label'])}</th>" for e in exp)
    warnings = "".join(f"<li>{escape(t)}</li>" for t in avisos)
    html = f'''<!doctype html>
<html lang="pt-BR"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Comparação de experimentos SLAM</title><style>
:root{{--fundo:#f5f7fa;--azul:#163d61;--borda:#dbe2e9}}body{{margin:0;background:var(--fundo);color:#233140;font:16px/1.5 system-ui,Arial,sans-serif}}
header{{background:var(--azul);color:white;padding:35px max(20px,5vw)}}header h1{{margin:0}}header p{{opacity:.9}}
main{{max-width:1200px;margin:auto;padding:24px}}section{{background:white;border:1px solid var(--borda);border-radius:12px;margin:22px 0;padding:22px;box-shadow:0 2px 8px #163d610a}}
.warning{{background:#fff8e7;border-left:5px solid #e4a021}}table{{border-collapse:collapse;width:100%;font-size:14px}}th,td{{border-bottom:1px solid var(--borda);padding:11px;text-align:right}}
th:first-child{{text-align:left}}thead th{{background:#edf3f8}}.scroller{{overflow:auto}}.melhor{{background:#e3f5e6;font-weight:700}}.grafico img{{width:100%;max-width:1000px;display:block;margin:auto}}.mapas{{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:16px}}
.mapa{{border:1px solid var(--borda);border-radius:10px;padding:12px;min-width:0}}.mapa img{{width:100%;height:auto}}.mapa h3{{font-size:16px}}
small,footer{{color:#607182}}footer{{text-align:center;padding:30px}}</style></head><body>
<header><h1>Comparação de experimentos SLAM</h1><p>{len(exp)} experimentos: {escape(', '.join(e['id'] for e in exp))} · Gerado em {datetime.now().strftime('%d/%m/%Y %H:%M')}</p></header>
<main><section class="warning"><h2>Avisos e condições de comparação</h2><ul>{warnings}</ul>
<p><b>Janela comum para curvas:</b> {numero(limite,2)} s. <b>Valores finais:</b> são aqueles calculados individualmente em cada experimento.</p></section>
<section><h2>Indicadores numéricos</h2><p>Valores destacados em verde representam somente o melhor valor numérico do indicador (quando aplicável), não uma conclusão geral sobre qual SLAM é superior.</p>
<div class="scroller"><table><thead><tr><th>Métrica</th>{header}</tr></thead><tbody>{''.join(tabela_html)}</tbody></table></div></section>
{imagens_html}<section><h2>Mapas finais registrados</h2><p>Imagens originais dos relatórios individuais; não são capturas da interface RViz2. Os referenciais de mapa podem ser distintos, portanto estas figuras são mostradas separadamente e não sobrepostas.</p><div class="mapas">{''.join(cards)}</div></section>
<section><h2>Arquivos exportados</h2><p><code>metricas_comparadas.csv</code> reúne uma linha por experimento; <code>metricas_por_indicador.csv</code> organiza uma linha por indicador e execução. As imagens estão em <code>graficos/</code>.</p></section></main>
<footer>TCC · ROS 2 / Gazebo · Comparação descritiva de SLAM</footer></body></html>'''
    (destino / "relatorio_comparativo.html").write_text(html,encoding="utf-8")
    (destino / "configuracao.json").write_text(json.dumps({"experimentos": [e["id"] for e in exp],"janela_comum_s": limite,"avisos":avisos},ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    return destino, avisos


def main():
    ap = argparse.ArgumentParser(description="Relatório HTML comparativo de experimentos SLAM já analisados; não depende do ROS 2.")
    ap.add_argument("experimentos", nargs="+", type=int, help="Números dos experimentos (ex.: 21 22 23 24)")
    ap.add_argument("--workspace", type=Path, default=Path.home()/"tcc_slam_ws", help="Caminho para o workspace principal")
    ap.add_argument("--saida", help="Nome da pasta em results/comparacoes/ (não substitui pastas existentes)")
    args=ap.parse_args()
    if len(args.experimentos)<2:
        ap.error("Informe pelo menos dois experimentos")
    if len(set(args.experimentos))!=len(args.experimentos) or any(x<0 for x in args.experimentos):
        ap.error("Os números devem ser diferentes e não negativos")
    pasta, avisos = comparar(args.workspace.expanduser().resolve(),args.experimentos,args.saida)
    print("\nCOMPARAÇÃO CONCLUÍDA")
    print(f"Relatório HTML: {pasta/'relatorio_comparativo.html'}")
    print(f"Métricas: {pasta/'metricas_comparadas.csv'}")
    for aviso in avisos:
        print(f"AVISO: {aviso}")
    print(f"\nAbra com: firefox '{pasta/'relatorio_comparativo.html'}'")

if __name__=="__main__":
    try:
        main()
    except (OSError,ValueError,RuntimeError) as exc:
        sys.exit(f"ERRO: {exc}")
