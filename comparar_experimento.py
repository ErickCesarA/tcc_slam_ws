#!/usr/bin/env python3
# TCC_ROUTE_END_V1
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

from experimento_paths import localizar_experimento

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


# Indicadores para percursos concluidos; tempo e custo de movimento predominam.
METRICAS_ROTA = [
    ("slam_ATE_m_RMSE", "ATE posicao — RMSE", "m", "menor"),
    ("slam_ATE_angular_graus_RMSE", "ATE angular — RMSE", "°", "menor"),
    ("slam_RPE_1s_translacao_m_RMSE", "RPE translacao (1 s)", "m", "menor"),
    ("rota_tempo_execucao_s", "Tempo para terminar a rota", "s", "menor"),
    ("rota_distancia_real_m", "Distancia realmente percorrida", "m", "menor"),
    ("rota_velocidade_media_m_s", "Velocidade media", "m/s", "informativo"),
    ("rota_waypoints_planejados", "Waypoints da rota", "", "informativo"),
    ("rota_waypoints_alcancados", "Waypoints atingidos", "", "informativo"),
    ("mapa_variacao_area_conhecida_m2", "Expansao da area conhecida", "m²", "informativo"),
    ("experimento_duracao_analisada_s", "Intervalo analisado", "s", "informativo"),
]


# TCC_ETAPA3_GEOMETRIA — indicadores em ambas as modalidades.
METRICAS_GEOMETRIA = [
    ("geometria_iou_conhecido", "IoU de obstáculos (região conhecida)", "", "informativo"),
    ("geometria_iou_global", "IoU global de obstáculos", "", "informativo"),
    ("geometria_precisao_obstaculos", "Precisão dos obstáculos", "", "informativo"),
    ("geometria_recall_obstaculos_conhecidos", "Recall dos obstáculos conhecidos", "", "informativo"),
    ("geometria_recall_obstaculos_global", "Recall global dos obstáculos", "", "informativo"),
    ("geometria_cobertura_conhecida_pct", "Cobertura da área do SDF", "%", "informativo"),
    ("geometria_erro_bordas_medio_m", "Erro médio de bordas", "m", "informativo"),
    ("geometria_erro_bordas_p95_m", "Erro P95 de bordas", "m", "informativo"),
    ("geometria_erro_centroide_medio_m", "Desvio aproximado de centroides", "m", "informativo"),
    ("geometria_area_falsos_ocupados_m2", "Área falsamente ocupada", "m²", "informativo"),
    ("geometria_area_obstaculos_desconhecidos_m2", "Área de obstáculos desconhecidos", "m²", "informativo"),
    ("geometria_area_ocupada_fora_piso_m2", "Ocupação fora do piso nominal", "m²", "informativo"),
]
METRICAS.extend(METRICAS_GEOMETRIA)
METRICAS_ROTA.extend(METRICAS_GEOMETRIA)


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
    pasta_bag, pasta = localizar_experimento(ws, numero_id)
    csv = pasta / "metricas.csv"
    if not csv.is_file():
        raise FileNotFoundError(f"Experimento {ident}: metricas.csv não encontrado: {csv}\n"
                                f"Gere primeiro: ~/tcc_slam_ws/analisar_experimento.sh {numero_id}")
    df = ler_csv(csv)
    if len(df) != 1:
        raise ValueError(f"{csv}: esperado exatamente 1 registro, encontrado {len(df)}")
    m = df.iloc[0].to_dict()
    manifesto = pasta_bag / "manifest.json"
    mapa_id = None
    modo = "fronteiras"
    rota_sha256 = None
    dado = {}
    if manifesto.is_file():
        try:
            dado = json.loads(manifesto.read_text(encoding="utf-8"))
            metodo_manifesto = dado.get("slam_method")
            mapa_id = dado.get("map_id")
            modo = dado.get("modo_experimento") or "fronteiras"
            rota_sha256 = dado.get("rota_sha256")
            if metodo_manifesto and m.get("experimento_metodo_slam") and str(metodo_manifesto) != str(m.get("experimento_metodo_slam")):
                print(f"AVISO: método no manifest do experimento {ident} difere de metricas.csv", file=sys.stderr)
        except (ValueError, OSError) as e:
            print(f"AVISO: manifest.json do experimento {ident}: {e}", file=sys.stderr)
    metodo = str(m.get("experimento_metodo_slam", "não informado"))
    label = f"{METODOS.get(metodo, metodo)} · {ident} · mapa {mapa_id if mapa_id is not None else chr(63)}"
    return {"id": ident, "num": numero_id, "pasta": pasta, "m": m,
            "mapa_id": mapa_id, "modo": modo, "rota_sha256": rota_sha256, "manifest": dado,
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


def graficos(experimentos, imgdir, limite_tempo, modo="fronteiras"):
    registros = []
    def registrar(arquivo, titulo, explicacao):
        registros.append((f"graficos/{arquivo}", titulo, explicacao))

    # Para rotas fixas, tempo e distancia sao indicadores prioritarios.
    selecoes = [
        ("slam_ATE_m_RMSE", "ATE translacional — RMSE", "m", "ate_posicao.png"),
        ("slam_ATE_angular_graus_RMSE", "ATE angular — RMSE", "graus", "ate_angular.png"),
        ("rota_tempo_execucao_s", "Tempo para concluir a rota", "s", "tempo_rota.png"),
        ("rota_distancia_real_m", "Distancia realmente percorrida", "m", "distancia_rota.png"),
    ] if modo == "rota_fixa" else [
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
    ax.set_ylabel("Número de metas"); ax.set_title("Waypoints da rota fixa" if modo == "rota_fixa" else "Resultados das metas de exploração"); ax.grid(axis="y", alpha=.25); ax.legend()
    salvar(fig, imgdir / "exploracao.png")
    registrar("exploracao.png", "Waypoints da rota fixa" if modo == "rota_fixa" else "Exploração autônoma", "Contagem de metas do Nav2; a rota termina somente apos o waypoint final." if modo == "rota_fixa" else "Metas sem resultado não devem ser contadas como sucesso.")
    return registros


def comparar(ws, ids, saida=None):
    exp = [carregar(ws, i) for i in ids]
    modos = {e["modo"] for e in exp}
    if len(modos) != 1:
        raise ValueError("Nao misture fronteiras e rota_fixa na mesma comparacao. "
                         "Gere relatorios separados para cada modalidade.")
    modo_atual = next(iter(modos))
    if modo_atual not in ("fronteiras", "rota_fixa"):
        raise ValueError(f"Modo desconhecido: {modo_atual}")
    metricas_modo = METRICAS_ROTA if modo_atual == "rota_fixa" else METRICAS
    titulo_modo = ("Comparacao de rotas fixas" if modo_atual == "rota_fixa"
                   else "Comparacao de exploracao autonoma")
    tempo = [val(e["m"].get("experimento_duracao_analisada_s")) for e in exp]
    duracoes = [val(e["m"].get("experimento_duracao_configurada_s")) for e in exp]
    validos = [x for x in tempo if np.isfinite(x) and x > 0]
    limite = min(validos) if validos else 0
    avisos = []
    mapas = [str(e["mapa_id"]) if e["mapa_id"] is not None else None for e in exp]
    mapas_conhecidos = {m for m in mapas if m is not None}
    mapas_incomparaveis = len(mapas_conhecidos) != 1 or any(x is None for x in mapas)
    if mapas_incomparaveis:
        avisos.append("MAPAS NÃO CONTROLADOS: experimentos usam mapas diferentes ou não possuem map_id no manifest. "
                      "Não atribua diferenças de cobertura, deslocamento ou área exclusivamente ao SLAM.")
    modos = {str(e["modo"]) if e["modo"] is not None else "não informado" for e in exp}
    if modo_atual == "rota_fixa":
        hashes = {e["rota_sha256"] for e in exp}
        if len(hashes) != 1 or None in hashes:
            avisos.append("ROTAS DIFERENTES OU NAO IDENTIFICADAS: os resultados nao representam a mesma tarefa geometrica.")
        avisos.append("Tempo de conclusao de rota e comparavel apenas com rota, mapa e condicoes identicos. ATE/RPE permanecem indicadores de localizacao.")
    if len(validos) != len(exp):
        avisos.append("Há experimentos sem duração analisada válida; algumas curvas podem estar ausentes.")
    if len(validos) > 1 and max(validos)-min(validos) > 1.0:
        if modo_atual == "rota_fixa":
            avisos.append("DURACOES DE ROTAS DIFERENTES: isto e um resultado do teste, nao uma configuracao diferente. Curvas temporais foram limitadas ao intervalo comum; valores finais sao mantidos.")
        else:
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
        avisos.append("Há mais de um experimento para o mesmo método. Se as configurações coincidirem, será apresentada uma análise descritiva de repetibilidade.")
    avisos.append("Comparação descritiva: execuções isoladas não demonstram superioridade estatística. Exige-se mesmo mapa, mesma versão de código, trajetória/método de navegação controlados e sementes registradas.")
    avisos.append("Metas e área conhecida dependem do percurso explorado e das características das grades; cobertura não mede automaticamente a acurácia geométrica do mapa.")
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
    figmeta = graficos(exp, imgdir, limite if limite > 0 else 1, modo=modo_atual)

    # Exporta uma linha por experimento com os números originais.
    todas_chaves = list(dict.fromkeys([chave for e in exp for chave in e["m"]]))
    registros = [{"experimento":e["id"], "metodo":e["metodo"], **{c:e["m"].get(c) for c in todas_chaves}} for e in exp]
    pd.DataFrame(registros).to_csv(destino / "metricas_comparadas.csv", index=False)

    tabela_long = []
    tabela_html = []
    for chave, titulo, unidade, objetivo in metricas_modo:
        valores = [val(e["m"].get(chave)) for e in exp]
        valid = np.array(valores, dtype=float)
        best = -1
        # Não classificar resultados quando duração varia, exceto erros de trajetória;
        # mesmo os erros exigem condições semelhantes.
        duracao_sensivel = chave.startswith("mapa_") or chave.startswith("exploracao_")
        if modo_atual == "rota_fixa" and chave in ("rota_tempo_execucao_s", "rota_distancia_real_m"):
            duracao_sensivel = False
        # Sem destaque de 'melhor' quando mapa ou rota nao foram controlados.
        trajetorias_diversas = (modo_atual == "rota_fixa" and
            (len({e["rota_sha256"] for e in exp}) != 1 or None in {e["rota_sha256"] for e in exp}))

        if objetivo != "informativo" and np.isfinite(valid).any() and not (trajetorias_diversas or (modo_atual == "rota_fixa" and mapas_incomparaveis) or (duracao_sensivel and (mapas_incomparaveis or (modo_atual == "fronteiras" and len(validos)>1 and max(validos)-min(validos)>1.0)))):
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
    <title>{escape(titulo_modo)}</title><style>
:root{{--fundo:#f5f7fa;--azul:#163d61;--borda:#dbe2e9}}body{{margin:0;background:var(--fundo);color:#233140;font:16px/1.5 system-ui,Arial,sans-serif}}
header{{background:var(--azul);color:white;padding:35px max(20px,5vw)}}header h1{{margin:0}}header p{{opacity:.9}}
main{{max-width:1200px;margin:auto;padding:24px}}section{{background:white;border:1px solid var(--borda);border-radius:12px;margin:22px 0;padding:22px;box-shadow:0 2px 8px #163d610a}}
.warning{{background:#fff8e7;border-left:5px solid #e4a021}}table{{border-collapse:collapse;width:100%;font-size:14px}}th,td{{border-bottom:1px solid var(--borda);padding:11px;text-align:right}}
th:first-child{{text-align:left}}thead th{{background:#edf3f8}}.scroller{{overflow:auto}}.melhor{{background:#e3f5e6;font-weight:700}}.grafico img{{width:100%;max-width:1000px;display:block;margin:auto}}.mapas{{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:16px}}
.mapa{{border:1px solid var(--borda);border-radius:10px;padding:12px;min-width:0}}.mapa img{{width:100%;height:auto}}.mapa h3{{font-size:16px}}
small,footer{{color:#607182}}footer{{text-align:center;padding:30px}}</style></head><body>
<header><h1>{escape(titulo_modo)}</h1><p>{len(exp)} experimentos: {escape(', '.join(e['id'] for e in exp))} · Gerado em {datetime.now().strftime('%d/%m/%Y %H:%M')}</p></header>
<main><section class="warning"><h2>Avisos e condições de comparação</h2><ul>{warnings}</ul>
<p><b>Janela comum para curvas:</b> {numero(limite,2)} s. <b>Valores finais:</b> são aqueles calculados individualmente em cada experimento.</p></section>
<section><h2>Indicadores numéricos</h2><p>Valores destacados em verde representam somente o melhor valor numérico do indicador (quando aplicável), não uma conclusão geral sobre qual SLAM é superior.</p>
<div class="scroller"><table><thead><tr><th>Métrica</th>{header}</tr></thead><tbody>{''.join(tabela_html)}</tbody></table></div></section>
{imagens_html}<section><h2>Mapas finais registrados</h2><p>Imagens originais dos relatórios individuais; não são capturas da interface RViz2. Os referenciais de mapa podem ser distintos, portanto estas figuras são mostradas separadamente e não sobrepostas.</p><div class="mapas">{''.join(cards)}</div></section>
<section><h2>Arquivos exportados</h2><p><code>metricas_comparadas.csv</code> reúne uma linha por experimento; <code>metricas_por_indicador.csv</code> organiza uma linha por indicador e execução. As imagens estão em <code>graficos/</code>.</p></section></main>
<footer>TCC · ROS 2 / Gazebo · Comparação descritiva de SLAM</footer></body></html>'''
    # TCC_CONSISTENCIA_REPETICOES_V1
    from consistencia_repeticoes import gerar_bloco_consistencia
    bloco_consistencia = gerar_bloco_consistencia(exp, destino)
    if bloco_consistencia:
        html = html.replace("<main>", "<main>" + bloco_consistencia, 1)
    # TCC_ETAPA3_GEOMETRIA: imagens e proveniência do SDF para o relatório.
    blocos_geometria = []
    hashes_por_mapa = {}
    faltando = []
    fontes = set()
    for e in exp:
        status = e["m"].get("geometria_status")
        png = e["pasta"] / "geometria" / "sobreposicao_mapa_gazebo.png"
        if status != "ok" or not png.is_file():
            faltando.append(e["id"])
            continue
        fontes.add(str(e["m"].get("geometria_fonte_sdf")))
        mid = str(e["m"].get("geometria_mapa_id"))
        hash_sdf = str(e["m"].get("geometria_sha256_sdf"))
        hashes_por_mapa.setdefault(mid, set()).add(hash_sdf)
        nome_fig = f"geometria_{e['id']}.png"
        shutil.copy2(png, imgdir / nome_fig)
        blocos_geometria.append(
            f'<article class="mapa"><h3>{escape(e["label"])}</h3>'
            f'<img src="graficos/{nome_fig}" alt="Sobreposição geométrica do experimento {e["id"]}"></article>'
        )
    alertas_geo = []
    if faltando:
        alertas_geo.append('Sem avaliação geométrica válida nos experimentos: ' + ', '.join(faltando))
    if any(len(h) > 1 for h in hashes_por_mapa.values()):
        alertas_geo.append('Diferentes arquivos SDF foram usados para o mesmo identificador de mapa.')
    if 'sdf_atual_nao_verificado' in fontes:
        alertas_geo.append('Pelo menos um experimento usa o SDF atual sem prova de correspondência histórica.')
    texto_alertas = ''.join(f'<li>{escape(s)}</li>' for s in alertas_geo)
    secao_geom = (
        '<section id="comparacao-geometrica"><h2>Fidelidade geométrica × Gazebo</h2>'
        '<p>Os indicadores de IoU, cobertura e erro estão na tabela numérica acima. '
        'Células desconhecidas são diferentes de células classificadas como livres. '
        'A IoU de obstáculos sólidos deve ser interpretada em conjunto com o erro de bordas.</p>'
        + (f'<ul>{texto_alertas}</ul>' if texto_alertas else '')
        + '<div class="mapas">' + ''.join(blocos_geometria) + '</div></section>'
    )
    html = html.replace('</main>', secao_geom + '</main>', 1)
    (destino / "relatorio_comparativo.html").write_text(html,encoding="utf-8")
    (destino / "configuracao.json").write_text(json.dumps({"experimentos": [e["id"] for e in exp],"janela_comum_s": limite,"avisos":avisos,"modo_experimento":modo_atual},ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    return destino, avisos



# TCC_COMPARACAO_GRUPOS_V1 — comparação de médias de repetições independentes.
def _normalizar_id(v):
    if v is None:
        return None
    if isinstance(v, float) and not math.isfinite(v):
        return None
    s = str(v).strip()
    return s if s and s.lower() not in ('nan', 'none', 'null') else None


def _num_manifesto(e, chave, fallback=None):
    m = e.get('manifest') or {}
    r = val(m.get(chave, fallback))
    return float(r) if np.isfinite(r) else None


def _validar_grupos(a, b):
    """Distingue incompatibilidade confirmada de metadados ausentes.

    Não exige que algoritmos diferentes tenham os mesmos hashes de configuração
    específicos do SLAM; exige, porém, mesmo mundo, modalidade e tarefa.
    """
    erros, avisos = [], []
    for letra, grupo in [('A', a), ('B', b)]:
        if len(grupo) < 2:
            erros.append(f'Grupo {letra}: informe ao menos duas repetições.')
        metodos = {e['metodo'] for e in grupo}
        if len(metodos) != 1:
            erros.append(f'Grupo {letra}: as repetições devem usar um único método SLAM.')
        mapas = {_normalizar_id(e['mapa_id']) for e in grupo}
        if None in mapas or len(mapas) != 1:
            erros.append(f'Grupo {letra}: mapa ausente ou diferente entre repetições.')
        modos = {e['modo'] for e in grupo}
        if len(modos) != 1:
            erros.append(f'Grupo {letra}: modalidades de movimentação diferentes.')
        if any(not e.get('manifest') for e in grupo):
            erros.append(f'Grupo {letra}: manifest.json ausente; não é possível comprovar as condições.')
        modo = next(iter(modos)) if len(modos) == 1 else None
        if modo == 'fronteiras':
            duracoes = [_num_manifesto(e, 'duracao_configurada_s', e['m'].get('experimento_duracao_configurada_s')) for e in grupo]
            if None in duracoes or max(duracoes) - min(duracoes) > 1e-6:
                erros.append(f'Grupo {letra}: duração programada ausente ou divergente.')
        elif modo == 'rota_fixa':
            hs = {_normalizar_id(e.get('rota_sha256')) for e in grupo}
            if None in hs or len(hs) != 1:
                erros.append(f'Grupo {letra}: hash da rota ausente ou divergente.')
            for e in grupo:
                concluida = e['manifest'].get('rota_concluida')
                if concluida is False:
                    erros.append(f'Grupo {letra}: rota não concluída no experimento {e["id"]}.')
                elif concluida is not True:
                    avisos.append(f'Grupo {letra}: conclusão da rota não confirmada no manifest do experimento {e["id"]}.')
        else:
            erros.append(f'Grupo {letra}: modalidade não reconhecida ({modo}).')
        # Mesmo método: alteração de configuração entre repetições é relevante.
        configs = [e['manifest'].get('hashes_configuracao_sha256') for e in grupo]
        if all(isinstance(h, dict) and h for h in configs):
            if any(h != configs[0] for h in configs[1:]):
                erros.append(f'Grupo {letra}: hashes das configurações variam entre repetições.')
        else:
            avisos.append(f'Grupo {letra}: hashes completos das configurações não disponíveis.')
        commits = {_normalizar_id(e['manifest'].get('git_commit')) for e in grupo}
        if len(commits - {None}) > 1:
            avisos.append(f'Grupo {letra}: commits Git diferentes; verifique o impacto nas condições.')
        perfis = {_normalizar_id(e['manifest'].get('perfil_sensores')) for e in grupo}
        if len(perfis - {None}) > 1:
            erros.append(f'Grupo {letra}: perfis sensoriais diferentes.')
        elif None in perfis:
            avisos.append(f'Grupo {letra}: perfil sensorial não registrado em todas as repetições.')
        for e in grupo:
            if not booleano(e['m'].get('experimento_intervalo_completo')):
                erros.append(f'Experimento {e["id"]}: intervalo analisado marcado como incompleto.')
    modos = {e['modo'] for e in a+b}
    if len(modos) != 1:
        erros.append('Os grupos devem utilizar a mesma modalidade de movimentação.')
    mapas = {_normalizar_id(e['mapa_id']) for e in a+b}
    if len(mapas) != 1:
        erros.append('Os grupos devem utilizar o mesmo mapa.')
    modo = next(iter(modos)) if len(modos) == 1 else None
    if modo == 'fronteiras':
        duracoes = [_num_manifesto(e, 'duracao_configurada_s', e['m'].get('experimento_duracao_configurada_s')) for e in a+b]
        if None in duracoes or max(duracoes) - min(duracoes) > 1e-6:
            erros.append('Os grupos possuem durações programadas diferentes ou ausentes.')
    elif modo == 'rota_fixa':
        hashes = {_normalizar_id(e['rota_sha256']) for e in a+b}
        if len(hashes) != 1:
            erros.append('Os grupos não utilizaram exatamente a mesma rota (SHA-256).')
    # SDF: preferir hash real do arquivo usado e não apenas número do mapa.
    hashes_sdf = {_normalizar_id(e['manifest'].get('sdf_sha256') or e['manifest'].get('mundo_sha256') or e['m'].get('geometria_sha256_sdf')) for e in a+b}
    if len(hashes_sdf - {None}) > 1:
        erros.append('Os experimentos usam arquivos SDF de hashes diferentes.')
    elif None in hashes_sdf:
        avisos.append('Não foi possível confirmar o hash do SDF em todas as execuções.')
    alinhamentos = {_normalizar_id(e['m'].get('experimento_alinhamento')) for e in a+b}
    if len(alinhamentos - {None}) > 1:
        erros.append('Alinhamentos de trajetória diferentes entre execuções.')
    if len({e['metodo'] for e in a}) == len({e['metodo'] for e in b}) == 1 and a[0]['metodo'] == b[0]['metodo']:
        avisos.append('Ambos os grupos usam o mesmo método; a comparação é entre dois conjuntos de repetições, não entre métodos.')
    if any(len(g) < 5 for g in [a,b]):
        avisos.append('Amostras pequenas: diferenças observadas são descritivas, sem teste de significância estatística.')
    avisos.append('A comparação usa uma métrica agregada por execução, com peso igual para cada repetição; não agrega amostras temporais como se fossem repetições independentes.')
    avisos.append('Repetições independentes não são automaticamente pareadas por ordem; trajetórias autônomas podem diferir.')
    # Deduplicar na ordem original.
    return list(dict.fromkeys(erros)), list(dict.fromkeys(avisos))


def _estatisticas_grupo(exps, chave):
    pares = []
    for e in exps:
        # Indicadores geométricos só entram quando a avaliação individual
        # reportou explicitamente status válido.
        if chave.startswith('geometria_') and str(e['m'].get('geometria_status')) != 'ok':
            continue
        v = val(e['m'].get(chave))
        if np.isfinite(v):
            pares.append((e['id'], float(v)))
    a = np.asarray([v for _,v in pares], dtype=float)
    n = len(a)
    return {'n': n, 'media': float(a.mean()) if n else np.nan,
            'desvio': float(a.std(ddof=1)) if n >= 2 else np.nan,
            'mediana': float(np.median(a)) if n else np.nan,
            'minimo': float(a.min()) if n else np.nan,
            'maximo': float(a.max()) if n else np.nan,
            'cv_pct': (float(a.std(ddof=1) / abs(a.mean()) * 100) if n >= 2 and abs(a.mean()) > 1e-12 else np.nan),
            'pares': pares}


def comparar_grupos(ws, ids_a, ids_b, saida=None):
    todos = ids_a + ids_b
    if set(ids_a) & set(ids_b):
        raise ValueError('Os grupos não podem compartilhar números de experimento.')
    if any(x < 0 for x in todos) or len(set(todos)) != len(todos):
        raise ValueError('IDs de experimentos devem ser distintos e não negativos.')
    a = [carregar(ws, i) for i in ids_a]
    b = [carregar(ws, i) for i in ids_b]
    erros, avisos = _validar_grupos(a, b)
    if erros:
        raise ValueError('Condições experimentais incompatíveis:\n - ' + '\n - '.join(erros))
    modo = a[0]['modo']
    metricas = METRICAS_ROTA if modo == 'rota_fixa' else METRICAS
    if any(str(e['m'].get('geometria_status')) != 'ok' for e in a+b):
        avisos.append('Avaliação geométrica ausente ou inválida em pelo menos uma execução; métricas de geometria usam apenas registros com status ok.')
    # Não comparar campos meramente administrativos como métricas de desempenho.
    metricas = [(c,t,u,d) for c,t,u,d in metricas if c not in {
        'experimento_duracao_configurada_s', 'experimento_duracao_analisada_s'}]
    if saida:
        if not re.fullmatch(r'[\w-]+', saida, re.UNICODE):
            raise ValueError('--saida aceita somente letras, números, _ e -')
        pasta = ws / 'results' / 'comparacoes' / saida
        if pasta.exists():
            raise FileExistsError(f'Diretório já existe: {pasta}')
    else:
        base = ws / 'results' / 'comparacoes' / (
            'grupos_A_' + '_'.join(str(i) for i in ids_a) + '_B_' +
            '_'.join(str(i) for i in ids_b) + '_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
        pasta = base
        n = 1
        while pasta.exists():
            pasta = Path(str(base) + f'_{n}')
            n += 1
    pasta.mkdir(parents=True, exist_ok=False)
    imgdir = pasta / 'graficos'
    imgdir.mkdir()
    nome_a = METODOS.get(a[0]['metodo'], a[0]['metodo'])
    nome_b = METODOS.get(b[0]['metodo'], b[0]['metodo'])
    tabela = []
    valores = []
    metricas_grafico = []
    for chave,titulo,unidade,direcao in metricas:
        x = _estatisticas_grupo(a, chave)
        y = _estatisticas_grupo(b, chave)
        if x['n'] == 0 and y['n'] == 0:
            continue
        delta = y['media']-x['media'] if x['n'] and y['n'] else np.nan
        rel = 100*delta / abs(x['media']) if x['n'] and y['n'] and abs(x['media']) > 1e-12 else np.nan
        vencedor = '—'
        if x['n'] and y['n'] and direcao in ('menor', 'maior'):
            if np.isclose(x['media'], y['media'], atol=1e-12, rtol=1e-9):
                vencedor = 'Empate numérico'
            elif (x['media'] < y['media']) == (direcao == 'menor'):
                vencedor = nome_a
            else:
                vencedor = nome_b
        tabela.append({'chave':chave, 'metrica':titulo, 'unidade':unidade, 'direcao':direcao,
                       'A_n':x['n'], 'A_media':x['media'], 'A_desvio_padrao':x['desvio'],
                       'A_cv_pct':x['cv_pct'], 'A_mediana':x['mediana'],
                       'A_min':x['minimo'], 'A_max':x['maximo'],
                       'B_n':y['n'], 'B_media':y['media'], 'B_desvio_padrao':y['desvio'],
                       'B_cv_pct':y['cv_pct'], 'B_mediana':y['mediana'],
                       'B_min':y['minimo'], 'B_max':y['maximo'],
                       'diferenca_B_menos_A':delta, 'variacao_relativa_B_vs_A_pct':rel,
                       'melhor_media_descritiva':vencedor})
        for letra, s in [('A', x), ('B', y)]:
            for idx,v in s['pares']:
                valores.append({'grupo':letra,'experimento':idx,'metodo':a[0]['metodo'] if letra == 'A' else b[0]['metodo'],
                                'metrica':chave,'valor':v,'unidade':unidade})
        if x['n'] >= 1 and y['n'] >= 1 and chave in {
            'slam_ATE_m_RMSE', 'slam_ATE_angular_graus_RMSE',
            'slam_RPE_1s_translacao_m_RMSE','rota_tempo_execucao_s',
            'rota_distancia_real_m','mapa_variacao_area_conhecida_m2',
            'geometria_iou_conhecido','geometria_iou_global',
            'geometria_erro_bordas_medio_m','geometria_area_falsos_ocupados_m2'}:
            metricas_grafico.append((chave,titulo,unidade,x,y))
    if not tabela:
        raise ValueError('Nenhuma métrica numérica encontrada para comparar os grupos.')
    pd.DataFrame(tabela).to_csv(pasta/'estatisticas_grupos.csv', index=False)
    pd.DataFrame(valores).to_csv(pasta/'valores_individuais_grupos.csv', index=False)
    # Graficos: pontos por repetição, barra da média e intervalo ±1 desvio padrão.
    figs=[]
    for chave,titulo,unidade,x,y in metricas_grafico[:8]:
        fig, ax = plt.subplots(figsize=(8.5,4.3))
        for pos, s, nome in [(0,x,nome_a),(1,y,nome_b)]:
            data = [v for _,v in s['pares']]
            if not data:
                continue
            offsets = np.linspace(-0.085, 0.085, len(data)) if len(data)>1 else [0.0]
            ax.scatter(np.asarray(offsets)+pos, data, s=48, zorder=4, label='Repetições' if pos == 0 else None)
            ax.plot([pos-0.23,pos+0.23], [s['media']]*2, linewidth=2.5,
                    label='Média' if pos == 0 else None)
            if np.isfinite(s['desvio']):
                ax.errorbar(pos, s['media'], yerr=s['desvio'], capsize=5, fmt='none',
                            linewidth=1.6, label='± 1 DP' if pos == 0 else None)
        ax.set_xticks([0,1], [f'A — {nome_a}', f'B — {nome_b}'])
        ax.set_ylabel(unidade or 'Valor')
        ax.set_title(titulo)
        ax.grid(alpha=.25, axis='y')
        ax.legend(loc='best', fontsize=8)
        fig.tight_layout()
        filename='grupos_'+re.sub(r'[^\w-]','_',chave)+'.png'
        fig.savefig(imgdir/filename, dpi=150)
        plt.close(fig)
        figs.append((filename,titulo))
    # Transparência das repetições e limites científicos.
    cabecalho = ['metrica','unidade','A','B','Diferença B−A','Melhor média*']
    linhas=[]
    for r in tabela:
        def resumo(k):
            s = numero(r[k+'_media'],4)
            dp = numero(r[k+'_desvio_padrao'],4)
            cv = numero(r[k+'_cv_pct'],1)
            return f'{s} ± {dp} (n={r[k+"_n"]}; CV={cv}%)'
        linhas.append('<tr><td>'+escape(r['metrica'])+'</td><td>'+escape(r['unidade'])+'</td>'+ 
                      '<td>'+escape(resumo('A'))+'</td><td>'+escape(resumo('B'))+'</td>'+ 
                      '<td>'+escape(numero(r['diferenca_B_menos_A'],4))+'</td><td>'+escape(r['melhor_media_descritiva'])+'</td></tr>')
    imagens = ''.join('<section class="grafico"><h3>'+escape(t)+'</h3><img src="graficos/'+escape(f)+'"></section>' for f,t in figs)
    aviso_html=''.join('<li>'+escape(w)+'</li>' for w in avisos)
    html = """<!doctype html><html lang="pt-BR"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Comparação entre grupos SLAM</title>
<style>body{font:15px/1.5 system-ui,Arial,sans-serif;background:#f5f7fa;color:#263445;margin:0}header{background:#163d61;color:white;padding:30px 5vw}main{max-width:1200px;margin:auto;padding:20px}section{background:white;padding:24px;margin:18px 0;border-radius:10px;border:1px solid #dbe2e9}table{border-collapse:collapse;width:100%;font-size:13px}td,th{padding:9px;border-bottom:1px solid #ddd;text-align:left}th{background:#eef3f8}img{max-width:100%;height:auto;display:block;margin:0 auto}.scroll{overflow-x:auto}.grafico{break-inside:avoid}small{color:#596e80}@page{size:A4;margin:12mm}</style></head><body>"""
    html += '<header><h1>Comparação entre grupos de repetições</h1><p>Mapa '+escape(str(a[0]['mapa_id']))+' · '+escape(modo)+'</p></header><main>'
    html += '<section><h2>Grupos experimentais</h2><p><b>A — '+escape(nome_a)+'</b>: '+escape(', '.join(e['id'] for e in a))+'</p><p><b>B — '+escape(nome_b)+'</b>: '+escape(', '.join(e['id'] for e in b))+'</p><p>Cada execução tem o mesmo peso na média. O sinal da diferença é B − A.</p></section>'
    html += '<section><h2>Média, dispersão e diferença</h2><div class="scroll"><table><thead><tr>'+''.join('<th>'+escape(c)+'</th>' for c in cabecalho)+'</tr></thead><tbody>'+''.join(linhas)+'</tbody></table></div><p><small>* Melhor média é uma indicação numérica descritiva, não significância estatística; métricas informativas não são classificadas. DP: desvio-padrão amostral.</small></p></section>'
    html += '<section><h2>Cuidados metodológicos</h2><ul>'+aviso_html+'</ul></section>'+imagens
    html += '<section><h2>Arquivos</h2><p><code>estatisticas_grupos.csv</code>: médias e dispersões; <code>valores_individuais_grupos.csv</code>: dados de cada repetição; <code>configuracao_grupos.json</code>: identificação e verificações.</p></section></main></body></html>'
    (pasta/'relatorio_comparativo.html').write_text(html,encoding='utf-8')
    (pasta/'configuracao_grupos.json').write_text(json.dumps({
        'grupo_a':[e['num'] for e in a], 'grupo_b':[e['num'] for e in b],
        'metodo_a':a[0]['metodo'],'metodo_b':b[0]['metodo'],'mapa':a[0]['mapa_id'],
        'movimentacao':modo,'avisos':avisos,'definicao_diferenca':'media_B - media_A',
        'pareamento':False},ensure_ascii=False,indent=2,default=str)+'\n',encoding='utf-8')
    return pasta, avisos


def main():
    ap = argparse.ArgumentParser(description="Relatório HTML comparativo de experimentos SLAM já analisados; não depende do ROS 2.")
    ap.add_argument("experimentos", nargs="*", type=int, help="Números dos experimentos para comparação individual (ex.: 21 22 23)")
    ap.add_argument("--grupo-a", nargs="+", type=int, metavar="ID", help="Repetições do grupo A (ex.: 1 2 3)")
    ap.add_argument("--grupo-b", nargs="+", type=int, metavar="ID", help="Repetições do grupo B (ex.: 4 5 6)")
    ap.add_argument("--workspace", type=Path, default=Path.home()/"tcc_slam_ws", help="Caminho para o workspace principal")
    ap.add_argument("--saida", help="Nome da pasta em results/comparacoes/ (não substitui pastas existentes)")
    args=ap.parse_args()
    agrupado = args.grupo_a is not None or args.grupo_b is not None
    if agrupado:
        if args.experimentos or args.grupo_a is None or args.grupo_b is None:
            ap.error('Use --grupo-a ... --grupo-b ... sem IDs posicionais.')
        if len(args.grupo_a)<2 or len(args.grupo_b)<2:
            ap.error('Cada grupo precisa de pelo menos duas repetições.')
        pasta, avisos = comparar_grupos(args.workspace.expanduser().resolve(), args.grupo_a, args.grupo_b, args.saida)
    else:
        if len(args.experimentos)<2:
            ap.error('Informe pelo menos dois experimentos ou --grupo-a e --grupo-b.')
        if len(set(args.experimentos))!=len(args.experimentos) or any(x<0 for x in args.experimentos):
            ap.error('Os números devem ser diferentes e não negativos.')
        pasta, avisos = comparar(args.workspace.expanduser().resolve(),args.experimentos,args.saida)
    # TCC_PDF_AUTOMATICO_V1
    try:
        import subprocess
        import sys
        exportador_pdf = args.workspace.expanduser().resolve() / "exportar_pdf.py"
        if exportador_pdf.is_file():
            resultado_pdf = subprocess.run(
                [sys.executable, str(exportador_pdf), str(pasta / "relatorio_comparativo.html")],
                check=False,
            )
            if resultado_pdf.returncode != 0:
                print("AVISO: PDF indisponivel; HTML e CSV preservados.")
    except OSError as exc:
        print(f"AVISO: nao foi possivel gerar PDF: {exc}")

    print("\nCOMPARAÇÃO CONCLUÍDA")
    print(f"Relatório HTML: {pasta/'relatorio_comparativo.html'}")
    if agrupado:
        print(f"Estatísticas dos grupos: {pasta/'estatisticas_grupos.csv'}")
        print(f"Valores individuais: {pasta/'valores_individuais_grupos.csv'}")
    else:
        print(f"Métricas: {pasta/'metricas_comparadas.csv'}")
    for aviso in avisos:
        print(f"AVISO: {aviso}")
    print(f"\nAbra com: firefox '{pasta/'relatorio_comparativo.html'}'")

if __name__=="__main__":
    try:
        main()
    except (OSError,ValueError,RuntimeError) as exc:
        sys.exit(f"ERRO: {exc}")
