#!/usr/bin/env python3

import argparse
import json
import math
from html import escape
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

import numpy as np
import pandas as pd

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ============================================================
# FUNÇÕES MATEMÁTICAS
# ============================================================

def normalizar_angulo(angulo):
    return (angulo + np.pi) % (2 * np.pi) - np.pi


def quaternion_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def timestamp_ros(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def estatisticas(valores, prefixo):
    valores = np.asarray(valores, dtype=float)
    valores = valores[np.isfinite(valores)]

    if len(valores) == 0:
        return {
            f"{prefixo}_RMSE": np.nan,
            f"{prefixo}_medio": np.nan,
            f"{prefixo}_mediano": np.nan,
            f"{prefixo}_minimo": np.nan,
            f"{prefixo}_maximo": np.nan,
            f"{prefixo}_desvio_padrao": np.nan,
            f"{prefixo}_P95": np.nan,
        }

    return {
        f"{prefixo}_RMSE": float(
            np.sqrt(np.mean(valores ** 2))
        ),
        f"{prefixo}_medio": float(np.mean(valores)),
        f"{prefixo}_mediano": float(np.median(valores)),
        f"{prefixo}_minimo": float(np.min(valores)),
        f"{prefixo}_maximo": float(np.max(valores)),
        f"{prefixo}_desvio_padrao": float(
            np.std(valores, ddof=1)
        ) if len(valores) > 1 else 0.0,
        f"{prefixo}_P95": float(np.percentile(valores, 95)),
    }


def preparar_dataframe(
    registros,
    colunas,
    remover_duplicados=True,
):
    if not registros:
        return pd.DataFrame(columns=colunas)

    df = pd.DataFrame(registros)

    if "tempo_s" in df.columns:

        df = df.sort_values("tempo_s")

        if remover_duplicados:
            df = df.drop_duplicates(
                subset="tempo_s",
                keep="last",
            )

        df = df.reset_index(drop=True)

    return df


def interpolar_pose(df, tempos, tolerancia=0.5):
    """
    Interpola x, y e theta.

    Não permite extrapolação nem interpolação através
    de lacunas maiores que 'tolerancia' segundos.
    """
    tempos = np.asarray(tempos, dtype=float)

    if len(df) < 2:
        return pd.DataFrame()

    t = df["tempo_s"].to_numpy(dtype=float)

    indices = np.searchsorted(t, tempos, side="left")
    direita = np.clip(indices, 0, len(t) - 1)
    esquerda = np.clip(indices - 1, 0, len(t) - 1)

    # Amostras exatamente coincidentes são permitidas.
    exato = np.abs(t[direita] - tempos) < 1e-6

    valido = (
        (tempos >= t[0])
        & (tempos <= t[-1])
        & (
            exato
            | (
                (direita > esquerda)
                & ((t[direita] - t[esquerda]) <= tolerancia)
            )
        )
    )

    resultado = pd.DataFrame({
        "tempo_s": tempos,
        "x": np.full(len(tempos), np.nan),
        "y": np.full(len(tempos), np.nan),
        "theta": np.full(len(tempos), np.nan),
    })

    if not np.any(valido):
        return resultado

    tv = tempos[valido]

    resultado.loc[valido, "x"] = np.interp(
        tv, t, df["x"]
    )
    resultado.loc[valido, "y"] = np.interp(
        tv, t, df["y"]
    )

    theta_continuo = np.unwrap(
        df["theta"].to_numpy(dtype=float)
    )

    resultado.loc[valido, "theta"] = normalizar_angulo(
        np.interp(tv, t, theta_continuo)
    )

    return resultado


def compor_poses(a, b):
    """
    Composição de transformações planas SE(2).

    a = (x, y, theta)
    b = (x, y, theta)
    """
    ax, ay, at = a
    bx, by, bt = b

    return (
        ax + math.cos(at) * bx - math.sin(at) * by,
        ay + math.sin(at) * bx + math.cos(at) * by,
        normalizar_angulo(at + bt),
    )


def inversa_pose(p):
    x, y, theta = p

    c = math.cos(theta)
    s = math.sin(theta)

    return (
        -c * x - s * y,
        s * x - c * y,
        normalizar_angulo(-theta),
    )


# ============================================================
# LEITURA DA ROSBAG
# ============================================================

def ler_rosbag(caminho, storage_id):
    """
    Lê os tópicos necessários em uma única passagem.

    Para Odometry e TF, utiliza header.stamp.
    Para OccupancyGrid, utiliza header.stamp.
    Para eventos, utiliza o timestamp JSON publicado
    pelo Frontier Explorer.
    """

    reader = rosbag2_py.SequentialReader()

    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(caminho),
            storage_id=storage_id,
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    tipos = {
        item.name: item.type
        for item in reader.get_all_topics_and_types()
    }

    desejados = {
        "/ground_truth/odom",
        "/odom",
        "/tf",
        "/tf_static",
        "/map",
        "/exploration/events",
        "/clock",
    }

    classes = {
        nome: get_message(tipo)
        for nome, tipo in tipos.items()
        if nome in desejados
    }

    odometrias = {
        "/ground_truth/odom": [],
        "/odom": [],
    }

    transformacoes = []
    mapas = []
    mapa_final = None
    eventos = []
    clocks = []

    frames = {}

    while reader.has_next():

        topico, dados, timestamp_bag = reader.read_next()

        if topico not in classes:
            continue

        msg = deserialize_message(
            dados, classes[topico]
        )

        if topico in odometrias:

            tempo = timestamp_ros(msg.header.stamp)

            if tempo <= 0:
                continue

            frames.setdefault(
                topico, set()
            ).add(msg.header.frame_id)

            odometrias[topico].append({
                "tempo_s": tempo,
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "theta": quaternion_yaw(
                    msg.pose.pose.orientation
                ),
            })

        elif topico in ("/tf", "/tf_static"):

            for tr in msg.transforms:

                tempo = timestamp_ros(
                    tr.header.stamp
                )

                if tempo <= 0:
                    continue

                transformacoes.append({
                    "tempo_s": tempo,
                    "pai": tr.header.frame_id.lstrip("/"),
                    "filho": tr.child_frame_id.lstrip("/"),
                    "x": tr.transform.translation.x,
                    "y": tr.transform.translation.y,
                    "theta": quaternion_yaw(
                        tr.transform.rotation
                    ),
                    "estatica": topico == "/tf_static",
                })

        elif topico == "/map":

            tempo = timestamp_ros(msg.header.stamp)

            if tempo <= 0:
                continue

            dados_mapa = np.asarray(
                msg.data, dtype=np.int16
            )

            resolucao = float(msg.info.resolution)
            # Preserva somente a grade mais recente; não guarda cópias
            # de todos os mapas na memória.
            mapa_final = {
                "tempo_s": tempo,
                "data": dados_mapa.copy().reshape((msg.info.height, msg.info.width)),
                "resolucao": resolucao,
                "origem_x": float(msg.info.origin.position.x),
                "origem_y": float(msg.info.origin.position.y),
                "frame_id": msg.header.frame_id,
            }

            conhecidos = int(
                np.count_nonzero(dados_mapa >= 0)
            )

            livres = int(
                np.count_nonzero(dados_mapa == 0)
            )

            ocupados = int(
                np.count_nonzero(dados_mapa > 0)
            )

            mapas.append({
                "tempo_s": tempo,
                "area_conhecida_m2":
                    conhecidos * resolucao ** 2,
                "area_livre_m2":
                    livres * resolucao ** 2,
                "area_ocupada_m2":
                    ocupados * resolucao ** 2,
                "celulas_conhecidas": conhecidos,
                "celulas_desconhecidas": int(
                    np.count_nonzero(dados_mapa == -1)
                ),
                "resolucao_m": resolucao,
                "largura": msg.info.width,
                "altura": msg.info.height,
            })

        elif topico == "/exploration/events":

            try:
                evento = json.loads(msg.data)

                if "event" not in evento:
                    continue

                tempo = evento.get(
                    "timestamp",
                    timestamp_bag * 1e-9
                )

                evento["tempo_s"] = float(tempo)

                eventos.append(evento)

            except (ValueError, TypeError):
                continue

        elif topico == "/clock":

            clocks.append(
                timestamp_ros(msg.clock)
            )

    referencia = preparar_dataframe(
        odometrias["/ground_truth/odom"],
        ["tempo_s", "x", "y", "theta"],
    )

    odom = preparar_dataframe(
        odometrias["/odom"],
        ["tempo_s", "x", "y", "theta"],
    )

    tf = preparar_dataframe(
        transformacoes,
        [
            "tempo_s",
            "pai",
            "filho",
            "x",
            "y",
            "theta",
            "estatica",
        ],
        remover_duplicados=False,
    )
    
    mapas = preparar_dataframe(
        mapas,
        ["tempo_s", "area_conhecida_m2",
         "area_livre_m2", "area_ocupada_m2"],
    )

    eventos = preparar_dataframe(
        eventos,
        ["tempo_s", "event"],
    )

    return {
        "referencia": referencia,
        "odom": odom,
        "tf": tf,
        "mapas": mapas,
        "mapa_final": mapa_final,
        "eventos": eventos,
        "clocks": clocks,
        "frames": frames,
        "topicos": tipos,
    }


# ============================================================
# RECONSTRUÇÃO DA TRAJETÓRIA DO SLAM
# ============================================================

def reconstruir_slam(tf, tolerancia=0.5):
    """
    Reconstrói a pose map -> base_link:

        map -> odom -> base_link

    A composição usa as transformações publicadas em /tf.
    """

    if tf.empty:
        return pd.DataFrame()

    map_odom = tf[
        (tf["pai"] == "map")
        & (tf["filho"] == "odom")
    ].copy()

    odom_base = tf[
        (tf["pai"] == "odom")
        & (tf["filho"] == "base_link")
    ].copy()

    if len(map_odom) < 1 or len(odom_base) < 2:
        return pd.DataFrame()

    map_odom = (
        map_odom.sort_values("tempo_s")
        .drop_duplicates("tempo_s", keep="last")
    )

    odom_base = (
        odom_base.sort_values("tempo_s")
        .drop_duplicates("tempo_s", keep="last")
    )

    tempos = map_odom["tempo_s"].to_numpy()

    base_interp = interpolar_pose(
        odom_base,
        tempos,
        tolerancia,
    )

    registros = []

    for i, (_, linha) in enumerate(map_odom.iterrows()):

        base = base_interp.iloc[i]

        if not np.isfinite(base["x"]):
            continue

        pose = compor_poses(
            (
                linha["x"],
                linha["y"],
                linha["theta"],
            ),
            (
                base["x"],
                base["y"],
                base["theta"],
            ),
        )

        registros.append({
            "tempo_s": linha["tempo_s"],
            "x": pose[0],
            "y": pose[1],
            "theta": pose[2],
        })

    return preparar_dataframe(
        registros,
        ["tempo_s", "x", "y", "theta"],
    )


# ============================================================
# ALINHAMENTO DAS TRAJETÓRIAS
# ============================================================

def alinhar_trajetorias(
    referencia,
    estimada,
    inicio,
    fim,
    tolerancia=0.5,
    alinhamento="inicial",
):
    """
    Compara trajetórias nos mesmos instantes.

    alinhamento='inicial':
        Aplica uma transformação SE(2) constante para
        coincidir a primeira pose estimada com a referência.

    alinhamento='nenhum':
        Não altera os referenciais.
        Usar somente quando já forem equivalentes.
    """

    if len(referencia) < 2 or len(estimada) < 2:
        return pd.DataFrame()

    ref = referencia[
        (referencia["tempo_s"] >= inicio)
        & (referencia["tempo_s"] <= fim)
    ].copy()

    if len(ref) < 2:
        return pd.DataFrame()

    est = interpolar_pose(
        estimada,
        ref["tempo_s"].to_numpy(),
        tolerancia,
    )

    valido = est["x"].notna().to_numpy()

    ref = ref.iloc[
        np.flatnonzero(valido)
    ].reset_index(drop=True)

    est = est.iloc[
        np.flatnonzero(valido)
    ].reset_index(drop=True)

    if len(ref) < 2:
        return pd.DataFrame()

    if alinhamento == "inicial":

        pose_ref = tuple(
            ref.loc[0, ["x", "y", "theta"]]
        )

        pose_est = tuple(
            est.loc[0, ["x", "y", "theta"]]
        )

        transformacao = compor_poses(
            pose_ref,
            inversa_pose(pose_est),
        )

        poses_alinhadas = [
            compor_poses(
                transformacao,
                (linha.x, linha.y, linha.theta),
            )
            for linha in est.itertuples()
        ]

        est[["x", "y", "theta"]] = np.asarray(
            poses_alinhadas
        )

    tabela = pd.DataFrame({
        "tempo_s": ref["tempo_s"],
        "x_ref": ref["x"],
        "y_ref": ref["y"],
        "theta_ref": ref["theta"],
        "x_est": est["x"],
        "y_est": est["y"],
        "theta_est": est["theta"],
    })

    tabela["erro_x_m"] = (
        tabela["x_est"] - tabela["x_ref"]
    )

    tabela["erro_y_m"] = (
        tabela["y_est"] - tabela["y_ref"]
    )

    tabela["erro_posicao_m"] = np.hypot(
        tabela["erro_x_m"],
        tabela["erro_y_m"],
    )

    tabela["erro_angular_rad"] = normalizar_angulo(
        tabela["theta_est"] - tabela["theta_ref"]
    )

    tabela["erro_angular_graus"] = np.degrees(
        np.abs(tabela["erro_angular_rad"])
    )

    return tabela


# ============================================================
# CÁLCULO DO RPE SE(2)
# ============================================================

def calcular_rpe(tabela, delta_s, tolerancia=0.5):
    """
    Relative Pose Error para intervalo temporal fixo.

    E = inv(inv(Tref_i) * Tref_j)
        * (inv(Test_i) * Test_j)

    Retorna erros de translação e rotação.
    """

    if len(tabela) < 2:
        return np.array([]), np.array([])

    tempos = tabela["tempo_s"].to_numpy()

    ref = pd.DataFrame({
        "tempo_s": tempos,
        "x": tabela["x_ref"],
        "y": tabela["y_ref"],
        "theta": tabela["theta_ref"],
    })

    est = pd.DataFrame({
        "tempo_s": tempos,
        "x": tabela["x_est"],
        "y": tabela["y_est"],
        "theta": tabela["theta_est"],
    })

    tempos_futuros = tempos + delta_s

    ref_futuro = interpolar_pose(
        ref, tempos_futuros, tolerancia
    )

    est_futuro = interpolar_pose(
        est, tempos_futuros, tolerancia
    )

    erros_translacao = []
    erros_rotacao = []

    for i in range(len(tabela)):

        if not np.isfinite(ref_futuro.iloc[i]["x"]):
            continue

        if not np.isfinite(est_futuro.iloc[i]["x"]):
            continue

        r0 = (
            tabela.iloc[i]["x_ref"],
            tabela.iloc[i]["y_ref"],
            tabela.iloc[i]["theta_ref"],
        )

        r1 = (
            ref_futuro.iloc[i]["x"],
            ref_futuro.iloc[i]["y"],
            ref_futuro.iloc[i]["theta"],
        )

        e0 = (
            tabela.iloc[i]["x_est"],
            tabela.iloc[i]["y_est"],
            tabela.iloc[i]["theta_est"],
        )

        e1 = (
            est_futuro.iloc[i]["x"],
            est_futuro.iloc[i]["y"],
            est_futuro.iloc[i]["theta"],
        )

        movimento_ref = compor_poses(
            inversa_pose(r0), r1
        )

        movimento_est = compor_poses(
            inversa_pose(e0), e1
        )

        erro = compor_poses(
            inversa_pose(movimento_ref),
            movimento_est,
        )

        erros_translacao.append(
            math.hypot(erro[0], erro[1])
        )

        erros_rotacao.append(
            abs(math.degrees(erro[2]))
        )

    return (
        np.asarray(erros_translacao),
        np.asarray(erros_rotacao),
    )


# ============================================================
# MÉTRICAS DE LOCALIZAÇÃO
# ============================================================

def calcular_metricas_trajetoria(tabela, prefixo):

    if tabela.empty:
        return {}

    resultados = {
        f"{prefixo}_numero_amostras": len(tabela),
        f"{prefixo}_duracao_amostrada_s": float(
            tabela["tempo_s"].iloc[-1]
            - tabela["tempo_s"].iloc[0]
        ),
    }

    resultados.update(
        estatisticas(
            tabela["erro_posicao_m"],
            f"{prefixo}_ATE_m",
        )
    )

    resultados.update(
        estatisticas(
            tabela["erro_angular_graus"],
            f"{prefixo}_ATE_angular_graus",
        )
    )

    referencia = tabela[
        ["x_ref", "y_ref"]
    ].to_numpy()

    estimada = tabela[
        ["x_est", "y_est"]
    ].to_numpy()

    distancia_ref = float(
        np.sum(
            np.linalg.norm(
                np.diff(referencia, axis=0),
                axis=1,
            )
        )
    )

    distancia_est = float(
        np.sum(
            np.linalg.norm(
                np.diff(estimada, axis=0),
                axis=1,
            )
        )
    )

    resultados[
        f"{prefixo}_distancia_referencia_m"
    ] = distancia_ref

    resultados[
        f"{prefixo}_distancia_estimada_m"
    ] = distancia_est

    resultados[
        f"{prefixo}_erro_distancia_percentual"
    ] = (
        abs(distancia_est - distancia_ref)
        / distancia_ref * 100
        if distancia_ref > 0
        else np.nan
    )

    for delta in (1.0, 5.0, 10.0):

        erro_t, erro_r = calcular_rpe(
            tabela, delta
        )

        nome = f"{prefixo}_RPE_{int(delta)}s"

        resultados[
            f"{nome}_numero_amostras"
        ] = len(erro_t)

        resultados.update(
            estatisticas(
                erro_t,
                f"{nome}_translacao_m",
            )
        )

        resultados.update(
            estatisticas(
                erro_r,
                f"{nome}_rotacao_graus",
            )
        )

    return resultados


# ============================================================
# MÉTRICAS DE MAPEAMENTO
# ============================================================

def analisar_mapeamento(mapas, inicio, fim):

    if mapas.empty:
        return {}, pd.DataFrame()

    # Mantém a última observação disponível antes do início.
    anteriores = mapas[
        mapas["tempo_s"] <= inicio
    ].tail(1)

    durante = mapas[
        (mapas["tempo_s"] > inicio)
        & (mapas["tempo_s"] <= fim)
    ]

    selecionados = pd.concat(
        [anteriores, durante],
        ignore_index=True,
    )

    if selecionados.empty:
        return {}, pd.DataFrame()

    selecionados = selecionados.copy()

    selecionados["tempo_relativo_s"] = (
        selecionados["tempo_s"] - inicio
    )

    primeira = selecionados.iloc[0]
    ultima = selecionados.iloc[-1]

    area_inicial = float(
        primeira["area_conhecida_m2"]
    )

    area_final = float(
        ultima["area_conhecida_m2"]
    )

    area_nova = area_final - area_inicial

    duracao = fim - inicio

    resultados = {
        "mapa_numero_atualizacoes": len(selecionados),
        "mapa_area_inicial_m2": area_inicial,
        "mapa_area_final_m2": area_final,
        "mapa_variacao_area_conhecida_m2": area_nova,
        "mapa_area_livre_final_m2": float(
            ultima["area_livre_m2"]
        ),
        "mapa_area_ocupada_final_m2": float(
            ultima["area_ocupada_m2"]
        ),
        "mapa_taxa_media_m2_s": (
            area_nova / duracao
            if duracao > 0 else np.nan
        ),
    }

    return resultados, selecionados


# ============================================================
# MÉTRICAS DE EXPLORAÇÃO
# ============================================================

def analisar_exploracao(eventos, inicio, fim):
    """Métricas de exploração V1-V4, com cancelamentos sem falsa falha.

    Os eventos V2+ podem conter goal_id. Para gravações legadas, o
    pareamento conserva uma estratégia sequencial, já que o explorador
    antigo enviava apenas uma meta por vez.
    """
    if eventos.empty:
        return {}, pd.DataFrame()

    eventos = eventos[(eventos["tempo_s"] >= inicio)
                      & (eventos["tempo_s"] <= fim)].copy()
    eventos["tempo_relativo_s"] = eventos["tempo_s"] - inicio
    if eventos.empty:
        return {}, eventos

    eventos = eventos.sort_values("tempo_s", kind="stable")
    contagem = eventos["event"].value_counts()
    enviados = int(contagem.get("GOAL_SENT", 0))
    alcancados = int(contagem.get("GOAL_REACHED", 0))
    falhos = int(contagem.get("GOAL_FAILED", 0) +
                  contagem.get("GOAL_ERROR", 0))
    rejeitados = int(contagem.get("GOAL_REJECTED", 0))
    cancelados = int(contagem.get("GOAL_CANCELLED", 0))
    terminados = alcancados + falhos + rejeitados + cancelados
    concluidos_sem_cancelamento = alcancados + falhos + rejeitados

    resultados = {
        "exploracao_objetivos_enviados": enviados,
        "exploracao_objetivos_alcancados": alcancados,
        "exploracao_objetivos_falhos": falhos,
        "exploracao_objetivos_rejeitados": rejeitados,
        "exploracao_objetivos_cancelados": cancelados,
        "exploracao_objetivos_sem_resultado": max(0, enviados - terminados),
        "exploracao_fronteiras_selecionadas": int(
            contagem.get("FRONTIER_SELECTED", 0)),
        "exploracao_reavaliacoes": int(contagem.get("GOAL_REEVALUATED", 0)),
        "exploracao_trocas_meta": int(contagem.get("GOAL_SWITCHED", 0)),
        "exploracao_solicitacoes_troca": int(
            contagem.get("GOAL_SWITCH_REQUESTED", 0)),
        "exploracao_trocas_abortadas": int(
            contagem.get("GOAL_SWITCH_ABORTED", 0)),
        # Mantém a definição histórica de sucesso entre as metas que
        # encerraram com sucesso/falha, excluindo cancelamentos voluntários.
        "exploracao_taxa_sucesso_percentual": (
            100 * alcancados / concluidos_sem_cancelamento
            if concluidos_sem_cancelamento else np.nan
        ),
        "exploracao_taxa_alcance_sobre_enviadas_percentual": (
            100 * alcancados / enviados if enviados else np.nan
        ),
    }

    # Pareamento por identificador quando presente (V2-V4); fallback para
    # mensagens antigas sem identificador. Um cancelamento é terminal,
    # mas não é contabilizado como falha.
    inicio_por_id = {}
    inicio_legado = None
    duracoes = []
    finais = {"GOAL_REACHED", "GOAL_FAILED", "GOAL_ERROR",
              "GOAL_REJECTED", "GOAL_CANCELLED"}
    for linha in eventos.to_dict("records"):
        event = linha.get("event")
        goal_id = linha.get("goal_id")
        stamp = float(linha["tempo_s"])
        if event == "GOAL_SENT":
            if goal_id is None or (isinstance(goal_id, float) and
                                   np.isnan(goal_id)):
                inicio_legado = stamp
            else:
                inicio_por_id[str(goal_id)] = stamp
        elif event in finais:
            if goal_id is None or (isinstance(goal_id, float) and
                                   np.isnan(goal_id)):
                start = inicio_legado
                inicio_legado = None
            else:
                start = inicio_por_id.pop(str(goal_id), None)
            if start is not None and stamp >= start:
                duracoes.append(stamp - start)

    resultados.update(estatisticas(duracoes,
                                   "exploracao_tempo_por_objetivo_s"))
    return resultados, eventos


# ============================================================
# QUALIDADE DOS DADOS
# ============================================================

def analisar_qualidade(dados, inicio, fim):

    resultados = {}

    for nome in ("referencia", "odom"):

        df = dados[nome]

        df = df[
            (df["tempo_s"] >= inicio)
            & (df["tempo_s"] <= fim)
        ]

        resultados[
            f"qualidade_{nome}_amostras"
        ] = len(df)

        if len(df) > 1:

            intervalos = np.diff(
                df["tempo_s"].to_numpy()
            )

            resultados[
                f"qualidade_{nome}_intervalo_maximo_s"
            ] = float(np.max(intervalos))

            resultados[
                f"qualidade_{nome}_frequencia_media_hz"
            ] = float(
                (len(df) - 1)
                / (
                    df["tempo_s"].iloc[-1]
                    - df["tempo_s"].iloc[0]
                )
            )

    clocks = np.asarray(
        dados["clocks"], dtype=float
    )

    resultados[
        "qualidade_clock_retornos"
    ] = int(
        np.count_nonzero(np.diff(clocks) < -0.001)
    ) if len(clocks) > 1 else 0

    for topico, frames in dados["frames"].items():

        nome = topico.strip("/").replace("/", "_")

        resultados[
            f"qualidade_frame_{nome}"
        ] = ";".join(sorted(frames))

    return resultados


# ============================================================
# RELATORIO VISUAL OFFLINE
# ============================================================

def gerar_relatorio_visual(pasta, resultados, tabela_odom, tabela_slam,
                          evolucao, eventos, mapa_final, inicio):
    """Cria HTML e PNG a partir da rosbag, sem depender do RViz aberto."""
    pasta = Path(pasta)
    pasta.mkdir(parents=True, exist_ok=True)
    imagens = pasta / "graficos"
    imagens.mkdir(exist_ok=True)
    figuras = []

    def salvar(fig, arquivo, titulo, interpretacao):
        destino = imagens / arquivo
        fig.savefig(destino, dpi=155, bbox_inches="tight")
        plt.close(fig)
        figuras.append(("graficos/" + arquivo, titulo, interpretacao))

    metodo = str(resultados.get("experimento_metodo_slam", "nao_informado"))

    # 1. Trajetorias alinhadas: a comparacao e da geometria, nao da rota do Nav2.
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(tabela_odom["x_ref"], tabela_odom["y_ref"],
            label="Ground truth", linewidth=2)
    ax.plot(tabela_odom["x_est"], tabela_odom["y_est"],
            label="Odometria", linewidth=1.3, alpha=.85)
    if not tabela_slam.empty:
        ax.plot(tabela_slam["x_est"], tabela_slam["y_est"],
                label="SLAM (" + metodo.upper() + ")", linewidth=1.4)
    ax.set(title="Trajetorias no plano XY", xlabel="X (m)", ylabel="Y (m)")
    ax.axis("equal")
    ax.grid(alpha=.25)
    ax.legend()
    salvar(fig, "trajetorias.png", "Trajetórias",
           "Compara o percurso de referência com a odometria e a estimativa global do método SLAM, após alinhamento inicial SE(2).")

    # 2. Erros absolutos de posicao ao longo do tempo
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(tabela_odom["tempo_s"] - inicio, tabela_odom["erro_posicao_m"],
            label="Odometria", linewidth=1.4)
    if not tabela_slam.empty:
        ax.plot(tabela_slam["tempo_s"] - inicio, tabela_slam["erro_posicao_m"],
                label="SLAM (" + metodo.upper() + ")", linewidth=1.4)
    ax.set(title="Erro absoluto de posicao (ATE)", xlabel="Tempo experimental (s)",
           ylabel="Erro de posicao (m)")
    ax.legend(); ax.grid(alpha=.25)
    salvar(fig, "erro_posicao.png", "Evolução do erro de posição",
           "Indica em quais instantes a estimativa se afasta da referência; valores menores são preferíveis.")

    # 3. Erros absolutos angulares
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(tabela_odom["tempo_s"] - inicio, tabela_odom["erro_angular_graus"],
            label="Odometria", linewidth=1.4)
    if not tabela_slam.empty:
        ax.plot(tabela_slam["tempo_s"] - inicio, tabela_slam["erro_angular_graus"],
                label="SLAM (" + metodo.upper() + ")", linewidth=1.4)
    ax.set(title="Erro absoluto angular", xlabel="Tempo experimental (s)",
           ylabel="Erro de orientacao (graus)")
    ax.legend(); ax.grid(alpha=.25)
    salvar(fig, "erro_angular.png", "Evolução do erro angular",
           "Mostra desvios de orientação que podem influenciar a projeção de varreduras e a navegação.")

    # 4. RPE translacional entre janelas temporais
    janelas = [1, 5, 10]
    odom_rpe = [resultados.get(f"odom_RPE_{j}s_translacao_m_RMSE", float("nan")) for j in janelas]
    slam_rpe = [resultados.get(f"slam_RPE_{j}s_translacao_m_RMSE", float("nan")) for j in janelas]
    fig, ax = plt.subplots(figsize=(8, 4))
    xx = np.arange(len(janelas))
    ax.bar(xx - .18, odom_rpe, .35, label="Odometria")
    if np.any(np.isfinite(slam_rpe)):
        ax.bar(xx + .18, slam_rpe, .35, label="SLAM (" + metodo.upper() + ")")
    ax.set_xticks(xx, [f"{j} s" for j in janelas])
    ax.set(ylabel="RPE translacional RMSE (m)", title="Consistencia local da trajetoria")
    ax.legend(); ax.grid(axis="y", alpha=.25)
    salvar(fig, "rpe_translacao.png", "Erro relativo de trajetória (RPE)",
           "Compara a precisão dos deslocamentos relativos em janelas de 1, 5 e 10 segundos. Valores menores indicam maior consistência local.")

    # 5. Evolucao da area conhecida e composicao final
    if not evolucao.empty:
        fig, ax = plt.subplots(figsize=(9, 4))
        ax.plot(evolucao["tempo_relativo_s"], evolucao["area_conhecida_m2"],
                label="Area conhecida", linewidth=2)
        ax.plot(evolucao["tempo_relativo_s"], evolucao["area_livre_m2"],
                label="Area livre", linewidth=1.1)
        ax.plot(evolucao["tempo_relativo_s"], evolucao["area_ocupada_m2"],
                label="Area ocupada (celulas > 0)", linewidth=1.1)
        ax.set(title="Evolucao das areas do mapa", xlabel="Tempo experimental (s)",
               ylabel="Area (m²)")
        ax.legend(); ax.grid(alpha=.25)
        salvar(fig, "evolucao_mapa.png", "Evolução do mapeamento",
               "A área conhecida indica quanto do ambiente foi observado. A curva não mede diretamente a precisão geométrica do mapa.")

    # 6. Eventos da exploracao
    if not eventos.empty:
        tipos = ["GOAL_SENT", "GOAL_REACHED", "GOAL_FAILED", "GOAL_ERROR",
                 "GOAL_REJECTED", "GOAL_CANCELLED", "GOAL_SWITCHED"]
        contagens = [int((eventos["event"] == e).sum()) for e in tipos]
        fig, ax = plt.subplots(figsize=(10, 3.8))
        ax.bar(["Enviados", "Alcançados", "Falhos", "Erros",
                "Rejeitados", "Cancelados", "Trocas"], contagens)
        ax.set(title="Eventos da navegação autônoma", ylabel="Ocorrências")
        ax.tick_params(axis="x", labelrotation=20)
        ax.grid(axis="y", alpha=.25)
        salvar(fig, "eventos.png", "Objetivos de navegação",
               "Distingue metas alcançadas, falhas, rejeições, cancelamentos voluntários e trocas de destino. Cancelamentos voluntários não são falhas.")

    # 7. Mapa final: sem screenshot da interface, fiel aos dados publicados
    mapa_status = "Mapa final indisponivel: o topico /map nao foi registrado."
    if mapa_final is not None:
        dados_m = mapa_final["data"]
        # Em ROS OccupancyGrid -1 e desconhecido, 0 livre, 1..100 ocupado.
        # O mapa usa escala de cinza semelhante à exibição usual no RViz.
        cores = np.empty(dados_m.shape + (3,), dtype=np.uint8)
        cores[:] = (170, 170, 170)  # desconhecido
        conhecido = dados_m >= 0
        cinza = np.clip(255.0 * (1.0 - dados_m[conhecido] / 100.0), 0, 255).astype(np.uint8)
        cores[conhecido] = np.stack([cinza, cinza, cinza], axis=-1)
        # origin fica no canto inferior esquerdo no grid ROS, origin='lower'
        alt, larg = dados_m.shape
        x0, y0 = mapa_final["origem_x"], mapa_final["origem_y"]
        res = mapa_final["resolucao"]
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.imshow(cores, origin="lower", interpolation="nearest",
                  extent=[x0, x0 + larg * res, y0, y0 + alt * res])
        ax.set(title=f"Ultimo mapa /map na rosbag - {metodo.upper()}",
               xlabel=f"X ({mapa_final['frame_id']}, m)",
               ylabel=f"Y ({mapa_final['frame_id']}, m)")
        ax.set_aspect("equal")
        salvar(fig, "mapa_final.png", "Mapa de ocupação final",
               "Imagem gerada da última mensagem /map disponível na rosbag: branco = livre, preto = ocupado, cinza = desconhecido. Não é uma captura da interface do RViz2.")
        # arquivo de acesso direto na raiz da pasta de resultados
        import shutil
        shutil.copy2(imagens / "mapa_final.png", pasta / "mapa_final.png")
        mapa_status = (f"Último mapa registrado em t={mapa_final['tempo_s']:.2f} s "
                       f"(relógio absoluto); referencial {mapa_final['frame_id']}.")

    def numero(chave, casas=3):
        valor = resultados.get(chave)
        try:
            if valor is None or not math.isfinite(float(valor)):
                return "Indisponível"
            return f"{float(valor):.{casas}f}"
        except (ValueError, TypeError):
            return "Indisponível"

    kpis = [
        ("Método", escape(metodo.upper()), "Algoritmo declarado no experimento"),
        ("Duração analisada", numero("experimento_duracao_analisada_s", 1) + " s", "Tempo efetivo disponível"),
        ("ATE SLAM", numero("slam_ATE_m_RMSE") + " m", "RMSE translacional; menor é melhor"),
        ("ATE odometria", numero("odom_ATE_m_RMSE") + " m", "Referência de comparação"),
        ("ATE angular SLAM", numero("slam_ATE_angular_graus_RMSE") + "°", "RMSE da orientação"),
        ("Área acrescentada", numero("mapa_variacao_area_conhecida_m2", 2) + " m²", "Variação da área conhecida"),
        ("Metas alcançadas", numero("exploracao_objetivos_alcancados", 0), "Não inclui metas pendentes"),
        ("Trocas de destino", numero("exploracao_trocas_meta", 0), "Replanejamento dinâmico V4"),
        ("Metas canceladas", numero("exploracao_objetivos_cancelados", 0), "Cancelamento não equivale a falha"),
        ("Execução completa", "SIM" if resultados.get("experimento_intervalo_completo") else "NÃO", "Conferência da duração requerida"),
    ]
    cards = "\n".join(
        f'<div class="card"><div class="card-title">{titulo}</div><div class="number">{valor}</div><small>{nota}</small></div>'
        for titulo, valor, nota in kpis
    )
    blocos = "\n".join(
        f'<section><h2>{escape(titulo)}</h2><p>{escape(nota)}</p><img src="{arquivo}" alt="{escape(titulo)}"></section>'
        for arquivo, titulo, nota in figuras
    )
    notas = (
        "A estimativa SLAM é reconstruída de map→odom→base_link. "
        "As trajetórias são comparadas após alinhamento inicial SE(2). "
        "O mapa final provém da mensagem /map e pode diferir da aparência do RViz2 "
        "se o RViz2 estiver exibindo outro Fixed Frame, camadas ou marcadores. "
        "A ausência de uma métrica não deve ser interpretada como valor zero."
    )
    html = f"""<!doctype html><html lang="pt-BR"><head><meta charset="utf-8">
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <title>Experimento SLAM — {escape(metodo.upper())}</title>
    <style>
    :root {{font-family:system-ui,Arial,sans-serif;color:#192438;background:#f3f6fb}}
    body {{margin:0}}header {{background:#10233f;color:white;padding:42px max(24px,calc((100vw - 1100px)/2))}}
    main {{max-width:1100px;margin:auto;padding:24px}}
    .grid {{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:13px}}
    .card,section {{background:white;border-radius:12px;padding:18px;box-shadow:0 2px 10px #10233f14}}
    .card-title {{font-weight:700;color:#40526a}}.number {{font-size:1.7rem;font-weight:800;margin:8px 0}}
    small,p {{color:#56657a;line-height:1.55}}section {{margin-top:22px}}
    section img {{width:100%;height:auto;display:block;object-fit:contain}}
    h1 {{margin:0 0 8px}}h2 {{font-size:1.25rem}}footer {{padding:24px 0;color:#53677c}}
    </style></head><body>
    <header><h1>Relatório visual de experimento SLAM</h1>
    <div>Método: {escape(metodo.upper())} · Início simulado absoluto: {numero('experimento_inicio_simulado_s',2)} s
    · Duração solicitada: {numero('experimento_duracao_configurada_s',1)} s</div></header>
    <main><div class="grid">{cards}</div>
    <section><h2>Como interpretar</h2><p>{escape(notas)}</p><p>{escape(mapa_status)}</p></section>
    {blocos}<footer>Relatório gerado localmente a partir dos dados da rosbag. Dados tabulares disponíveis nos arquivos CSV do mesmo diretório.</footer>
    </main></body></html>"""
    (pasta / "relatorio.html").write_text(html, encoding="utf-8")
    return pasta / "relatorio.html"


# ============================================================
# EXECUÇÃO PRINCIPAL
# ============================================================

def main():

    parser = argparse.ArgumentParser(
        description=(
            "Análise individual de experimento "
            "SLAM e exploração autônoma."
        )
    )

    parser.add_argument("arquivo_bag")

    parser.add_argument(
        "--storage-id",
        choices=["mcap", "sqlite3"],
        default="mcap",
    )

    parser.add_argument(
        "--duracao",
        type=float,
        default=600.0,
    )

    parser.add_argument(
        "--metodo",
        choices=["toolbox", "graph", "rbpf", "ekf", "nao_informado"],
        default="nao_informado",
        help="Metodo SLAM declarado pelo executor (nao inferido do rosbag)",
    )

    parser.add_argument(
        "--alinhamento",
        choices=["inicial", "nenhum"],
        default="inicial",
    )

    parser.add_argument(
        "--saida-trajetoria",
        default="trajetoria.csv",
    )

    parser.add_argument(
        "--saida-metricas",
        default="metricas.csv",
    )

    args = parser.parse_args()

    if args.duracao <= 0:
        parser.error(
            "A duração deve ser maior que zero."
        )

    bag = Path(args.arquivo_bag)

    if not bag.is_dir():
        raise FileNotFoundError(
            f"Rosbag não encontrada: {bag}"
        )

    saida_trajetoria = Path(
        args.saida_trajetoria
    )

    saida_metricas = Path(
        args.saida_metricas
    )

    pasta_resultados = saida_metricas.parent

    pasta_resultados.mkdir(
        parents=True,
        exist_ok=True,
    )

    saida_trajetoria.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    print("\n====================================")
    print("ANÁLISE INDIVIDUAL DO EXPERIMENTO")
    print("====================================")

    print(f"Metodo declarado: {args.metodo}")
    print(f"Duracao solicitada: {args.duracao:g} s simulados")
    print("\nLendo rosbag...")

    dados = ler_rosbag(
        bag,
        args.storage_id,
    )

    referencia = dados["referencia"]
    odom = dados["odom"]
    eventos = dados["eventos"]

    if len(referencia) < 2:
        raise RuntimeError(
            "Ground Truth insuficiente."
        )

    if len(odom) < 2:
        raise RuntimeError(
            "Odometria insuficiente."
        )

    # --------------------------------------------------------
    # IDENTIFICAÇÃO DO INTERVALO EXPERIMENTAL
    # --------------------------------------------------------

    if eventos.empty:
        raise RuntimeError(
            "Não foram encontrados eventos da exploração."
        )

    primeiros_objetivos = eventos[
        eventos["event"] == "GOAL_SENT"
    ]

    if primeiros_objetivos.empty:
        raise RuntimeError(
            "Nenhum evento GOAL_SENT encontrado."
        )

    inicio = float(
        primeiros_objetivos["tempo_s"].min()
    )

    fim_previsto = inicio + args.duracao

    # A disponibilidade dos dados nao deve ser confundida com
    # a duracao solicitada. Nao extrapolamos metricas alem do clock.
    clocks = np.asarray(dados["clocks"], dtype=float)
    clocks_validos = clocks[np.isfinite(clocks) & (clocks >= inicio)]
    if len(clocks_validos) == 0:
        raise RuntimeError("Nao ha /clock valido apos o primeiro GOAL_SENT.")
    ultimo_clock = float(np.max(clocks_validos))
    fim = min(fim_previsto, ultimo_clock)
    if fim <= inicio:
        raise RuntimeError("Duracao observada insuficiente para analisar.")

    duracao_observada = fim - inicio
    intervalo_completo = bool(ultimo_clock >= fim_previsto - 0.1)
    print(f"\nInicio no relogio simulado (absoluto): {inicio:.3f} s")
    print(f"Fim previsto no relogio simulado (absoluto): {fim_previsto:.3f} s")
    print(f"Duracao solicitada: {args.duracao:.3f} s")
    print(f"Duracao disponivel para analise: {duracao_observada:.3f} s")
    if not intervalo_completo:
        print("AVISO: gravacao incompleta em relacao a duracao solicitada.")

    # --------------------------------------------------------
    # ANÁLISE DA ODOMETRIA
    # --------------------------------------------------------

    print("\nAnalisando odometria...")

    tabela_odom = alinhar_trajetorias(
        referencia,
        odom,
        inicio,
        fim,
        alinhamento=args.alinhamento,
    )

    if tabela_odom.empty:
        raise RuntimeError(
            "Não foi possível alinhar a odometria "
            "com a Ground Truth."
        )

    resultados = {
        "experimento_metodo_slam": args.metodo,
        "experimento_inicio_simulado_s": inicio,
        "experimento_fim_previsto_s": fim_previsto,
        "experimento_fim_analisado_s": fim,
        "experimento_duracao_configurada_s": args.duracao,
        "experimento_duracao_analisada_s": duracao_observada,
        "experimento_alinhamento": args.alinhamento,
    }

    resultados.update(
        calcular_metricas_trajetoria(
            tabela_odom,
            "odom",
        )
    )

    tabela_odom.to_csv(
        saida_trajetoria,
        index=False,
    )

    tabela_odom.to_csv(
        pasta_resultados / "trajetoria_odom.csv",
        index=False,
    )

    # --------------------------------------------------------
    # ANÁLISE DO SLAM
    # --------------------------------------------------------

    print(f"\nReconstruindo trajetória SLAM ({args.metodo})...")

    slam = reconstruir_slam(
        dados["tf"]
    )

    tabela_slam = pd.DataFrame()

    if not slam.empty:

        tabela_slam = alinhar_trajetorias(
            referencia,
            slam,
            inicio,
            fim,
            alinhamento=args.alinhamento,
        )

    if not tabela_slam.empty:

        resultados.update(
            calcular_metricas_trajetoria(
                tabela_slam,
                "slam",
            )
        )

        tabela_slam.to_csv(
            pasta_resultados / "trajetoria_slam.csv",
            index=False,
        )

        resultados[
            "slam_trajetoria_disponivel"
        ] = True

    else:

        print(
            "AVISO: não foi possível reconstruir "
            "a trajetória corrigida pelo SLAM."
        )

        resultados[
            "slam_trajetoria_disponivel"
        ] = False

    # --------------------------------------------------------
    # ANÁLISE DO MAPEAMENTO
    # --------------------------------------------------------

    print("\nAnalisando mapeamento...")

    metricas_mapa, evolucao = analisar_mapeamento(
        dados["mapas"],
        inicio,
        fim,
    )

    resultados.update(metricas_mapa)

    if not evolucao.empty:

        evolucao.to_csv(
            pasta_resultados / "mapa_evolucao.csv",
            index=False,
        )

    # --------------------------------------------------------
    # ANÁLISE DA EXPLORAÇÃO
    # --------------------------------------------------------

    print("\nAnalisando exploração...")

    metricas_exploracao, eventos_filtrados = (
        analisar_exploracao(
            eventos,
            inicio,
            fim,
        )
    )

    resultados.update(
        metricas_exploracao
    )

    if not eventos_filtrados.empty:

        eventos_filtrados.to_csv(
            pasta_resultados / "eventos_exploracao.csv",
            index=False,
        )

    # --------------------------------------------------------
    # EFICIÊNCIA ESPACIAL
    # --------------------------------------------------------

    area_nova = metricas_mapa.get(
        "mapa_variacao_area_conhecida_m2",
        np.nan,
    )

    distancia = resultados.get(
        "odom_distancia_referencia_m",
        np.nan,
    )

    resultados[
        "exploracao_eficiencia_m2_por_m"
    ] = (
        area_nova / distancia
        if np.isfinite(area_nova)
        and np.isfinite(distancia)
        and distancia > 0
        else np.nan
    )

    # --------------------------------------------------------
    # QUALIDADE DOS DADOS
    # --------------------------------------------------------

    resultados.update(
        analisar_qualidade(
            dados,
            inicio,
            fim,
        )
    )

    # --------------------------------------------------------
    # VERIFICAÇÃO DA DURAÇÃO
    # --------------------------------------------------------

    resultados["experimento_duracao_clock_observada_s"] = duracao_observada
    resultados["experimento_intervalo_completo"] = intervalo_completo

    # --------------------------------------------------------
    # --------------------------------------------------------
    # EXPORTAÇÃO
    # --------------------------------------------------------

    pd.DataFrame([resultados]).to_csv(
        saida_metricas,
        index=False,
    )

    print("\nGerando graficos e mapa final a partir da rosbag...")
    relatorio = gerar_relatorio_visual(
        pasta_resultados, resultados, tabela_odom, tabela_slam,
        evolucao, eventos_filtrados, dados["mapa_final"], inicio,
    )
    print(f"Relatorio visual: {relatorio}")
    print(f"Mapa final: {pasta_resultados / 'mapa_final.png'}"
          if (pasta_resultados / 'mapa_final.png').exists()
          else "Mapa final indisponivel na gravacao.")

    print("\n====================================")
    print("RESULTADOS")
    print("====================================")

    for nome, valor in resultados.items():
        print(f"{nome}: {valor}")

    print("\nArquivos gerados em:")
    print(pasta_resultados)

    print("\nAnálise concluída.")


if __name__ == "__main__":
    main()
