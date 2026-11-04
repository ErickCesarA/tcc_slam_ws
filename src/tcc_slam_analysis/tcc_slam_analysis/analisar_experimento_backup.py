import argparse
from pathlib import Path

import numpy as np
import pandas as pd

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def extrair_trajetoria(
    arquivo_bag,
    topico,
    storage_id="mcap"
):
    """
    Extrai x, y e timestamp de um tópico nav_msgs/msg/Odometry.
    O timestamp utilizado é o timestamp de gravação da mensagem.
    """

    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(
        uri=str(arquivo_bag),
        storage_id=storage_id
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader.open(storage_options, converter_options)

    tipos = {
        item.name: item.type
        for item in reader.get_all_topics_and_types()
    }

    if topico not in tipos:
        raise ValueError(
            f"O tópico '{topico}' não foi encontrado na rosbag. "
            f"Tópicos disponíveis: {list(tipos.keys())}"
        )

    tipo_mensagem = tipos[topico]

    if tipo_mensagem != "nav_msgs/msg/Odometry":
        raise ValueError(
            f"O tópico '{topico}' possui o tipo '{tipo_mensagem}', "
            "mas esta versão do programa espera "
            "nav_msgs/msg/Odometry."
        )

    classe_mensagem = get_message(tipo_mensagem)

    tempos = []
    posicoes_x = []
    posicoes_y = []

    while reader.has_next():
        nome_topico, dados_serializados, timestamp_ns = reader.read_next()

        if nome_topico != topico:
            continue

        mensagem = deserialize_message(
            dados_serializados,
            classe_mensagem
        )

        tempos.append(timestamp_ns * 1e-9)

        posicoes_x.append(
            mensagem.pose.pose.position.x
        )

        posicoes_y.append(
            mensagem.pose.pose.position.y
        )

    if len(tempos) < 2:
        raise ValueError(
            f"O tópico '{topico}' possui menos de duas amostras."
        )

    resultado = pd.DataFrame({
        "tempo_s": tempos,
        "x": posicoes_x,
        "y": posicoes_y
    })

    resultado = resultado.sort_values("tempo_s")

    # Elimina timestamps duplicados para permitir a interpolação.
    resultado = resultado.drop_duplicates(
        subset="tempo_s",
        keep="last"
    )

    return resultado.reset_index(drop=True)


def alinhar_trajetorias(referencia, estimada):
    """
    Interpola a trajetória estimada nos instantes da referência,
    utilizando somente o intervalo temporal comum às duas.
    """

    inicio = max(
        referencia["tempo_s"].min(),
        estimada["tempo_s"].min()
    )

    fim = min(
        referencia["tempo_s"].max(),
        estimada["tempo_s"].max()
    )

    if fim <= inicio:
        raise ValueError(
            "As trajetórias não possuem intervalo temporal comum."
        )

    ref = referencia[
        (referencia["tempo_s"] >= inicio)
        & (referencia["tempo_s"] <= fim)
    ].copy()

    est = estimada

    x_est = np.interp(
        ref["tempo_s"].to_numpy(),
        est["tempo_s"].to_numpy(),
        est["x"].to_numpy()
    )

    y_est = np.interp(
        ref["tempo_s"].to_numpy(),
        est["tempo_s"].to_numpy(),
        est["y"].to_numpy()
    )

    tabela = pd.DataFrame({
        "tempo_s": ref["tempo_s"].to_numpy(),
        "x_ref": ref["x"].to_numpy(),
        "y_ref": ref["y"].to_numpy(),
        "x_est": x_est,
        "y_est": y_est
    })

    if len(tabela) < 2:
        raise ValueError(
            "São necessárias pelo menos duas amostras alinhadas."
        )

    return tabela


def calcular_metricas(tabela):
    """
    Calcula métricas de erro de posição e deslocamento relativo.
    """

    referencia = tabela[
        ["x_ref", "y_ref"]
    ].to_numpy()

    estimada = tabela[
        ["x_est", "y_est"]
    ].to_numpy()

    erro = estimada - referencia

    erro_posicao = np.linalg.norm(
        erro,
        axis=1
    )

    deslocamento_ref = np.diff(
        referencia,
        axis=0
    )

    deslocamento_est = np.diff(
        estimada,
        axis=0
    )

    erro_rpe = np.linalg.norm(
        deslocamento_est - deslocamento_ref,
        axis=1
    )

    distancia_ref = np.linalg.norm(
        deslocamento_ref,
        axis=1
    )

    distancia_est = np.linalg.norm(
        deslocamento_est,
        axis=1
    )

    distancia_percorrida_ref = np.sum(distancia_ref)

    distancia_percorrida_est = np.sum(distancia_est)

    resultados = {
        "numero_amostras": len(tabela),

        "ATE_RMSE_m": np.sqrt(
            np.mean(erro_posicao ** 2)
        ),

        "ATE_medio_m": np.mean(
            erro_posicao
        ),

        "ATE_mediano_m": np.median(
            erro_posicao
        ),

        "ATE_maximo_m": np.max(
            erro_posicao
        ),

        "ATE_minimo_m": np.min(
            erro_posicao
        ),

        "ATE_desvio_padrao_m": np.std(
            erro_posicao,
            ddof=1
        ),

        "RPE_RMSE_m": np.sqrt(
            np.mean(erro_rpe ** 2)
        ),

        "RPE_medio_m": np.mean(
            erro_rpe
        ),

        "RPE_maximo_m": np.max(
            erro_rpe
        ),

        "RPE_minimo_m": np.min(
            erro_rpe
        ),

        "distancia_referencia_m": distancia_percorrida_ref,

        "distancia_estimada_m": distancia_percorrida_est,

        "duracao_s": (
            tabela["tempo_s"].iloc[-1]
            - tabela["tempo_s"].iloc[0]
        )
    }

    if distancia_percorrida_ref > 0:
        resultados["erro_distancia_percorrida_percentual"] = (
            abs(
                distancia_percorrida_est
                - distancia_percorrida_ref
            )
            / distancia_percorrida_ref
            * 100
        )
    else:
        resultados["erro_distancia_percorrida_percentual"] = np.nan

    return resultados


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Extrai trajetorias de uma rosbag e calcula "
            "metricas de desempenho."
        )
    )

    parser.add_argument(
        "arquivo_bag",
        help="Diretorio da rosbag gravada"
    )

    parser.add_argument(
        "--storage-id",
        choices=["mcap", "sqlite3"],
        default="mcap",
        help="Formato de armazenamento da rosbag (padrão: mcap)."
    )

    parser.add_argument(
        "--referencia",
        default="/ground_truth/odom",
        help="Topico nav_msgs/msg/Odometry de referencia."
    )

    parser.add_argument(
        "--estimada",
        default="/odom",
        help="Topico nav_msgs/msg/Odometry da estimativa."
    )

    parser.add_argument(
        "--saida-trajetoria",
        default="trajetoria.csv",
        help="Caminho do CSV com as trajetorias alinhadas."
    )

    parser.add_argument(
        "--saida-metricas",
        default="metricas.csv",
        help="Caminho do CSV com as metricas calculadas."
    )

    args = parser.parse_args()

    arquivo_bag = Path(args.arquivo_bag)

    if not arquivo_bag.exists():
        raise FileNotFoundError(
            f"A rosbag nao foi encontrada: {arquivo_bag}"
        )

    print("Extraindo trajetória de referência...")

    referencia = extrair_trajetoria(
        arquivo_bag,
        args.referencia,
        args.storage_id
        #"mcap"
    )

    print("Extraindo trajetória estimada...")

    estimada = extrair_trajetoria(
        arquivo_bag,
        args.estimada,
        args.storage_id
        #"mcap"
    )

    print("Alinhando as trajetórias...")

    tabela = alinhar_trajetorias(
        referencia,
        estimada
    )

    print("Calculando métricas...")

    resultados = calcular_metricas(tabela)

    caminho_trajetoria = Path(args.saida_trajetoria)
    caminho_metricas = Path(args.saida_metricas)

    caminho_trajetoria.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    caminho_metricas.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    tabela.to_csv(
        caminho_trajetoria,
        index=False
    )

    pd.DataFrame([resultados]).to_csv(
        caminho_metricas,
        index=False
    )

    print("\nMétricas calculadas:")

    for nome, valor in resultados.items():
        print(f"{nome}: {valor}")

    print(
        f"\nTrajetória salva em: {caminho_trajetoria}"
    )

    print(
        f"Métricas salvas em: {caminho_metricas}"
    )


if __name__ == "__main__":
    main()
