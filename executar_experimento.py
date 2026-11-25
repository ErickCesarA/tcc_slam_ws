#!/usr/bin/env python3

import argparse
import json
import hashlib
import os
import math
import xml.etree.ElementTree as ET
import signal
import subprocess
import time
from pathlib import Path

from experimento_paths import nome_experimento, checar_id_disponivel

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose


TOPICOS = [
    "/clock",
    "/scan",
    "/odom",
    "/ground_truth/odom",
    "/imu/data",
    "/tf",
    "/tf_static",
    "/map",
    "/cmd_vel",
    "/cmd_vel_nav",
    "/cmd_vel_smoothed",
    "/exploration/events",
]


class MonitorExperimento(Node):

    def __init__(self):
        super().__init__("monitor_experimento")

        self.tempo_simulado = None
        self.tempo_inicial = None
        self.ultimo_clock = None
        self.erro_clock = False

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )

        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        event_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            Clock,
            "/clock",
            self.clock_callback,
            clock_qos,
        )

        self.create_subscription(
            String,
            "/exploration/events",
            self.event_callback,
            event_qos,
        )

    def clock_callback(self, msg):

        atual = (
            msg.clock.sec
            + msg.clock.nanosec / 1e9
        )

        if (
            self.ultimo_clock is not None
            and atual < self.ultimo_clock - 0.001
        ):
            self.erro_clock = True

        self.ultimo_clock = atual
        self.tempo_simulado = atual

    def event_callback(self, msg):

        try:
            evento = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(
                "Evento JSON inválido."
            )
            return

        nome = evento.get("event")

        self.get_logger().info(
            f"Evento recebido: {nome}"
        )

        if (
            nome == "GOAL_SENT"
            and self.tempo_inicial is None
        ):
            self.tempo_inicial = evento.get(
                "timestamp",
                self.tempo_simulado,
            )

            self.get_logger().info(
                "CRONÔMETRO INICIADO"
            )

    def nav_disponivel(self):
        return self.nav_client.server_is_ready()


def iniciar_processo(comando, log=None):

    kwargs = {
        "start_new_session": True,
    }

    if log is not None:
        kwargs["stdout"] = log
        kwargs["stderr"] = subprocess.STDOUT

    return subprocess.Popen(
        comando,
        **kwargs,
    )


def encerrar_processo(processo):

    if processo is None:
        return

    if processo.poll() is not None:
        return

    try:
        os.killpg(
            processo.pid,
            signal.SIGINT,
        )

        processo.wait(timeout=15)

    except subprocess.TimeoutExpired:

        os.killpg(
            processo.pid,
            signal.SIGTERM,
        )

        try:
            processo.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(
                processo.pid,
                signal.SIGKILL,
            )
            processo.wait()


def aguardar_nav2(node, timeout=120):

    inicio = time.monotonic()

    while rclpy.ok():

        rclpy.spin_once(
            node,
            timeout_sec=0.2,
        )

        if node.nav_disponivel():
            return True

        if time.monotonic() - inicio > timeout:
            return False

    return False


def registrar_proveniencia(workspace, sdf):
    """Registra referências da implementação sem depender do ROS.

    Os hashes documentam os arquivos em src/; precisam corresponder ao
    código compilado em install/ para representar a execução efetiva.
    """
    caminhos = {
        'mundo_sdf': sdf,
        'modelo_xacro': workspace / 'src/tcc_slam_description/urdf/tcc_robot.urdf.xacro',
        'nav2_yaml': workspace / 'src/tcc_slam_bringup/config/nav2_params.yaml',
        'frontier_explorer_py': workspace / 'src/tcc_slam_exploration/tcc_slam_exploration/frontier_explorer.py',
    }
    hashes = {
        nome: hashlib.sha256(arquivo.read_bytes()).hexdigest()
        for nome, arquivo in caminhos.items() if arquivo.is_file()
    }
    git_rev = None
    git_dirty = None
    try:
        rev = subprocess.run(['git','-C',str(workspace),'rev-parse','HEAD'],
                             capture_output=True,text=True,check=True,timeout=5)
        git_rev = rev.stdout.strip()
        estado = subprocess.run(['git','-C',str(workspace),'status','--porcelain'],
                                capture_output=True,text=True,check=True,timeout=5)
        git_dirty = bool(estado.stdout.strip())
    except (OSError, subprocess.SubprocessError):
        pass
    return {'git_commit': git_rev, 'git_alteracoes_nao_commitadas': git_dirty,
            'hashes_configuracao_sha256': hashes}


def executar(numero, duracao, metodo, mapa):

    workspace = Path.home() / "tcc_slam_ws"

    identificador = f"{numero:03d}"
    checar_id_disponivel(workspace, numero)
    pasta_nome = nome_experimento(numero, metodo, duracao, mapa)

    print(f"Metodo de SLAM informado: {metodo}")
    print(f"Duracao planejada: {duracao:g} s simulados")
    print(f"Mapa declarado: {mapa}")
    print(f"Pasta do experimento: {pasta_nome}")

    # Confere o arquivo de cenário cadastrado (não verifica o Gazebo em execução).
    nome_sdf = "tcc_world.sdf" if mapa == 1 else f"tcc_world_{mapa}.sdf"
    sdf = workspace / "src" / "tcc_slam_description" / "worlds" / nome_sdf
    if not sdf.is_file():
        raise FileNotFoundError(f"Arquivo SDF do mapa {mapa} não encontrado: {sdf}")
    elem = ET.parse(sdf).find("world")
    if elem is None or not elem.get("name"):
        raise ValueError(f"SDF sem nome interno do mundo: {sdf}")
    world_name = elem.get("name")

    pasta = (
        workspace
        / "bags"
        / pasta_nome
        / "dados"
    )

    bag = pasta / "ground_truth"

    if bag.exists():
        raise RuntimeError(
            f"A gravação já existe: {bag}\n"
            "Escolha outro número de experimento."
        )

    pasta.mkdir(
        parents=True,
        exist_ok=True,
    )

    rclpy.init()

    node = MonitorExperimento()

    gravador = None
    explorador = None

    arquivo_log = None
    concluido = False

    try:

        print("\nAguardando Nav2...")

        if not aguardar_nav2(node):
            raise RuntimeError(
                "Nav2 não ficou disponível."
            )

        print("Nav2 disponível.")

        print("Aguardando relógio simulado...")

        inicio_espera = time.monotonic()

        while node.tempo_simulado is None:

            rclpy.spin_once(
                node,
                timeout_sec=0.2,
            )

            if time.monotonic() - inicio_espera > 30:
                raise RuntimeError(
                    "Não foi possível receber /clock."
                )

        print("Relógio recebido.")

        # Metadata explicita; o rosbag por si so nao informa com seguranca
        # qual metodo estava ativo.
        (pasta.parent / "manifest.json").write_text(
            json.dumps({
                "experimento": identificador,
                "slam_method": metodo,
                "duracao_configurada_s": duracao,
                "map_id": mapa,
                "world_name": world_name,
                "modo_experimento": "fronteiras",
                "perfil_sensores": "nominal",
                "nome_pasta": pasta_nome,
                **registrar_proveniencia(workspace, sdf),
            }, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

        arquivo_log = open(
            pasta / "gravacao.log",
            "w",
        )

        comando_bag = [
            "ros2",
            "bag",
            "record",
            "--output",
            str(bag),
            "--topics",
            *TOPICOS,
        ]

        print("\nIniciando rosbag2...")

        gravador = iniciar_processo(
            comando_bag,
            arquivo_log,
        )

        # Intervalo apenas para inicialização do gravador.
        # Não faz parte da duração experimental configurada.
        time.sleep(3)

        if gravador.poll() is not None:
            raise RuntimeError(
                "O gravador rosbag2 encerrou inesperadamente."
            )

        print("Gravador iniciado.")

        print("\nIniciando Frontier Explorer...")

        explorador = iniciar_processo([
            "ros2",
            "run",
            "tcc_slam_exploration",
            "frontier_explorer",
            "--ros-args",
            "-p",
            "use_sim_time:=true",
        ])

        inicio_espera = time.monotonic()

        while node.tempo_inicial is None:

            rclpy.spin_once(
                node,
                timeout_sec=0.2,
            )

            if node.erro_clock:
                raise RuntimeError(
                    "Retrocesso detectado no relógio simulado."
                )

            if explorador.poll() is not None:
                raise RuntimeError(
                    "Frontier Explorer encerrou antes do primeiro objetivo."
                )

            if time.monotonic() - inicio_espera > 120:
                raise RuntimeError(
                    "Nenhum GOAL_SENT recebido em 120 segundos reais."
                )

        print(
            f"\nExperimento {identificador} iniciado."
        )

        print(
            f"Duração: {duracao} segundos simulados."
        )

        ultimo_relatorio = -1

        while rclpy.ok():

            rclpy.spin_once(
                node,
                timeout_sec=0.2,
            )

            if node.erro_clock:
                raise RuntimeError(
                    "Relógio simulado retrocedeu."
                )

            if explorador.poll() is not None:
                raise RuntimeError(
                    "Frontier Explorer encerrou inesperadamente."
                )

            if gravador.poll() is not None:
                raise RuntimeError(
                    "Rosbag2 encerrou inesperadamente."
                )

            if node.tempo_simulado is None:
                continue

            decorrido = (
                node.tempo_simulado
                - node.tempo_inicial
            )

            if decorrido < 0:
                raise RuntimeError(
                    "Tempo experimental negativo."
                )

            intervalo = int(decorrido // 10)

            if intervalo > ultimo_relatorio:

                ultimo_relatorio = intervalo

                print(
                    f"Tempo simulado: "
                    f"{decorrido:.1f}/{duracao:.1f} s",
                    flush=True,
                )

            if decorrido >= duracao:
                concluido = True
                break

        print("\nEncerrando exploração...")

    finally:

        encerrar_processo(explorador)
        encerrar_processo(gravador)

        if arquivo_log is not None:
            arquivo_log.close()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    if not concluido:
        raise RuntimeError(
            "Experimento interrompido."
        )

    print("\nExecutando análise...")

    subprocess.run(
        [
            str(
                workspace
                / "analisar_experimento.sh"
            ),
            str(numero),
        ],
        check=True,
    )

    print(
        f"\nExperimento {identificador} concluído."
    )


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "numero",
        type=int,
    )

    parser.add_argument(
        "--metodo",
        choices=["toolbox", "graph", "rbpf", "ekf"],
        required=True,
        help="Metodo SLAM ativo no navigation.launch.py",
    )

    parser.add_argument(
        "--duracao",
        type=float,
        default=600.0,
    )

    parser.add_argument(
        "--mapa",
        type=int,
        required=True,
        help="ID do mapa ativo no navigation.launch.py (ex.: 1 ou 2).",
    )

    args = parser.parse_args()

    if args.numero < 0:
        parser.error(
            "O número do experimento deve ser não negativo."
        )

    if not math.isfinite(args.duracao) or args.duracao <= 0:
        parser.error(
            "A duração deve ser positiva."
        )

    if args.mapa < 1:
        parser.error("O número do mapa deve ser positivo.")

    executar(
        args.numero,
        args.duracao,
        args.metodo,
        args.mapa,
    )


if __name__ == "__main__":

    try:
        main()

    except KeyboardInterrupt:
        print("\nExecução interrompida.")

    except Exception as erro:
        print(f"\nERRO: {erro}")
        raise SystemExit(1)
