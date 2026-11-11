#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


class BootstrapGraphSLAM(Node):

    def __init__(self):
        super().__init__("bootstrap_graphslam")

        # Configuracoes de seguranca
        self.velocidade = 0.08       # m/s
        self.distancia_minima = 0.60 # m
        self.tempo_maximo = 20.0     # segundos de avanco
        self.espera_sensor = 15.0    # segundos
        self.min_frontiers = 5

        self.inicio = time.monotonic()
        self.inicio_movimento = None

        self.scan = None
        self.ultimo_scan = None
        self.fronteira_encontrada = False
        self.resultado = None

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            map_qos
        )

        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(
            0.1,
            self.controlar
        )

        self.get_logger().info(
            "Bootstrap GraphSLAM iniciado."
        )

    def scan_callback(self, msg):
        self.scan = msg
        self.ultimo_scan = time.monotonic()

    def map_callback(self, msg):
        if self.detectar_fronteira(msg):
            if not self.fronteira_encontrada:
                self.get_logger().info(
                    "Fronteira encontrada no mapa!"
                )

            self.fronteira_encontrada = True

    def detectar_fronteira(self, mapa):
        largura = mapa.info.width
        altura = mapa.info.height
        dados = mapa.data

        if largura < 3 or altura < 3:
            return False

        # Uma fronteira e uma celula livre
        # adjacente a uma celula desconhecida.
        quantidade = 0

        for y in range(1, altura - 1):
            for x in range(1, largura - 1):

                i = y * largura + x

                if dados[i] != 0:
                    continue

                vizinhos = (
                    dados[i - 1],
                    dados[i + 1],
                    dados[i - largura],
                    dados[i + largura],
                )

                if -1 in vizinhos:
                    quantidade += 1

                    if quantidade >= self.min_frontiers:
                        return True

        return False

    def caminho_livre(self):
        if self.scan is None:
            return False

        distancias = []

        for i, distancia in enumerate(self.scan.ranges):

            angulo = (
                self.scan.angle_min
                + i * self.scan.angle_increment
            )

            # Setor frontal de 50 graus
            if abs(angulo) > math.radians(25):
                continue

            if math.isfinite(distancia):
                if self.scan.range_min <= distancia <= self.scan.range_max:
                    distancias.append(distancia)

        if not distancias:
            return False

        return min(distancias) > self.distancia_minima

    def parar(self):
        self.cmd_pub.publish(Twist())

    def finalizar(self, sucesso, mensagem):
        self.parar()
        self.get_logger().info(mensagem)
        self.resultado = 0 if sucesso else 1

    def controlar(self):
        if self.resultado is not None:
            return

        agora = time.monotonic()

        # Condicao de sucesso
        if self.fronteira_encontrada:
            self.finalizar(
                True,
                "Fronteira detectada. Liberando inicializacao do Nav2."
            )
            return

        # Aguardar primeiro scan
        if self.scan is None:
            self.parar()

            if agora - self.inicio > self.espera_sensor:
                self.finalizar(
                    False,
                    "ERRO: LiDAR nao disponivel."
                )

            return

        # Bloquear caso as leituras parem
        if agora - self.ultimo_scan > 1.0:
            self.finalizar(
                False,
                "ERRO: LiDAR parou de atualizar."
            )
            return

        # Bloquear avanco diante de obstaculos
        if not self.caminho_livre():
            self.finalizar(
                False,
                "ERRO: Obstaculo detectado. Movimento interrompido."
            )
            return

        if self.inicio_movimento is None:
            self.inicio_movimento = agora
            self.get_logger().info(
                "Nenhuma fronteira detectada. Iniciando avanco."
            )

        # Tempo maximo de movimento
        if agora - self.inicio_movimento >= self.tempo_maximo:
            self.finalizar(
                False,
                "ERRO: Limite de movimentacao atingido sem fronteiras."
            )
            return

        comando = Twist()
        comando.linear.x = self.velocidade

        self.cmd_pub.publish(comando)


def main():
    rclpy.init()

    node = BootstrapGraphSLAM()

    try:
        while rclpy.ok() and node.resultado is None:
            rclpy.spin_once(node, timeout_sec=0.2)

        resultado = node.resultado
        if resultado is None:
            resultado = 1

    except KeyboardInterrupt:
        resultado = 1

    finally:
        # Repetir comando de parada antes de encerrar
        for _ in range(5):
            node.parar()
            time.sleep(0.05)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(resultado)


if __name__ == "__main__":
    main()
