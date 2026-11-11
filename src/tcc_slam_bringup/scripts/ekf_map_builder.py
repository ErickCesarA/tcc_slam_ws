#!/usr/bin/env python3

import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException,
)


class EKFMapBuilder(Node):

    def __init__(self):
        super().__init__("ekf_map_builder")

        # Configuracao da grade
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("map_width", 20.0)
        self.declare_parameter("map_height", 20.0)
        self.declare_parameter("max_laser_range", 7.5)

        self.resolution = float(
            self.get_parameter("resolution").value
        )

        width_m = float(
            self.get_parameter("map_width").value
        )

        height_m = float(
            self.get_parameter("map_height").value
        )

        self.max_range = float(
            self.get_parameter("max_laser_range").value
        )

        self.width = math.ceil(width_m / self.resolution)
        self.height = math.ceil(height_m / self.resolution)

        self.origin_x = -width_m / 2.0
        self.origin_y = -height_m / 2.0

        # Grade de log-odds
        # Valores positivos indicam ocupacao.
        # Valores negativos indicam espaco livre.
        self.log_odds = np.zeros(
            (self.height, self.width),
            dtype=np.int16,
        )

        self.observed = np.zeros(
            (self.height, self.width),
            dtype=bool,
        )

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # Publicacao do mapa
        qos_map = QoSProfile(depth=1)
        qos_map.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_map.reliability = ReliabilityPolicy.RELIABLE

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            "/map",
            qos_map,
        )

        # Fila para aguardar TF correspondente ao scan
        self.pending_scans = deque(maxlen=20)

        self.scan_counter = 0
        self.processed_scans = 0
        self.map_load_time = self.get_clock().now().to_msg()

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.processing_timer = self.create_timer(
            0.1,
            self.process_pending,
        )

        self.map_timer = self.create_timer(
            0.5,
            self.publish_map,
        )

        self.get_logger().info(
            "EKF Map Builder iniciado."
        )

    # ==================================================
    # Coordenadas
    # ==================================================

    def world_to_grid(self, x, y):

        gx = math.floor(
            (x - self.origin_x) / self.resolution
        )

        gy = math.floor(
            (y - self.origin_y) / self.resolution
        )

        if (
            0 <= gx < self.width
            and 0 <= gy < self.height
        ):
            return gx, gy

        return None

    # ==================================================
    # Bresenham
    # ==================================================

    def ray_cells(self, x0, y0, x1, y1):

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        error = dx - dy

        while True:

            yield x0, y0

            if x0 == x1 and y0 == y1:
                break

            e2 = 2 * error

            if e2 > -dy:
                error -= dy
                x0 += sx

            if e2 < dx:
                error += dx
                y0 += sy

    # ==================================================
    # Integracao do LiDAR
    # ==================================================

    def integrate_scan(self, scan, transform):

        px = transform.transform.translation.x
        py = transform.transform.translation.y

        q = transform.transform.rotation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z),
        )

        origin = self.world_to_grid(px, py)

        if origin is None:
            return False

        x0, y0 = origin

        max_range = min(
            self.max_range,
            float(scan.range_max),
        )

        # Usar todos os raios da varredura
        for i, raw_range in enumerate(scan.ranges):

            if math.isnan(raw_range):
                continue

            if raw_range < scan.range_min:
                continue

            # Inf significa que nenhum obstaculo
            # foi detectado dentro do alcance.
            if math.isinf(raw_range):
                distance = max_range
                hit = False
            else:
                distance = min(
                    float(raw_range),
                    max_range,
                )

                hit = (
                    raw_range < max_range
                    and raw_range < scan.range_max
                )

            angle = (
                yaw
                + scan.angle_min
                + i * scan.angle_increment
            )

            end_x = px + distance * math.cos(angle)
            end_y = py + distance * math.sin(angle)

            # Coordenada da extremidade em indices
            # inteiros, mesmo quando fica fora do mapa
            x1 = math.floor(
                (end_x - self.origin_x) / self.resolution
            )

            y1 = math.floor(
                (end_y - self.origin_y) / self.resolution
            )

            # Descartar raios que exigiriam percorrer
            # distancias excessivas em grade
            max_steps = math.ceil(
                max_range / self.resolution
            ) + 3

            cells = []

            for step, (gx, gy) in enumerate(
                self.ray_cells(x0, y0, x1, y1)
            ):
                if step > max_steps:
                    break

                if not (
                    0 <= gx < self.width
                    and 0 <= gy < self.height
                ):
                    break

                cells.append((gx, gy))

            if not cells:
                continue

            # Espaco livre ao longo do raio
            free_cells = cells[:-1] if hit else cells

            for gx, gy in free_cells:

                self.observed[gy, gx] = True

                self.log_odds[gy, gx] = max(
                    -40,
                    int(self.log_odds[gy, gx]) - 2,
                )

            # Celula ocupada no retorno do LiDAR
            if hit and cells[-1] == (x1, y1):

                gx, gy = cells[-1]

                self.observed[gy, gx] = True

                self.log_odds[gy, gx] = min(
                    40,
                    int(self.log_odds[gy, gx]) + 6,
                )

        self.processed_scans += 1
        return True

    # ==================================================
    # Recebimento das varreduras
    # ==================================================

    def scan_callback(self, scan):

        self.scan_counter += 1

        # Comecar integrando cerca de 2 scans/s
        # para reduzir a carga de processamento.
        if self.scan_counter % 5 != 0:
            return

        self.pending_scans.append(scan)

    # ==================================================
    # Processamento quando TF estiver disponivel
    # ==================================================

    def process_pending(self):

        if not self.pending_scans:
            return

        scan = self.pending_scans[0]

        try:
            stamp = Time.from_msg(scan.header.stamp)

            transform = self.tf_buffer.lookup_transform(
                "map",
                scan.header.frame_id,
                stamp,
            )

        except TransformException:
            # A transformacao para este instante
            # pode ainda nao ter sido publicada.
            # Manter temporariamente na fila.
            return

        self.pending_scans.popleft()

        self.integrate_scan(scan, transform)

    # ==================================================
    # Publicacao da grade
    # ==================================================

    def publish_map(self):

        if self.processed_scans == 0:
            return

        msg = OccupancyGrid()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.info.map_load_time = self.map_load_time

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        # Converter log-odds para probabilidade
        probability = 1.0 / (
            1.0 + np.exp(
                -self.log_odds.astype(np.float32) / 5.0
            )
        )

        occupancy = np.round(
            probability * 100.0
        ).astype(np.int16)

        occupancy[~self.observed] = -1

        msg.data = occupancy.ravel().tolist()

        self.map_pub.publish(msg)

        if self.processed_scans % 20 == 0:
            self.get_logger().info(
                f"Mapa publicado. "
                f"Scans integrados: {self.processed_scans}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = EKFMapBuilder()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
