#!/usr/bin/env python3

import math

import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan

from visualization_msgs.msg import Marker, MarkerArray

from mrpt_msgs.msg import (
    ObservationRangeBearing,
    SingleRangeBearingObservation,
)


class LandmarkDetector(Node):

    def __init__(self):

        super().__init__("landmark_detector")

        # ==================================================
        # 1. PARAMETROS DO DETECTOR
        # ==================================================

        self.declare_parameter("max_range", 7.5)
        self.declare_parameter("min_range", 0.20)

        # Distancia maxima entre pontos consecutivos
        self.declare_parameter("cluster_gap", 0.30)

        # Tolerancia do ajuste de retas
        self.declare_parameter("split_threshold", 0.06)

        # Segmentos muito pequenos sao descartados
        self.declare_parameter("min_line_points", 4)
        self.declare_parameter("min_line_length", 0.15)

        # Angulos permitidos entre segmentos
        self.declare_parameter("min_corner_angle", 35.0)
        self.declare_parameter("max_corner_angle", 145.0)

        # Distancia maxima entre intersecao e extremidades
        self.declare_parameter("corner_endpoint_distance", 0.30)

        # Distancia minima entre cantos detectados
        self.declare_parameter("min_corner_separation", 0.15)

        # Incertezas iniciais das observacoes
        self.declare_parameter("sensor_std_range", 0.03)
        self.declare_parameter(
            "sensor_std_yaw",
            math.radians(0.1)
        )
        
        # Distancia maxima entre extremidades
        # de segmentos pertencentes a agrupamentos diferentes

        self.declare_parameter(
            "intercluster_endpoint_gap",
            0.25
        )

        # Distancia maxima da intersecao ate
        # cada extremidade considerada

        self.declare_parameter(
            "intercluster_corner_tolerance",
            0.20
        )

        # ==================================================
        # 2. PUBLICADORES
        # ==================================================

        self.landmark_pub = self.create_publisher(
            ObservationRangeBearing,
            "/landmark_observations",
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/landmark_viz",
            10
        )

        # ==================================================
        # 3. ASSINATURA DO LIDAR
        # ==================================================

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.scan_count = 0

        self.get_logger().info(
            "Detector automatico de landmarks iniciado."
        )

    # ======================================================
    # 4. AJUSTE DE UMA RETA POR PCA
    # ======================================================

    def fit_line(self, points):

        center = np.mean(points, axis=0)

        centered = points - center

        covariance = centered.T @ centered

        eigenvalues, eigenvectors = np.linalg.eigh(
            covariance
        )

        direction = eigenvectors[:, np.argmax(eigenvalues)]

        normal = np.array([
            -direction[1],
            direction[0]
        ])

        errors = np.abs(centered @ normal)

        return center, direction, normal, errors

    # ======================================================
    # 5. SEGMENTACAO SPLIT-AND-MERGE
    # ======================================================

    def split_lines(self, points):

        minimum = int(
            self.get_parameter("min_line_points").value
        )

        threshold = float(
            self.get_parameter("split_threshold").value
        )

        min_length = float(
            self.get_parameter("min_line_length").value
        )

        if len(points) < minimum:
            return []

        # Ajuste da reta por PCA
        _, _, _, errors = self.fit_line(points)

        max_error = float(np.max(errors))
        split_index = int(np.argmax(errors))

        # Dividir apenas quando ambos os segmentos
        # resultantes possuem pontos suficientes.
        can_split = (
            split_index >= minimum - 1
            and len(points) - split_index >= minimum
        )

        if max_error > threshold and can_split:

            # Compartilhar o ponto de separacao
            # preserva a continuidade geometrica.
            left_points = points[:split_index + 1]

            right_points = points[split_index:]

            left_lines = self.split_lines(left_points)

            right_lines = self.split_lines(right_points)

            return left_lines + right_lines

        # Se a reta ainda apresenta erro elevado
        # e nao pode ser dividida, nao devemos
        # considera-la automaticamente valida.
        if max_error > threshold:
            return []

        length = np.linalg.norm(
            points[-1] - points[0]
        )

        if length < min_length:
            return []

        return [points]
    # ======================================================
    # 6. AGRUPAMENTO DOS PONTOS DO LIDAR
    # ======================================================

    def get_clusters(self, scan):

        ranges = np.asarray(
            scan.ranges,
            dtype=np.float64
        )

        angles = (
            scan.angle_min
            + np.arange(len(ranges)) * scan.angle_increment
        )

        min_range = max(
            scan.range_min,
            float(self.get_parameter("min_range").value)
        )

        max_range = min(
            scan.range_max,
            float(self.get_parameter("max_range").value)
        )

        valid = (
            np.isfinite(ranges)
            & (ranges >= min_range)
            & (ranges <= max_range)
        )

        points = np.column_stack((
            ranges * np.cos(angles),
            ranges * np.sin(angles)
        ))

        gap = float(
            self.get_parameter("cluster_gap").value
        )

        clusters = []
        current = []

        for i, point in enumerate(points):

            if not valid[i]:

                if current:
                    clusters.append(np.asarray(current))

                current = []
                continue

            if current:

                distance = np.linalg.norm(
                    point - current[-1]
                )

                if distance > gap:

                    clusters.append(np.asarray(current))
                    current = []

            current.append(point)

        if current:
            clusters.append(np.asarray(current))

        return clusters

    # ======================================================
    # 7. INTERSECAO ENTRE DUAS RETAS
    # ======================================================

    def line_intersection(self, line_a, line_b):

        center_a, dir_a, _, _ = self.fit_line(line_a)
        center_b, dir_b, _, _ = self.fit_line(line_b)

        matrix = np.column_stack((
            dir_a,
            -dir_b
        ))

        determinant = np.linalg.det(matrix)

        if abs(determinant) < 1e-6:
            return None

        try:

            parameters = np.linalg.solve(
                matrix,
                center_b - center_a
            )

        except np.linalg.LinAlgError:
            return None

        intersection = (
            center_a + parameters[0] * dir_a
        )

        return intersection

    # ======================================================
    # 8. DETECCAO DE CANTOS
    # ======================================================

    def evaluate_corner(
        self,
        line_a,
        line_b,
        endpoint_limit
    ):

        _, direction_a, _, _ = self.fit_line(line_a)
        _, direction_b, _, _ = self.fit_line(line_b)

        dot = abs(
            np.dot(direction_a, direction_b)
        )

        dot = np.clip(dot, -1.0, 1.0)

        angle = math.degrees(
            math.acos(dot)
        )

        min_angle = float(
            self.get_parameter("min_corner_angle").value
        )

        max_angle = float(
            self.get_parameter("max_corner_angle").value
        )

        if not (min_angle <= angle <= max_angle):
            return None

        intersection = self.line_intersection(
            line_a,
            line_b
        )

        if intersection is None:
            return None

        distance_a = np.linalg.norm(
            intersection - line_a[-1]
        )

        distance_b = np.linalg.norm(
            intersection - line_b[0]
        )

        if (
            distance_a > endpoint_limit
            or distance_b > endpoint_limit
        ):
            return None

        return intersection


    def detect_corners(self, clusters):

        corners = []

        all_cluster_lines = []

        separation = float(
            self.get_parameter(
                "min_corner_separation"
            ).value
        )

        same_cluster_tolerance = float(
            self.get_parameter(
                "corner_endpoint_distance"
            ).value
        )

        intercluster_gap = float(
            self.get_parameter(
                "intercluster_endpoint_gap"
            ).value
        )

        intercluster_tolerance = float(
            self.get_parameter(
                "intercluster_corner_tolerance"
            ).value
        )

        # ==================================================
        # 1. Identificar linhas de cada agrupamento
        # ==================================================

        for cluster in clusters:

            lines = self.split_lines(cluster)

            all_cluster_lines.append(lines)

        # ==================================================
        # 2. Cantos dentro do mesmo agrupamento
        # ==================================================

        for lines in all_cluster_lines:

            for i in range(len(lines) - 1):

                corner = self.evaluate_corner(
                    lines[i],
                    lines[i + 1],
                    same_cluster_tolerance
                )

                if corner is not None:

                    corners.append(corner)

        # ==================================================
        # 3. Cantos entre agrupamentos adjacentes
        # ==================================================

        for i in range(len(all_cluster_lines) - 1):

            lines_a = all_cluster_lines[i]
            lines_b = all_cluster_lines[i + 1]

            if not lines_a or not lines_b:
                continue

            # Ultima linha do primeiro agrupamento
            line_a = lines_a[-1]

            # Primeira linha do agrupamento seguinte
            line_b = lines_b[0]

            # Proximidade entre extremidades
            endpoint_gap = np.linalg.norm(
                line_a[-1] - line_b[0]
            )

            if endpoint_gap > intercluster_gap:
                continue

            corner = self.evaluate_corner(
                line_a,
                line_b,
                intercluster_tolerance
            )

            if corner is not None:

                corners.append(corner)

        # ==================================================
        # 4. Remover cantos duplicados
        # ==================================================

        unique_corners = []

        for corner in corners:

            duplicate = any(
                np.linalg.norm(corner - previous)
                < separation
                for previous in unique_corners
            )

            if not duplicate:

                unique_corners.append(corner)

        return unique_corners

    # ======================================================
    # 9. PUBLICACAO PARA O MRPT
    # ======================================================

    def publish_landmarks(self, scan, corners):

        # O MRPT exige pelo menos um landmark por mensagem.
        # Se nenhum canto foi identificado, nao publica.

        if not corners:
            return

        msg = ObservationRangeBearing()

        msg.header = scan.header

        # O wrapper MRPT utiliza o TF para obter
        # a pose do lidar em relacao ao base_link.
        msg.sensor_pose_on_robot.orientation.w = 1.0

        msg.min_sensor_distance = float(
            scan.range_min
        )

        msg.max_sensor_distance = float(
            scan.range_max
        )

        msg.sensor_std_range = float(
            self.get_parameter("sensor_std_range").value
        )

        msg.sensor_std_yaw = float(
            self.get_parameter("sensor_std_yaw").value
        )

        msg.sensor_std_pitch = math.radians(0.1)

        for corner in corners:

            x, y = corner

            landmark = SingleRangeBearingObservation()

            landmark.range = float(
                math.hypot(x, y)
            )

            landmark.yaw = float(
                math.atan2(y, x)
            )

            landmark.pitch = 0.0

            # -1: landmark ainda nao identificado.
            # A associacao sera tratada posteriormente.
            landmark.id = -1

            msg.sensed_data.append(landmark)

        self.landmark_pub.publish(msg)

    # ======================================================
    # 10. VISUALIZACAO NO RVIZ2
    # ======================================================

    def publish_markers(self, scan, corners):

        markers = MarkerArray()

        clear = Marker()

        clear.header = scan.header
        clear.ns = "landmark_detector"
        clear.action = Marker.DELETEALL

        markers.markers.append(clear)

        for i, corner in enumerate(corners):

            marker = Marker()

            marker.header = scan.header
            marker.ns = "geometric_corners"

            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(corner[0])
            marker.pose.position.y = float(corner[1])
            marker.pose.position.z = 0.0

            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.12
            marker.scale.y = 0.12
            marker.scale.z = 0.12

            marker.color.r = 1.0
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 1.0

            markers.markers.append(marker)

        self.marker_pub.publish(markers)

    # ======================================================
    # 11. CALLBACK PRINCIPAL
    # ======================================================

    def scan_callback(self, scan):

        # ==========================================
        # 1. Agrupar pontos
        # ==========================================

        clusters = self.get_clusters(scan)

        # ==========================================
        # 2. Identificar segmentos de reta
        # ==========================================

        total_lines = 0
        clusters_with_multiple_lines = 0

        for cluster in clusters:

            lines = self.split_lines(cluster)

            total_lines += len(lines)

            if len(lines) >= 2:
                clusters_with_multiple_lines += 1

        # ==========================================
        # 3. Detectar cantos
        # ==========================================

        corners = self.detect_corners(clusters)

        # ==========================================
        # 4. Publicar
        # ==========================================

        self.publish_landmarks(scan, corners)

        self.publish_markers(scan, corners)

        # ==========================================
        # 5. Diagnostico
        # ==========================================

        self.scan_count += 1

        if self.scan_count % 30 == 0:

            self.get_logger().info(
                "\n"
                "========== DIAGNOSTICO ==========\n"
                f"Agrupamentos: {len(clusters)}\n"
                f"Segmentos de reta: {total_lines}\n"
                f"Agrupamentos com 2+ retas: "
                f"{clusters_with_multiple_lines}\n"
                f"Landmarks detectados: {len(corners)}\n"
                "================================="
            )

def main(args=None):

    rclpy.init(args=args)

    node = LandmarkDetector()

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
