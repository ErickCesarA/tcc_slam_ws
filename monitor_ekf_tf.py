#!/usr/bin/env python3

import math
import csv
import os

import rclpy

from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def yaw_from_quaternion(q):

    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    )


def normalize_angle(angle):

    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


class MonitorEKFTF(Node):

    def __init__(self):

        super().__init__("monitor_ekf_tf")

        # ==============================================
        # CONFIGURACOES
        # ==============================================

        self.declare_parameter(
            "threshold_position",
            0.05
        )

        self.declare_parameter(
            "threshold_angle",
            2.0
        )

        self.threshold_position = float(
            self.get_parameter(
                "threshold_position"
            ).value
        )

        self.threshold_angle = float(
            self.get_parameter(
                "threshold_angle"
            ).value
        )

        # ==============================================
        # ESTADO
        # ==============================================

        self.previous = None

        self.count = 0

        self.corrections = 0

        # ==============================================
        # ARQUIVO CSV
        # ==============================================

        self.output_dir = os.path.expanduser(
            "~/tcc_slam_ws/logs/ekf_tf"
        )

        os.makedirs(
            self.output_dir,
            exist_ok=True
        )

        self.csv_path = os.path.join(
            self.output_dir,
            "monitor_ekf_tf.csv"
        )

        self.csv_file = open(
            self.csv_path,
            "w",
            newline=""
        )

        self.writer = csv.writer(
            self.csv_file
        )

        self.writer.writerow([
            "tempo_sim",
            "x",
            "y",
            "yaw_deg",
            "delta_pos",
            "delta_yaw_deg",
            "correcao_detectada"
        ])

        # ==============================================
        # SUBSCRIBER
        # ==============================================

        self.create_subscription(
            TFMessage,
            "/tf",
            self.callback,
            100
        )

        self.get_logger().info(
            "Monitor EKF TF iniciado."
        )

        self.get_logger().info(
            f"Arquivo CSV: {self.csv_path}"
        )

        self.get_logger().info(
            f"Limite posicao: "
            f"{self.threshold_position:.3f} m"
        )

        self.get_logger().info(
            f"Limite angular: "
            f"{self.threshold_angle:.2f} graus"
        )

    # ==============================================
    # CALLBACK
    # ==============================================

    def callback(self, msg):

        for transform in msg.transforms:

            if (
                transform.header.frame_id != "map"
                or transform.child_frame_id != "odom"
            ):
                continue

            stamp = (
                transform.header.stamp.sec
                + transform.header.stamp.nanosec * 1e-9
            )

            p = transform.transform.translation
            q = transform.transform.rotation

            x = float(p.x)
            y = float(p.y)

            yaw = yaw_from_quaternion(q)

            yaw_deg = math.degrees(yaw)

            self.count += 1

            delta_pos = 0.0
            delta_yaw_deg = 0.0
            correction = False

            if self.previous is not None:

                old_x, old_y, old_yaw, old_stamp = (
                    self.previous
                )

                # Ignorar timestamps repetidos ou antigos
                if stamp <= old_stamp:
                    continue

                dx = x - old_x
                dy = y - old_y

                delta_pos = math.hypot(
                    dx,
                    dy
                )

                delta_yaw = abs(
                    normalize_angle(
                        yaw - old_yaw
                    )
                )

                delta_yaw_deg = math.degrees(
                    delta_yaw
                )

                if (
                    delta_pos > self.threshold_position
                    or
                    delta_yaw_deg > self.threshold_angle
                ):

                    correction = True

                    self.corrections += 1

                    self.get_logger().warn(
                        "\n"
                        "========== CORRECAO TF ==========\n"
                        f"Tempo simulado: {stamp:.3f} s\n"
                        f"Delta posicao: {delta_pos:.4f} m\n"
                        f"Delta angular: {delta_yaw_deg:.3f} graus\n"
                        f"Posicao map->odom: "
                        f"x={x:.3f}, y={y:.3f}\n"
                        f"Total correcoes: {self.corrections}\n"
                        "================================="
                    )

            # ======================================
            # REGISTRAR CSV
            # ======================================

            self.writer.writerow([
                stamp,
                x,
                y,
                yaw_deg,
                delta_pos,
                delta_yaw_deg,
                int(correction)
            ])

            self.csv_file.flush()

            # ======================================
            # ATUALIZAR ESTADO
            # ======================================

            self.previous = (
                x,
                y,
                yaw,
                stamp
            )

    def close(self):

        if not self.csv_file.closed:
            self.csv_file.close()


def main(args=None):

    rclpy.init(args=args)

    node = MonitorEKFTF()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.close()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
