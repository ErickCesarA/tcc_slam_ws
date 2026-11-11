#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


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


class TestarOdometria(Node):

    def __init__(self):

        super().__init__("testar_odometria")

        self.odom = None
        self.truth = None

        self.initial_odom = None
        self.initial_truth = None

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.create_subscription(
            Odometry,
            "/ground_truth/odom",
            self.truth_callback,
            10
        )

        self.create_timer(
            1.0,
            self.analisar
        )

        self.get_logger().info(
            "Monitor de odometria iniciado."
        )

    def odom_callback(self, msg):

        self.odom = msg

    def truth_callback(self, msg):

        self.truth = msg

    def extrair_pose(self, msg):

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        return (
            p.x,
            p.y,
            yaw_from_quaternion(q)
        )

    def pose_relativa(self, pose, inicial):

        x, y, yaw = pose

        x0, y0, yaw0 = inicial

        dx = x - x0
        dy = y - y0

        c = math.cos(yaw0)
        s = math.sin(yaw0)

        return (
            c*dx + s*dy,
            -s*dx + c*dy,
            normalize_angle(yaw-yaw0)
        )

    def analisar(self):

        if self.odom is None or self.truth is None:
            return

        # Evitar comparar mensagens muito distantes no tempo
        t_odom = (
            self.odom.header.stamp.sec
            + self.odom.header.stamp.nanosec * 1e-9
        )

        t_truth = (
            self.truth.header.stamp.sec
            + self.truth.header.stamp.nanosec * 1e-9
        )

        if abs(t_odom - t_truth) > 0.10:

            self.get_logger().warn(
                "Mensagens nao sincronizadas."
            )
            return

        pose_odom = self.extrair_pose(self.odom)

        pose_truth = self.extrair_pose(self.truth)

        if self.initial_odom is None:

            self.initial_odom = pose_odom
            self.initial_truth = pose_truth

            self.get_logger().info(
                "Referencias iniciais definidas."
            )

            return

        x_o, y_o, yaw_o = self.pose_relativa(
            pose_odom,
            self.initial_odom
        )

        x_t, y_t, yaw_t = self.pose_relativa(
            pose_truth,
            self.initial_truth
        )

        erro_pos = math.hypot(
            x_o-x_t,
            y_o-y_t
        )

        erro_yaw = abs(
            normalize_angle(yaw_o-yaw_t)
        )

        self.get_logger().info(
            "\n"
            "========= ODOMETRIA =========\n"
            f"Odom:  x={x_o:.3f}, y={y_o:.3f}\n"
            f"Truth: x={x_t:.3f}, y={y_t:.3f}\n"
            f"Erro posicao: {erro_pos:.3f} m\n"
            f"Erro angular: {math.degrees(erro_yaw):.2f} graus\n"
            "============================="
        )


def main():

    rclpy.init()

    node = TestarOdometria()

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
