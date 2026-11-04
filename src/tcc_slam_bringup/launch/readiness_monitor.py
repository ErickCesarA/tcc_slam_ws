#!/usr/bin/env python3

import argparse
import math
import time

import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from rclpy.time import Time

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid

from tf2_ros import Buffer, TransformListener


class ReadinessMonitor(Node):

    def __init__(self, mode):

        super().__init__("readiness_monitor_" + mode)

        self.mode = mode

        self.clock_time = None
        self.clock_count = 0

        self.scan_received = False
        self.odom_received = False
        self.map_received = False

        self.clock_reset = False
        self.ready = False

        self.last_clock_jump_wall = time.monotonic()
        self.clock_stable_seconds = 5.0
        self.last_clock_progress_wall = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        clock_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            Clock,
            "/clock",
            self.clock_callback,
            clock_qos,
        )

        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            sensor_qos,
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            sensor_qos,
        )

        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            map_qos,
        )

        self.timer = self.create_timer(
            1.0,
            self.check_readiness,
        )

        self.get_logger().info(
            f"Monitor iniciado: {mode}"
        )

    def clock_callback(self, msg):

        current_time = (
            msg.clock.sec
            + msg.clock.nanosec * 1e-9
        )

        if not math.isfinite(current_time):
            return

        now = time.monotonic()

        if (
            self.clock_time is not None
            and current_time < self.clock_time
        ):

            self.get_logger().warning(
                "Retrocesso no relógio detectado."
            )

            self.last_clock_jump_wall = now

            self.clock_count = 0

            self.scan_received = False
            self.odom_received = False
            self.map_received = False

            self.tf_buffer.clear()

        elif (
            self.clock_time is None
            or current_time > self.clock_time
        ):

            self.last_clock_progress_wall = now

        self.clock_time = current_time
        self.clock_count += 1

    def scan_callback(self, msg):

        if len(msg.ranges) > 0:
            self.scan_received = True

    def odom_callback(self, msg):

        if (
            msg.header.frame_id == "odom"
            and msg.child_frame_id == "base_link"
        ):
            self.odom_received = True

    def map_callback(self, msg):

        if (
            msg.info.width > 0
            and msg.info.height > 0
            and len(msg.data) > 0
        ):
            self.map_received = True

    def check_tf(self, target, source):

        try:

            return self.tf_buffer.can_transform(
                target,
                source,
                Time(),
            )

        except Exception:
            return False

    def check_readiness(self):

        now = time.monotonic()

        clock_stable = (
            now - self.last_clock_jump_wall
            >= self.clock_stable_seconds
        )

        clock_progressing = (
            self.last_clock_progress_wall is not None
            and now - self.last_clock_progress_wall < 2.0
        )

        clock_ok = (
            self.clock_count >= 5
            and clock_stable
            and clock_progressing
        )

        scan_ok = self.scan_received
        odom_ok = self.odom_received

        tf_odom_lidar = self.check_tf(
            "odom",
            "lidar_link",
        )

        if self.mode == "pre_slam":

            conditions = {
                "clock": clock_ok,
                "scan": scan_ok,
                "odom": odom_ok,
                "tf_odom_lidar": tf_odom_lidar,
            }

        elif self.mode == "pre_nav":

            tf_map_base = self.check_tf(
                "map",
                "base_link",
            )

            conditions = {
                "clock": clock_ok,
                "scan": scan_ok,
                "odom": odom_ok,
                "map": self.map_received,
                "tf_odom_lidar": tf_odom_lidar,
                "tf_map_base": tf_map_base,
            }

        else:
            self.get_logger().error(
                "Modo desconhecido."
            )
            return

        missing = [
            name
            for name, value in conditions.items()
            if not value
        ]

        if missing:

            self.get_logger().info(
                "Aguardando: " + ", ".join(missing)
            )

            return

        self.ready = True

        self.get_logger().info(
            "Todas as condições foram atendidas."
        )


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--mode",
        choices=["pre_slam", "pre_nav"],
        required=True,
    )

    args = parser.parse_args()

    rclpy.init()

    node = ReadinessMonitor(args.mode)

    try:

        while rclpy.ok() and not node.ready:

            rclpy.spin_once(
                node,
                timeout_sec=0.2,
            )

        success = node.ready

    except KeyboardInterrupt:

        success = False

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0 if success else 1


if __name__ == "__main__":

    raise SystemExit(main())
