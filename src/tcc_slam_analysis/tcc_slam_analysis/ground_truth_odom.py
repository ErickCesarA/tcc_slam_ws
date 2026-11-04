#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry


class GroundTruthOdom(Node):

    def __init__(self):
        super().__init__('ground_truth_odom')

        self.publisher = self.create_publisher(
            Odometry,
            '/ground_truth/odom',
            10
        )

        self.subscription = self.create_subscription(
            TFMessage,
            '/ground_truth/tf',
            self.tf_callback,
            10
        )

        self.target_frame = None

        self.get_logger().info(
            'Publicando ground truth a partir de /ground_truth/tf'
        )

    def tf_callback(self, msg):

        transforms = list(msg.transforms)

        if not transforms:
            return

        # Na primeira mensagem, identificar o referencial do robô.
        if self.target_frame is None:

            # Preferir o referencial base_link pertencente ao robô.
            candidates = [
                t for t in transforms
                if 'tcc_robot' in t.child_frame_id
                and (
                    t.child_frame_id.endswith('base_link')
                    or t.child_frame_id.endswith('::base_link')
                    or t.child_frame_id.endswith('/base_link')
                )
            ]

            # Se não houver base_link, procurar a pose do próprio modelo.
            if not candidates:
                candidates = [
                    t for t in transforms
                    if (
                        t.child_frame_id == 'tcc_robot'
                        or t.child_frame_id.endswith('::tcc_robot')
                        or t.child_frame_id.endswith('/tcc_robot')
                    )
                ]

            if not candidates:
                self.get_logger().warning(
                    'Não encontrei o referencial do robô. '
                    'Confira os child_frame_id em /ground_truth/tf.'
                )
                return

            self.target_frame = candidates[0].child_frame_id

            self.get_logger().info(
                f'Referencial selecionado: {self.target_frame}'
            )

        # Localizar a transformação selecionada.
        transform = next(
            (
                t for t in transforms
                if t.child_frame_id == self.target_frame
            ),
            None
        )

        if transform is None:
            return

        odom = Odometry()

        # Preservar o instante informado pela transformação.
        odom.header.stamp = transform.header.stamp

        # O referencial de origem deve corresponder ao da pose do Gazebo.
        odom.header.frame_id = (
            transform.header.frame_id or 'world'
        )

        odom.child_frame_id = self.target_frame

        # Posição real obtida da transformação do Gazebo.
        odom.pose.pose.position.x = transform.transform.translation.x
        odom.pose.pose.position.y = transform.transform.translation.y
        odom.pose.pose.position.z = transform.transform.translation.z

        # Orientação real do robô.
        odom.pose.pose.orientation = transform.transform.rotation

        # A velocidade não é estimada por este nó.
        # Para a análise de trajetória, utilizaremos principalmente a pose.
        self.publisher.publish(odom)


def main(args=None):

    rclpy.init(args=args)

    node = GroundTruthOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
