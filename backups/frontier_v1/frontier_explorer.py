import math
from collections import deque

import json
from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener, TransformException


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        # Parâmetros
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('free_threshold', 25)
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('blacklist_radius', 0.5)

        self.map_topic = self.get_parameter('map_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.free_threshold = self.get_parameter(
            'free_threshold').value
        self.min_frontier_size = self.get_parameter(
            'min_frontier_size').value
        self.blacklist_radius = self.get_parameter(
            'blacklist_radius').value

        self.map_data = None
        self.navigation_active = False
        self.goal_handle = None

        # Publisher dos eventos da exploração
        self.event_publisher = self.create_publisher(
            String,
            '/exploration/events',
            10
        )

        # Fronteiras recentemente visitadas ou que falharam
        self.blacklist = []

        # TF: posição atual do robô
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        # Assinatura do mapa
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos
        )

        # Action Client do Nav2
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        # Timer principal da exploração
        self.timer = self.create_timer(
            2.0,
            self.exploration_cycle
        )

        self.get_logger().info(
            'Frontier Explorer iniciado.'
        )

        self.publish_event('EXPLORATION_STARTED')

    def map_callback(self, msg):
        self.map_data = msg

    def publish_event(self, event, **kwargs):
        """
        Publica eventos estruturados da exploração em formato JSON.
        """

        timestamp = self.get_clock().now().nanoseconds / 1e9

        event_data = {
            'event': event,
            'timestamp': timestamp,
            **kwargs
        }

        message = String()
        message.data = json.dumps(event_data)

        self.event_publisher.publish(message)

        self.get_logger().info(
            f'Evento registrado: {event}'
        )
        
    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                Time(),
                timeout=Duration(seconds=0.2)
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            return x, y

        except TransformException as ex:
            self.get_logger().warning(
                f'Não foi possível obter TF do robô: {ex}'
            )
            return None

    def is_frontier(self, x, y, width, height, data):
        index = y * width + x
        value = data[index]

        # A célula precisa ser conhecida e livre
        if value < 0 or value > self.free_threshold:
            return False

        # Verifica as oito células vizinhas
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):

                if dx == 0 and dy == 0:
                    continue

                nx = x + dx
                ny = y + dy

                if not (0 <= nx < width and 0 <= ny < height):
                    continue

                neighbor_value = data[ny * width + nx]

                if neighbor_value == -1:
                    return True

        return False

    def find_frontiers(self):
        if self.map_data is None:
            return []

        width = self.map_data.info.width
        height = self.map_data.info.height
        data = self.map_data.data

        if width == 0 or height == 0:
            return []

        if len(data) != width * height:
            return []

        frontier_cells = set()

        # Identifica todas as células de fronteira
        for y in range(height):
            for x in range(width):

                if self.is_frontier(
                    x, y, width, height, data
                ):
                    frontier_cells.add((x, y))

        # Agrupa células de fronteira conectadas
        clusters = []

        while frontier_cells:

            start = frontier_cells.pop()
            queue = deque([start])
            cluster = [start]

            while queue:
                cx, cy = queue.popleft()

                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):

                        if dx == 0 and dy == 0:
                            continue

                        neighbor = (cx + dx, cy + dy)

                        if neighbor in frontier_cells:
                            frontier_cells.remove(neighbor)
                            queue.append(neighbor)
                            cluster.append(neighbor)

            if len(cluster) >= self.min_frontier_size:
                clusters.append(cluster)

        return clusters

    def cell_to_world(self, cell):
        map_info = self.map_data.info

        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y

        q = map_info.origin.orientation

        origin_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        cell_x, cell_y = cell

        local_x = (cell_x + 0.5) * resolution
        local_y = (cell_y + 0.5) * resolution

        world_x = (
            origin_x
            + local_x * math.cos(origin_yaw)
            - local_y * math.sin(origin_yaw)
        )

        world_y = (
            origin_y
            + local_x * math.sin(origin_yaw)
            + local_y * math.cos(origin_yaw)
        )

        return world_x, world_y

    def select_frontier(self, clusters, robot_x, robot_y):

        candidates = []

        # Remove fronteiras temporariamente bloqueadas
        now = self.get_clock().now().nanoseconds / 1e9

        self.blacklist = [
            item for item in self.blacklist
            if now - item['time'] < item['duration']
        ]

        for cluster in clusters:

            cluster_size = len(cluster)

            # Centro geométrico do agrupamento
            center_x = sum(p[0] for p in cluster) / cluster_size
            center_y = sum(p[1] for p in cluster) / cluster_size

            # Seleciona uma célula real próxima ao centro
            target_cell = min(
                cluster,
                key=lambda p: (
                    (p[0] - center_x) ** 2
                    + (p[1] - center_y) ** 2
                )
            )

            target_x, target_y = self.cell_to_world(target_cell)

            # Distância real entre robô e destino
            distance = math.hypot(
                target_x - robot_x,
                target_y - robot_y
            )

            # Pontuação: distância versus tamanho da fronteira
            score = distance / math.sqrt(cluster_size)

            candidates.append({
                'cell': target_cell,
                'x': target_x,
                'y': target_y,
                'distance': distance,
                'size': cluster_size,
                'score': score
            })

        # Ordena pela melhor pontuação
        candidates.sort(key=lambda c: c['score'])

        # Verifica se a fronteira está temporariamente bloqueada
        for candidate in candidates:

            blocked = False

            for item in self.blacklist:

                distance = math.hypot(
                    candidate['x'] - item['x'],
                    candidate['y'] - item['y']
                )

                if distance < self.blacklist_radius:
                    blocked = True
                    break

            if not blocked:
                return candidate

        return None

    def send_navigation_goal(self, x, y, robot_x, robot_y):

        goal = NavigateToPose.Goal()

        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        # Orienta o robô na direção do destino
        yaw = math.atan2(
            y - robot_y,
            x - robot_x
        )

        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.navigation_active = True

        self.publish_event(
            'GOAL_SENT',
            x=x,
            y=y
        )

        self.get_logger().info(
            f'Enviando destino: x={x:.2f}, y={y:.2f}'
        )

        future = self.nav_client.send_goal_async(goal)

        future.add_done_callback(
            lambda f: self.goal_response_callback(f, x, y)
        )

    def goal_response_callback(self, future, x, y):

        try:
            goal_handle = future.result()
        except Exception as ex:
            self.get_logger().error(
                f'Erro ao enviar meta: {ex}'
            )
            self.navigation_active = False
            self.add_to_blacklist(x, y)
            return

        if not goal_handle.accepted:
            self.get_logger().warning(
                'Nav2 rejeitou a meta.'
            )
            self.navigation_active = False
            self.add_to_blacklist(x, y)
            return

        self.goal_handle = goal_handle

        self.get_logger().info(
            'Meta aceita pelo Nav2. Navegação iniciada.'
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            lambda f: self.navigation_result_callback(f, x, y)
        )

    def navigation_result_callback(self, future, x, y):

        try:
            result = future.result()
            status = result.status

            # Status 4 = SUCCEEDED
            if status == 4:

                self.get_logger().info(
                    'Destino alcançado. Procurando nova fronteira.'
                )
                
                self.publish_event(
                    'GOAL_REACHED',
                    x=x,
                    y=y,
                status=int(status)
                )
                
                self.add_to_blacklist(
                    x, y, duration=10.0
                )

            else:

                self.get_logger().warning(
                    f'Navegação terminou com status {status}.'
                )

                self.publish_event(
                    'GOAL_FAILED',
                    x=x,
                    y=y,
                    status=int(status)
                )

                self.add_to_blacklist(
                    x, y, duration=60.0
                )

        except Exception as ex:

            self.get_logger().error(
                f'Erro ao receber resultado: {ex}'
            )
            
            self.publish_event(
                'GOAL_ERROR',
                x=x,
                y=y,
                error=str(ex)
            )
            
            self.add_to_blacklist(
                x, y, duration=60.0
            )

        self.navigation_active = False
        self.goal_handle = None

    def add_to_blacklist(self, x, y, duration=60.0):

        self.blacklist.append({
            'x': x,
            'y': y,
            'time': self.get_clock().now().nanoseconds / 1e9,
            'duration': duration
        })

    def exploration_cycle(self):

        if self.map_data is None:
            self.get_logger().info(
                'Aguardando mapa do SLAM Toolbox...'
            )
            return

        if self.navigation_active:
            return

        if not self.nav_client.server_is_ready():
            self.get_logger().warning(
                'Aguardando action /navigate_to_pose do Nav2...'
            )
            return

        robot_position = self.get_robot_position()

        if robot_position is None:
            return

        robot_x, robot_y = robot_position

        frontiers = self.find_frontiers()

        if not frontiers:
            self.get_logger().info(
                'Nenhuma fronteira válida encontrada.'
            )
            return

        target = self.select_frontier(
            frontiers,
            robot_x,
            robot_y
        )

        if target is None:
            self.get_logger().info(
                'Todas as fronteiras encontradas estão temporariamente bloqueadas.'
            )
            return

        self.get_logger().info(
            f'{len(frontiers)} agrupamentos encontrados. '
            f'Fronteira selecionada: {target["size"]} células. '
            f'Distância: {target["distance"]:.2f} m. '
            f'Pontuação: {target["score"]:.3f}.'
        )

        self.publish_event(
            'FRONTIER_SELECTED',
            x=target['x'],
            y=target['y'],
            distance=target['distance'],
            frontier_size=target['size'],
            score=target['score']
        )

        self.send_navigation_goal(
            target['x'],
            target['y'],
            robot_x,
            robot_y
        )


def main(args=None):

    rclpy.init(args=args)

    node = FrontierExplorer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
