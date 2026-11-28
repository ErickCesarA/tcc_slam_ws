#!/usr/bin/env python3
# TCC_ROUTE_END_V1
"""Controlador de rota de waypoints fixa, para o TCC de SLAM.

Usa as mesmas acoes Nav2 e os mesmos eventos analisados pelo projeto.
O arquivo de pontos e igual para todos os metodos SLAM de um mapa.
Nao le Ground Truth nem publica TF. Nao modifica o algoritmo de SLAM.

Nota cientifica: mesmos waypoints != trajetorias reais identicas.
"""
from __future__ import annotations

import json
import hashlib
import math
import time
from pathlib import Path

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class RotaFixa(Node):
    def __init__(self) -> None:
        super().__init__('fixed_route_runner')
        self.declare_parameter('map_id', 1)
        self.declare_parameter('route_file', '')
        self.declare_parameter('max_wait_wall_s', 120.0)
        self.declare_parameter('goal_timeout_sim_s', 0.0)
        self.declare_parameter('loop', False)

        self.map_id = int(self.get_parameter('map_id').value)
        path_param = str(self.get_parameter('route_file').value).strip()
        if path_param:
            self.route_file = Path(path_param).expanduser().resolve()
        else:
            share = Path(get_package_share_directory('tcc_slam_exploration'))
            self.route_file = share / 'routes' / f'mapa_{self.map_id}.json'
        if not self.route_file.is_file():
            raise FileNotFoundError(
                f'Rota para mapa {self.map_id} nao encontrada: {self.route_file}. '
                'Confirme a instalacao e execute colcon build --symlink-install.')
        raw_route = self.route_file.read_bytes()
        self.route_sha256 = hashlib.sha256(raw_route).hexdigest()
        config = json.loads(raw_route.decode('utf-8'))
        if int(config.get('mapa', -1)) != self.map_id:
            raise ValueError(f'Arquivo de rota nao corresponde ao mapa {self.map_id}.')
        if config.get('referencial') != 'map':
            raise ValueError('As rotas desta etapa devem usar referencial "map".')
        self.points = self._validate_points(config['waypoints'])
        self.loop = bool(self.get_parameter('loop').value or config.get('repetir', False))
        self.max_wait = float(self.get_parameter('max_wait_wall_s').value)
        self.goal_timeout = float(self.get_parameter('goal_timeout_sim_s').value)
        if self.max_wait <= 0 or self.goal_timeout < 0:
            raise ValueError('Tempos de espera devem ser positivos.')

        self.events_pub = self.create_publisher(String, '/exploration/events', 10)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.map_ready = False
        self.current_index = 0
        self.goal_handle = None
        self.goal_start_sim = None
        self.goal_sent_id = None
        self.goal_cancellation_requested = False
        self.active = False
        self.completed = False
        self.failed = False
        self.failed_since = None
        self.started_sim = None
        self.start_wall = time.monotonic()
        self.last_notice_wall = 0.0
        self.laps = 0
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'Rota fixa carregada: mapa={self.map_id}, {len(self.points)} pontos, '
            f'loop={self.loop}, arquivo={self.route_file}')
        self._event('ROUTE_LOADED', waypoints=len(self.points), loop=self.loop,
                    route_file=self.route_file.name)

    @staticmethod
    def _validate_points(raw):
        if not isinstance(raw, list) or not raw:
            raise ValueError('O arquivo precisa de uma lista nao vazia de waypoints.')
        points = []
        for index, item in enumerate(raw, 1):
            if not isinstance(item, dict) or 'x' not in item or 'y' not in item:
                raise ValueError(f'Waypoint {index}: informe x e y numericos.')
            x, y = float(item['x']), float(item['y'])
            yaw = float(item['yaw']) if 'yaw' in item else None
            if not all(map(math.isfinite, [x, y] + ([] if yaw is None else [yaw]))):
                raise ValueError(f'Waypoint {index} possui valor nao finito.')
            points.append((x, y, yaw))
        return points

    def _on_map(self, msg):
        if msg.info.width <= 0 or msg.info.height <= 0:
            return
        if msg.header.frame_id not in ('map', '/map'):
            return
        if not self.map_ready and any(cell >= 0 for cell in msg.data):
            self.map_ready = True
            self.get_logger().info('Mapa de ocupacao recebido e contem celulas conhecidas.')

    def now_sim(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _event(self, name, **details):
        evt = {'event': name, 'timestamp': self.now_sim(), 'mode': 'rota_fixa',
               'map_id': self.map_id, **details}
        msg = String()
        msg.data = json.dumps(evt, ensure_ascii=False)
        self.events_pub.publish(msg)
        self.get_logger().info(f'Evento: {name} | {details}')

    def _notice(self, message):
        now = time.monotonic()
        if now - self.last_notice_wall > 5.0:
            self.get_logger().info(message)
            self.last_notice_wall = now

    def _tick(self):
        if self.completed:
            # Sucesso: permanece vivo ate o executor observar ROUTE_COMPLETED.
            # Falha: sinaliza ao executor com codigo de saida nao zero.
            if self.failed and time.monotonic() - self.failed_since > 2.0:
                if rclpy.ok():
                    rclpy.shutdown()
            return
        now_sim = self.now_sim()
        if not self.active:
            if time.monotonic() - self.start_wall > self.max_wait:
                self._fail('ROUTE_START_TIMEOUT',
                           f'Nao ficou pronto em {self.max_wait:.0f}s reais.')
                return
            if now_sim <= 0.0:
                self._notice('Aguardando /clock...')
                return
            if not self.map_ready:
                self._notice('Aguardando mapa /map conhecido...')
                return
            if not self.nav.server_is_ready():
                self._notice('Aguardando action /navigate_to_pose...')
                return
            if not self.tf_buffer.can_transform('map', 'base_link', Time()):
                self._notice('Aguardando TF map -> base_link...')
                return
            self.active = True
            self.started_sim = now_sim
            self._event('ROUTE_STARTED', waypoints=len(self.points),
                        route_sha256=self.route_sha256)
            self._send_next_goal()
            return

        if (self.goal_timeout > 0 and self.goal_handle is not None and self.goal_start_sim is not None
                and not self.goal_cancellation_requested
                and now_sim - self.goal_start_sim > self.goal_timeout):
            self.goal_cancellation_requested = True
            self._event('GOAL_TIMEOUT', goal_id=self.goal_sent_id,
                        waypoint_index=self.current_index + 1)
            self._event('ROUTE_FAILED', reason='goal_timeout',
                        waypoint_index=self.current_index + 1)
            self.completed = True
            self.failed = True
            self.failed_since = time.monotonic()
            self.goal_handle.cancel_goal_async()
            self.get_logger().error('Meta excedeu timeout; rota interrompida.')

    def _pose(self, index):
        x, y, yaw = self.points[index]
        if yaw is None:
            if index + 1 < len(self.points):
                nx, ny, _ = self.points[index + 1]
                yaw = math.atan2(ny-y, nx-x)
            elif len(self.points) >= 2:
                px, py, _ = self.points[index-1]
                yaw = math.atan2(y-py, x-px)
            else:
                yaw = 0.0
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw/2)
        pose.pose.orientation.w = math.cos(yaw/2)
        return pose

    def _send_next_goal(self):
        if self.completed:
            return
        if self.current_index >= len(self.points):
            self.laps += 1
            self._event('ROUTE_LAP_COMPLETED', lap=self.laps,
                        waypoints=len(self.points))
            if not self.loop:
                self.completed = True
                self._event('ROUTE_COMPLETED', lap=self.laps,
                            elapsed_sim_s=self.now_sim()-self.started_sim)
                self.get_logger().info('Rota concluida. Aguardando o executor finalizar a gravacao.')
                return
            self.current_index = 0

        if not self.nav.server_is_ready():
            self._fail('ROUTE_FAILED', 'Action Nav2 indisponivel durante rota.')
            return
        index = self.current_index
        x, y, _ = self.points[index]
        self.goal_sent_id = self.laps * len(self.points) + index + 1
        self.goal_start_sim = self.now_sim()
        self.goal_cancellation_requested = False
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(index)
        try:
            fut = self.nav.send_goal_async(goal)
        except Exception as ex:
            self._fail('GOAL_ERROR', str(ex))
            return
        self._event('GOAL_SENT', goal_id=self.goal_sent_id,
                    waypoint_index=index+1, x=x, y=y, lap=self.laps+1)
        fut.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        if self.completed:
            return
        try:
            handle = future.result()
        except Exception as ex:
            self._fail('GOAL_ERROR', str(ex))
            return
        if not handle.accepted:
            self._fail('GOAL_REJECTED', 'Nav2 rejeitou waypoint.')
            return
        self.goal_handle = handle
        self._event('GOAL_ACCEPTED', goal_id=self.goal_sent_id,
                    waypoint_index=self.current_index+1)
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        if self.completed:
            return
        try:
            status = future.result().status
        except Exception as ex:
            self._fail('GOAL_ERROR', str(ex))
            return
        self.goal_handle = None
        x, y, _ = self.points[self.current_index]
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._event('GOAL_REACHED', goal_id=self.goal_sent_id,
                        waypoint_index=self.current_index+1, x=x, y=y)
            self.current_index += 1
            self._send_next_goal()
        elif status == GoalStatus.STATUS_CANCELED:
            self._fail('GOAL_CANCELLED', 'Nav2 cancelou objetivo.')
        else:
            self._fail('GOAL_FAILED', f'Nav2 terminou com status {status}.')

    def _fail(self, event, detail):
        self._event(event, goal_id=self.goal_sent_id,
                    waypoint_index=self.current_index+1, reason=detail)
        if event != 'ROUTE_FAILED':
            self._event('ROUTE_FAILED', goal_id=self.goal_sent_id,
                        waypoint_index=self.current_index+1, reason=detail)
        self.get_logger().error(detail)
        self.completed = True
        self.failed = True
        self.failed_since = time.monotonic()
        # O proprio processo sai com codigo 1 para nao registrar como
        # experimento valido uma rota interrompida.


def main(args=None):
    rclpy.init(args=args)
    node = None
    failed = False
    try:
        node = RotaFixa()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            failed = node.failed
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
