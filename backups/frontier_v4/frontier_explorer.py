#!/usr/bin/env python3
"""Frontier Explorer V4: V3 + reavaliacao controlada de metas em movimento.

Preempcao com histerese, tempo minimo entre trocas e cancelamento confirmado
pelo Nav2. Preserva ganho de informacao, validacao de caminhos e eventos.
"""
import json
import math
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def angle_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('free_threshold', 25)
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('blacklist_radius', 0.5)
        self.declare_parameter('exploration_period_s', 2.0)
        self.declare_parameter('safety_radius_m', 0.30)
        self.declare_parameter('standoff_min_m', 0.25)
        self.declare_parameter('standoff_max_m', 0.85)
        self.declare_parameter('candidate_limit', 5)
        self.declare_parameter('goal_min_distance_m', 0.35)
        self.declare_parameter('plan_timeout_s', 8.0)
        self.declare_parameter('planning_weight_size', 0.5)
        self.declare_parameter('blacklist_failure_s', 60.0)
        self.declare_parameter('blacklist_success_s', 10.0)
        self.declare_parameter('planner_id', '')
        self.declare_parameter('use_path_validation', True)
        # Ganho de informacao: valores 0 desativam o criterio (modo V2).
        self.declare_parameter('information_gain_weight', 0.75)
        self.declare_parameter('information_gain_radius_m', 3.0)
        self.declare_parameter('information_gain_rays', 64)
        self.declare_parameter('information_gain_obstacle_threshold', 65)
        # V4: parametros da reavaliacao (todos ajustaveis em ROS 2).
        self.declare_parameter('enable_goal_reevaluation', True)
        self.declare_parameter('reevaluation_period_s', 8.0)
        self.declare_parameter('minimum_goal_age_s', 12.0)
        self.declare_parameter('minimum_switch_interval_s', 20.0)
        self.declare_parameter('switch_improvement_fraction', 0.30)
        self.declare_parameter('switch_min_distance_m', 0.85)
        self.declare_parameter('max_goal_switches', 5)
        self.declare_parameter('cancel_timeout_s', 8.0)
        self.declare_parameter('switch_target_separation_m', 0.75)

        def p(name):
            return self.get_parameter(name).value

        self.map_topic = str(p('map_topic'))
        self.map_frame = str(p('map_frame'))
        self.robot_frame = str(p('robot_frame'))
        self.free_threshold = int(p('free_threshold'))
        self.min_frontier_size = int(p('min_frontier_size'))
        self.blacklist_radius = float(p('blacklist_radius'))
        self.safety_radius_m = float(p('safety_radius_m'))
        self.standoff_min_m = float(p('standoff_min_m'))
        self.standoff_max_m = float(p('standoff_max_m'))
        self.candidate_limit = max(1, int(p('candidate_limit')))
        self.goal_min_distance_m = float(p('goal_min_distance_m'))
        self.plan_timeout_s = float(p('plan_timeout_s'))
        self.planning_weight_size = float(p('planning_weight_size'))
        self.blacklist_failure_s = float(p('blacklist_failure_s'))
        self.blacklist_success_s = float(p('blacklist_success_s'))
        self.planner_id = str(p('planner_id'))
        self.use_path_validation = bool(p('use_path_validation'))
        self.information_gain_weight = float(p('information_gain_weight'))
        self.information_gain_radius_m = float(p('information_gain_radius_m'))
        self.information_gain_rays = int(p('information_gain_rays'))
        self.information_gain_obstacle_threshold = int(
            p('information_gain_obstacle_threshold'))
        self.enable_goal_reevaluation = bool(p('enable_goal_reevaluation'))
        self.reevaluation_period_s = float(p('reevaluation_period_s'))
        self.minimum_goal_age_s = float(p('minimum_goal_age_s'))
        self.minimum_switch_interval_s = float(p('minimum_switch_interval_s'))
        self.switch_improvement_fraction = float(p('switch_improvement_fraction'))
        self.switch_min_distance_m = float(p('switch_min_distance_m'))
        self.max_goal_switches = int(p('max_goal_switches'))
        self.cancel_timeout_s = float(p('cancel_timeout_s'))
        self.switch_target_separation_m = float(p('switch_target_separation_m'))
        if not (0.0 <= self.planning_weight_size <= 1.0):
            raise ValueError('planning_weight_size deve estar entre 0 e 1')
        if self.standoff_min_m < 0 or self.standoff_max_m < self.standoff_min_m:
            raise ValueError('standoff_min_m/max_m invalidos')
        if self.safety_radius_m < 0:
            raise ValueError('safety_radius_m nao pode ser negativo')
        if not math.isfinite(self.information_gain_weight) or self.information_gain_weight < 0:
            raise ValueError('information_gain_weight deve ser >= 0 e finito')
        if not (0.0 < self.information_gain_radius_m <= 30.0):
            raise ValueError('information_gain_radius_m deve estar em (0, 30] m')
        if not (8 <= self.information_gain_rays <= 360):
            raise ValueError('information_gain_rays deve estar entre 8 e 360')
        if not (1 <= self.information_gain_obstacle_threshold <= 100):
            raise ValueError('information_gain_obstacle_threshold invalido')
        if self.reevaluation_period_s <= 0 or self.minimum_goal_age_s < 0:
            raise ValueError('Intervalos de reavaliacao invalidos')
        if self.minimum_switch_interval_s < 0 or self.cancel_timeout_s <= 0:
            raise ValueError('Intervalos de troca/cancelamento invalidos')
        if not 0.0 < self.switch_improvement_fraction < 1.0:
            raise ValueError('switch_improvement_fraction deve estar em (0,1)')
        if self.switch_min_distance_m < 0 or self.switch_target_separation_m < 0:
            raise ValueError('Distancias de troca invalidas')
        if self.max_goal_switches < 0:
            raise ValueError('max_goal_switches deve ser >= 0')

        self.map_data = None
        self.blacklist = []
        # IDLE, PLANNING, SENDING, NAVIGATING, REEVALUATING, CANCELING
        self.state = 'IDLE'
        self.goal_handle = None
        self.current = None
        self.goal_counter = 0
        self.pending_candidates = []
        self.planning_started = 0.0
        self.current_plan_token = 0
        self.current_plan_handle = None
        self.plan_candidate = None
        self.reachable_candidates = []
        self.last_notice = {}
        self.planning_mode = None  # 'initial' ou 'reevaluation'
        self.reeval_goal_id = None
        self.last_reevaluation_s = -1e20
        self.last_switch_s = -1e20
        self.goal_switch_count = 0
        self.pending_switch_candidate = None
        self.cancel_requested_s = None
        self.cancel_acknowledged = False
        self.cancel_token = 0

        self.event_publisher = self.create_publisher(
            String, '/exploration/events', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self.map_topic,
                                 self.map_callback, qos)
        self.nav_client = ActionClient(self, NavigateToPose,
                                       '/navigate_to_pose')
        self.plan_client = ActionClient(self, ComputePathToPose,
                                        '/compute_path_to_pose')
        self.create_timer(float(p('exploration_period_s')),
                          self.exploration_cycle)
        self.create_timer(0.5, self.planning_watchdog)
        self.create_timer(0.5, self.cancellation_watchdog)
        self.get_logger().info(
            f'Frontier Explorer V4 iniciado: ganho peso={self.information_gain_weight:.2f}, '
            f'alcance={self.information_gain_radius_m:.2f} m, raios={self.information_gain_rays}.')
        self.publish_event('EXPLORATION_STARTED', version='4.0',
                           reevaluation_enabled=self.enable_goal_reevaluation,
                           reevaluation_period_s=self.reevaluation_period_s,
                           switch_improvement_fraction=self.switch_improvement_fraction,
                           information_gain_weight=self.information_gain_weight,
                           information_gain_radius_m=self.information_gain_radius_m)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def notice(self, key, message, interval=10.0):
        now = self.now_s()
        if now - self.last_notice.get(key, -1e10) >= interval:
            self.last_notice[key] = now
            self.get_logger().info(message)

    def publish_event(self, event, **fields):
        obj = {'event': event, 'timestamp': self.now_s(), **fields}
        message = String()
        message.data = json.dumps(obj, ensure_ascii=False)
        self.event_publisher.publish(message)
        self.get_logger().info(f'Evento: {event} | {fields}')

    def map_callback(self, msg):
        if msg.info.width and msg.info.height and \
                len(msg.data) == msg.info.width * msg.info.height:
            self.map_data = msg

    def robot_position(self):
        try:
            tr = self.tf_buffer.lookup_transform(self.map_frame,
                self.robot_frame, Time(), timeout=Duration(seconds=0.1))
            return tr.transform.translation.x, tr.transform.translation.y
        except TransformException as exc:
            self.notice('tf', f'Aguardando TF do robo: {exc}')
            return None

    def is_frontier(self, x, y, width, height, data):
        val = data[y * width + x]
        if val < 0 or val > self.free_threshold:
            return False
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height and \
                        data[ny * width + nx] == -1:
                    return True
        return False

    def find_frontiers(self):
        if self.map_data is None:
            return []
        w, h = self.map_data.info.width, self.map_data.info.height
        data = self.map_data.data
        cells = set()
        for y in range(h):
            for x in range(w):
                if self.is_frontier(x, y, w, h, data):
                    cells.add((x, y))
        groups = []
        while cells:
            start = cells.pop()
            queue = deque([start])
            group = [start]
            while queue:
                cx, cy = queue.popleft()
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        item = (cx + dx, cy + dy)
                        if item in cells:
                            cells.remove(item)
                            group.append(item)
                            queue.append(item)
            if len(group) >= self.min_frontier_size:
                groups.append(group)
        return groups

    def cell_to_world(self, cell):
        info = self.map_data.info
        x = (cell[0] + .5) * info.resolution
        y = (cell[1] + .5) * info.resolution
        yaw = angle_from_quaternion(info.origin.orientation)
        return (info.origin.position.x + math.cos(yaw)*x - math.sin(yaw)*y,
                info.origin.position.y + math.sin(yaw)*x + math.cos(yaw)*y)

    def world_to_cell(self, x, y):
        info = self.map_data.info
        dx, dy = x-info.origin.position.x, y-info.origin.position.y
        yaw = angle_from_quaternion(info.origin.orientation)
        return (math.floor((math.cos(yaw)*dx + math.sin(yaw)*dy) /
                           info.resolution),
                math.floor((-math.sin(yaw)*dx + math.cos(yaw)*dy) /
                           info.resolution))

    def safe_cell(self, cell):
        """Conservative check: entire disk known free in occupancy grid.

        Nav2's costmap and planner are still authoritative on reachability.
        """
        if self.map_data is None or self.map_data.info.resolution <= 0:
            return False
        w, h = self.map_data.info.width, self.map_data.info.height
        cx, cy = cell
        radius = math.ceil(self.safety_radius_m / self.map_data.info.resolution)
        data = self.map_data.data
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx*dx + dy*dy > (self.safety_radius_m /
                                    self.map_data.info.resolution)**2:
                    continue
                x, y = cx + dx, cy + dy
                if not (0 <= x < w and 0 <= y < h):
                    return False
                v = data[y * w + x]
                if v < 0 or v > self.free_threshold:
                    return False
        return True

    def blocked(self, x, y):
        now = self.now_s()
        self.blacklist = [b for b in self.blacklist
                          if now - b['time'] < b['duration']]
        return any(math.hypot(x-b['x'], y-b['y']) < self.blacklist_radius
                   for b in self.blacklist)

    def blacklist_goal(self, candidate, reason, duration):
        if candidate is None:
            return
        self.blacklist.append({'x': candidate['x'], 'y': candidate['y'],
                               'reason': reason, 'time': self.now_s(),
                               'duration': duration})

    @staticmethod
    def line_cells(x0, y0, x1, y1):
        """Itera celulas na linha de Bresenham, inclusive origem e destino."""
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            yield x0, y0
            if x0 == x1 and y0 == y1:
                break
            double_err = 2 * err
            if double_err > -dy:
                err -= dy
                x0 += sx
            if double_err < dx:
                err += dx
                y0 += sy

    def estimate_information_gain(self, candidate_cell):
        """Conta celulas desconhecidas potencialmente visiveis em raios 2D.

        Oclusao apenas por ocupacao CONHECIDA (>= limiar). Celulas
        desconhecidas nao bloqueiam: estimativa otimista, nao ray tracing
        fisico. Celulas sao contadas uma vez, mesmo vistas por varios raios.
        """
        info = self.map_data.info
        w, h = info.width, info.height
        radius_cells = self.information_gain_radius_m / info.resolution
        cx, cy = candidate_cell
        data = self.map_data.data
        unseen = set()
        for i in range(self.information_gain_rays):
            angle = 2.0 * math.pi * i / self.information_gain_rays
            ex = cx + round(radius_cells * math.cos(angle))
            ey = cy + round(radius_cells * math.sin(angle))
            for x, y in self.line_cells(cx, cy, ex, ey):
                if not (0 <= x < w and 0 <= y < h):
                    break
                value = data[y*w+x]
                if value >= self.information_gain_obstacle_threshold:
                    break
                if value == -1:
                    unseen.add(y*w+x)
        return len(unseen)

    def information_factor(self, candidate):
        """Bonus limitado ao maior ganho dentre candidatos da rodada."""
        if self.information_gain_weight <= 0.0:
            return 1.0
        return 1.0 + self.information_gain_weight * candidate.get('gain_ratio', 0.0)

    def normalize_gains(self, candidates):
        maximum = max((c.get('information_gain_cells', 0) for c in candidates),
                      default=0)
        for candidate in candidates:
            candidate['gain_ratio'] = (candidate.get('information_gain_cells', 0)
                                       / maximum if maximum > 0 else 0.0)

    def candidates(self, groups, robot_x, robot_y):
        """Find free standoff cells around a group's centroid, rank cheaply.

        Limit scans to an annulus around each frontier center. Exclude cells
        occupied/unknown or too near an obstacle. Return one goal per group.
        """
        info = self.map_data.info
        resolution = info.resolution
        if resolution <= 0:
            return []
        min_r = max(1, math.ceil(self.standoff_min_m / resolution))
        max_r = max(min_r, math.ceil(self.standoff_max_m / resolution))
        candidates = []
        for cluster in groups:
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)
            representative = min(cluster, key=lambda p:
                                 (p[0]-cx)**2+(p[1]-cy)**2)
            best = None
            # Search standoff around centroid, not necessarily on frontier.
            for dy in range(-max_r, max_r + 1):
                for dx in range(-max_r, max_r + 1):
                    r2 = dx*dx + dy*dy
                    if r2 < min_r*min_r or r2 > max_r*max_r:
                        continue
                    cell = (representative[0]+dx,
                            representative[1]+dy)
                    if not self.safe_cell(cell):
                        continue
                    x, y = self.cell_to_world(cell)
                    dist = math.hypot(x-robot_x, y-robot_y)
                    if dist < self.goal_min_distance_m or self.blocked(x, y):
                        continue
                    # Keep the nearest suitable approach point for this group.
                    deviation = math.sqrt((cell[0]-cx)**2 +
                                          (cell[1]-cy)**2) * resolution
                    cheap = dist + 0.15 * deviation
                    if best is None or cheap < best['cheap']:
                        best = {'x': x, 'y': y, 'cell': cell, 'size': len(cluster),
                                'distance': dist, 'cheap': cheap,
                                'frontier_cell': representative}
            if best:
                best['information_gain_cells'] = (
                    self.estimate_information_gain(best['cell'])
                    if self.information_gain_weight > 0.0 else 0)
                candidates.append(best)
        self.normalize_gains(candidates)
        for candidate in candidates:
            candidate['score'] = (
                candidate['distance']
                / (candidate['size'] ** self.planning_weight_size)
                / self.information_factor(candidate))
        candidates.sort(key=lambda c: (c['score'], c['distance']))
        return candidates[:self.candidate_limit]

    def pose_stamped(self, x, y, robot_x, robot_y):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        yaw = math.atan2(y-robot_y, x-robot_x)
        pose.pose.orientation.z = math.sin(yaw/2.0)
        pose.pose.orientation.w = math.cos(yaw/2.0)
        return pose

    def exploration_cycle(self):
        if self.state == 'NAVIGATING':
            self.maybe_reevaluate()
            return
        if self.state != 'IDLE':
            return
        if self.map_data is None:
            self.notice('map', 'Aguardando mapa de ocupacao /map...')
            return
        if not self.nav_client.server_is_ready():
            self.notice('nav', 'Aguardando action /navigate_to_pose...')
            return
        if self.use_path_validation and not self.plan_client.server_is_ready():
            self.notice('plan', 'Aguardando action /compute_path_to_pose...')
            return
        robot = self.robot_position()
        if robot is None:
            return
        groups = self.find_frontiers()
        if not groups:
            self.notice('none', 'Nenhuma fronteira valida encontrada.')
            return
        candidates = self.candidates(groups, *robot)
        if not candidates:
            self.notice('blocked', 'Sem destinos seguros nas fronteiras atuais.')
            return
        self.get_logger().info(
            f'{len(groups)} fronteiras; {len(candidates)} destinos candidatos seguros.')
        self.pending_candidates = candidates
        self.reachable_candidates = []
        self.plan_robot = robot
        self.planning_mode = 'initial'
        if self.use_path_validation:
            self.state = 'PLANNING'
            self.plan_next_candidate()
        else:
            candidate = self.pending_candidates.pop(0)
            self.commit_goal(candidate, None)

    def planning_watchdog(self):
        if self.state not in ('PLANNING', 'REEVALUATING'):
            return
        if self.now_s() - self.planning_started > self.plan_timeout_s:
            self.get_logger().warning('Timeout ao planejar rota; descartando candidato.')
            self.current_plan_token += 1
            # Cancel active planning goal (best effort) so the planner isn't loaded.
            if self.current_plan_handle is not None:
                try:
                    self.current_plan_handle.cancel_goal_async()
                except Exception:
                    pass
            self.current_plan_handle = None
            self.plan_next_candidate()

    def plan_next_candidate(self):
        if not self.pending_candidates:
            if self.planning_mode == 'reevaluation':
                self.finish_reevaluation()
            elif self.reachable_candidates:
                best = min(self.reachable_candidates,
                           key=lambda c: (c['path_score'], c['path_length_m']))
                self.planning_mode = None
                self.commit_goal(best, best['path_length_m'])
            else:
                self.planning_mode = None
                self.state = 'IDLE'
                self.notice('unreachable', 'Todas as candidatas foram rejeitadas pelo planejador.')
            return
        candidate = self.pending_candidates.pop(0)
        self.plan_candidate = candidate
        self.current_plan_handle = None
        self.current_plan_token += 1
        token = self.current_plan_token
        self.planning_started = self.now_s()
        goal = ComputePathToPose.Goal()
        goal.goal = self.pose_stamped(candidate['x'], candidate['y'],
                                      *self.plan_robot)
        goal.planner_id = self.planner_id
        goal.use_start = False  # planner obtains latest robot pose from TF
        future = self.plan_client.send_goal_async(goal)
        future.add_done_callback(lambda f, t=token:
                                 self.plan_accepted_callback(f, t))

    def plan_accepted_callback(self, future, token):
        if self.state not in ('PLANNING', 'REEVALUATING') or token != self.current_plan_token:
            return
        try:
            handle = future.result()
            if not handle.accepted:
                self.reject_candidate('PLANNER_GOAL_REJECTED')
                return
            self.current_plan_handle = handle
            handle.get_result_async().add_done_callback(
                lambda f, t=token: self.plan_result_callback(f, t))
        except Exception as exc:
            self.reject_candidate(f'PLANNER_ERROR: {exc}')

    def plan_result_callback(self, future, token):
        if self.state not in ('PLANNING', 'REEVALUATING') or token != self.current_plan_token:
            return
        try:
            response = future.result()
            poses = response.result.path.poses
            if response.status != GoalStatus.STATUS_SUCCEEDED or len(poses) < 2:
                self.reject_candidate(f'NO_PATH_STATUS_{response.status}')
                return
            length = sum(math.hypot(b.pose.position.x-a.pose.position.x,
                                    b.pose.position.y-a.pose.position.y)
                         for a, b in zip(poses[:-1], poses[1:]))
            self.current_plan_handle = None
            candidate = self.plan_candidate
            # Custo do caminho real, beneficio relativo ao ganho estimado.
            candidate['path_length_m'] = length
            candidate['path_score'] = (
                length / (candidate['size'] ** self.planning_weight_size)
                / self.information_factor(candidate))
            # Compare real path costs for the reachable shortlisted candidates.
            self.reachable_candidates.append(candidate)
            self.plan_next_candidate()
        except Exception as exc:
            self.reject_candidate(f'PLANNER_RESULT_ERROR: {exc}')

    def reject_candidate(self, reason):
        c = self.plan_candidate
        self.publish_event('FRONTIER_UNREACHABLE', x=c['x'], y=c['y'],
                           reason=reason)
        self.blacklist_goal(c, reason, self.blacklist_failure_s)
        self.current_plan_handle = None
        self.plan_next_candidate()

    def commit_goal(self, candidate, length):
        self.goal_counter += 1
        goal_id = self.goal_counter
        self.current = candidate
        self.current['goal_id'] = goal_id
        self.current['start_time'] = self.now_s()
        self.current['initial_robot_xy'] = tuple(self.plan_robot)
        self.state = 'SENDING'
        self.publish_event('FRONTIER_SELECTED', goal_id=goal_id,
                           x=candidate['x'], y=candidate['y'],
                           distance=candidate['distance'],
                           path_length_m=length,
                           frontier_size=candidate['size'],
                           score=candidate.get('path_score', candidate['score']),
                           information_gain_cells=candidate.get('information_gain_cells', 0),
                           information_gain_ratio=candidate.get('gain_ratio', 0.0),
                           information_gain_weight=self.information_gain_weight)
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self.pose_stamped(candidate['x'], candidate['y'],
                                          *self.plan_robot)
        self.publish_event('GOAL_SENT', goal_id=goal_id,
                           x=candidate['x'], y=candidate['y'],
                           path_length_m=length,
                           information_gain_cells=candidate.get('information_gain_cells', 0))
        try:
            future = self.nav_client.send_goal_async(nav_goal)
            future.add_done_callback(lambda f, gid=goal_id:
                                     self.goal_response_callback(f, gid))
        except Exception as exc:
            self.goal_terminal('GOAL_ERROR', goal_id,
                               reason=f'SEND_ERROR: {exc}')

    def goal_response_callback(self, future, goal_id):
        if self.state != 'SENDING' or self.current is None or \
                self.current['goal_id'] != goal_id:
            return
        try:
            handle = future.result()
            if not handle.accepted:
                self.goal_terminal('GOAL_REJECTED', goal_id,
                                   reason='NAV2_REJECTED')
                return
            self.goal_handle = handle
            self.state = 'NAVIGATING'
            self.publish_event('GOAL_ACCEPTED', goal_id=goal_id,
                               x=self.current['x'], y=self.current['y'])
            handle.get_result_async().add_done_callback(
                lambda f, gid=goal_id:
                self.navigation_result_callback(f, gid))
        except Exception as exc:
            self.goal_terminal('GOAL_ERROR', goal_id,
                               reason=f'ACCEPT_ERROR: {exc}')

    def navigation_result_callback(self, future, goal_id):
        if self.state not in ('NAVIGATING', 'REEVALUATING', 'CANCELING') or self.current is None or \
                self.current['goal_id'] != goal_id:
            return
        try:
            response = future.result()
            if response.status == GoalStatus.STATUS_SUCCEEDED:
                self.goal_terminal('GOAL_REACHED', goal_id,
                                   status=int(response.status))
            elif response.status == GoalStatus.STATUS_CANCELED:
                self.goal_terminal('GOAL_CANCELLED', goal_id,
                                   status=int(response.status))
            else:
                self.goal_terminal('GOAL_FAILED', goal_id,
                                   status=int(response.status))
        except Exception as exc:
            self.goal_terminal('GOAL_ERROR', goal_id,
                               reason=f'RESULT_ERROR: {exc}')

    def invalidate_planning(self):
        """Desconsidera callbacks atrasados de qualquer planejamento."""
        self.current_plan_token += 1
        if self.current_plan_handle is not None:
            try:
                self.current_plan_handle.cancel_goal_async()
            except Exception:
                pass
        self.current_plan_handle = None
        self.plan_candidate = None
        self.planning_mode = None
        self.reeval_goal_id = None

    def abort_reevaluation(self, reason):
        """Nao interfere na navegacao em andamento."""
        if self.state != 'REEVALUATING':
            return
        goal_id = self.reeval_goal_id
        self.invalidate_planning()
        self.pending_candidates = []
        self.reachable_candidates = []
        self.state = 'NAVIGATING'
        self.publish_event('GOAL_REEVALUATION_SKIPPED', goal_id=goal_id,
                           reason=reason)

    def maybe_reevaluate(self):
        """Replaneja destinos potenciais enquanto a meta original segue ativa."""
        if not self.enable_goal_reevaluation or not self.use_path_validation:
            return
        if self.goal_handle is None or self.current is None:
            return
        now = self.now_s()
        if self.goal_switch_count >= self.max_goal_switches:
            return
        if now - self.last_reevaluation_s < self.reevaluation_period_s:
            return
        if now - self.current['start_time'] < self.minimum_goal_age_s:
            return
        if now - self.last_switch_s < self.minimum_switch_interval_s:
            return
        if not self.plan_client.server_is_ready():
            return
        robot = self.robot_position()
        if robot is None or self.map_data is None:
            return
        current_distance = math.hypot(self.current['x'] - robot[0],
                                      self.current['y'] - robot[1])
        if current_distance < self.switch_min_distance_m:
            return
        groups = self.find_frontiers()
        if not groups:
            self.last_reevaluation_s = now
            return
        alternatives = self.candidates(groups, *robot)
        alternatives = [c for c in alternatives if
                        math.hypot(c['x'] - self.current['x'],
                                   c['y'] - self.current['y']) >=
                        self.switch_target_separation_m]
        self.last_reevaluation_s = now
        if not alternatives:
            return
        self.reeval_goal_id = self.current['goal_id']
        self.pending_candidates = alternatives
        self.reachable_candidates = []
        self.plan_robot = robot
        self.planning_mode = 'reevaluation'
        self.state = 'REEVALUATING'
        self.publish_event('GOAL_REEVALUATION_STARTED',
                           goal_id=self.reeval_goal_id,
                           candidates=len(alternatives),
                           remaining_euclidean_m=current_distance)
        self.plan_next_candidate()

    def finish_reevaluation(self):
        """Aplica a mesma normalizacao de ganho para atual e alternativas."""
        if self.state != 'REEVALUATING':
            return
        if self.current is None or self.current.get('goal_id') != self.reeval_goal_id:
            self.abort_reevaluation('GOAL_CHANGED')
            return
        if not self.reachable_candidates:
            self.abort_reevaluation('NO_REACHABLE_ALTERNATIVE')
            return
        robot = self.robot_position()
        if robot is None or self.map_data is None:
            self.abort_reevaluation('TF_UNAVAILABLE')
            return
        active = self.current
        remaining_euclid = math.hypot(active['x']-robot[0], active['y']-robot[1])
        if remaining_euclid < self.switch_min_distance_m:
            self.abort_reevaluation('NEAR_CURRENT_GOAL')
            return
        current_cell = self.world_to_cell(active['x'], active['y'])
        active_gain = self.estimate_information_gain(current_cell)
        maximum_gain = max([active_gain] + [c['information_gain_cells']
                           for c in self.reachable_candidates])
        def factor(gain):
            return 1.0 + (self.information_gain_weight * gain / maximum_gain
                          if maximum_gain > 0 else 0.0)
        # Estimativa conservadora do caminho restante: extrapola o caminho
        # inicial em funcao da distancia euclidiana ainda a percorrer.
        # Nunca supoe distancia menor que a euclidiana atual.
        initial_distance = math.hypot(
            active['x']-active['initial_robot_xy'][0],
            active['y']-active['initial_robot_xy'][1])
        initial_path = active.get('path_length_m') or initial_distance
        remaining_path = max(
            remaining_euclid,
            initial_path * min(1.0, remaining_euclid / max(initial_distance, 0.01)))
        active_score = remaining_path / max(
            active['size'], 1) ** self.planning_weight_size / factor(active_gain)
        for candidate in self.reachable_candidates:
            candidate['switch_score'] = (
                candidate['path_length_m'] /
                max(candidate['size'], 1) ** self.planning_weight_size /
                factor(candidate['information_gain_cells']))
        best = min(self.reachable_candidates,
                   key=lambda c: (c['switch_score'], c['path_length_m']))
        improvement = (1.0 - best['switch_score']/active_score
                       if active_score > 0 else -math.inf)
        self.publish_event('GOAL_REEVALUATED',
                           goal_id=active['goal_id'],
                           current_score=active_score,
                           alternative_score=best['switch_score'],
                           improvement_fraction=improvement,
                           alternative_x=best['x'],
                           alternative_y=best['y'],
                           switch_threshold=self.switch_improvement_fraction)
        self.invalidate_planning()
        self.pending_candidates = []
        self.reachable_candidates = []
        if improvement < self.switch_improvement_fraction:
            self.state = 'NAVIGATING'
            return
        if self.goal_handle is None or not self.nav_client.server_is_ready():
            self.state = 'NAVIGATING'
            return
        if self.goal_switch_count >= self.max_goal_switches:
            self.state = 'NAVIGATING'
            return
        self.pending_switch_candidate = best
        self.state = 'CANCELING'
        self.cancel_requested_s = self.now_s()
        self.cancel_acknowledged = False
        self.cancel_token += 1
        token = self.cancel_token
        goal_id = active['goal_id']
        self.publish_event('GOAL_SWITCH_REQUESTED',
                           goal_id=goal_id,
                           alternative_x=best['x'],
                           alternative_y=best['y'],
                           improvement_fraction=improvement)
        try:
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(
                lambda f, gid=goal_id, tok=token:
                self.cancel_response_callback(f, gid, tok))
        except Exception as exc:
            self.cancel_failed(f'CANCEL_SEND_ERROR: {exc}')

    def cancel_response_callback(self, future, goal_id, token):
        if self.state != 'CANCELING' or token != self.cancel_token or \
                self.current is None or self.current.get('goal_id') != goal_id:
            return
        try:
            result = future.result()
            if not result.goals_canceling:
                self.cancel_failed('CANCEL_NOT_ACCEPTED')
                return
            self.cancel_acknowledged = True
        except Exception as exc:
            self.cancel_failed(f'CANCEL_RESPONSE_ERROR: {exc}')

    def cancel_failed(self, reason):
        """Conserva a meta anterior e impede envio simultaneo de metas."""
        if self.state != 'CANCELING':
            return
        goal_id = self.current.get('goal_id') if self.current else None
        self.pending_switch_candidate = None
        self.cancel_requested_s = None
        self.cancel_acknowledged = False
        self.cancel_token += 1
        self.state = 'NAVIGATING'
        self.last_reevaluation_s = self.now_s()
        self.publish_event('GOAL_SWITCH_ABORTED', goal_id=goal_id,
                           reason=reason)

    def cancellation_watchdog(self):
        if self.state != 'CANCELING' or self.cancel_requested_s is None:
            return
        if self.now_s() - self.cancel_requested_s > self.cancel_timeout_s:
            # Nao envia nova meta: nao existe confirmacao de cancelamento.
            self.cancel_failed('CANCEL_TIMEOUT')

    def goal_terminal(self, event, goal_id, **extra):
        c = self.current
        if c is None or c.get('goal_id') != goal_id:
            return
        elapsed = max(0.0, self.now_s() - c['start_time'])
        self.publish_event(event, goal_id=goal_id, x=c['x'], y=c['y'],
                           duration_s=elapsed, **extra)
        # Uma troca intencional nao representa falha da fronteira abandonada.
        switching = (event == 'GOAL_CANCELLED'
                     and self.state == 'CANCELING'
                     and self.pending_switch_candidate is not None)
        if not switching:
            block_time = (self.blacklist_success_s if event == 'GOAL_REACHED'
                          else self.blacklist_failure_s)
            self.blacklist_goal(c, event, block_time)
        switch_candidate = self.pending_switch_candidate if switching else None
        self.invalidate_planning()
        self.goal_handle = None
        self.current = None
        self.pending_candidates = []
        self.reachable_candidates = []
        self.pending_switch_candidate = None
        self.cancel_requested_s = None
        self.cancel_acknowledged = False
        self.cancel_token += 1
        self.state = 'IDLE'
        if switch_candidate is not None:
            robot = self.robot_position()
            if robot is not None and self.nav_client.server_is_ready():
                self.plan_robot = robot
                self.goal_switch_count += 1
                self.last_switch_s = self.now_s()
                self.publish_event('GOAL_SWITCHED',
                                   old_goal_id=goal_id,
                                   new_goal_id=self.goal_counter + 1,
                                   switch_count=self.goal_switch_count,
                                   old_x=c['x'], old_y=c['y'],
                                   new_x=switch_candidate['x'],
                                   new_y=switch_candidate['y'])
                self.commit_goal(switch_candidate,
                                 switch_candidate.get('path_length_m'))
            else:
                self.notice('switch_aborted',
                            'Troca cancelada: TF ou Nav2 indisponivel.')


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
