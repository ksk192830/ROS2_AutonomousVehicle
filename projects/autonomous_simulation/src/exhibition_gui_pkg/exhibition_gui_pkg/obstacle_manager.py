#!/usr/bin/env python3

import json
import math
import os
import random
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import DeleteEntity, SetEntityState, SpawnEntity
from geometry_msgs.msg import Pose
from interfaces_pkg.msg import MotionCommand
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String


from exhibition_gui_pkg.obstacle_placement import OBSTACLES, ObstacleSpawnGuard
from exhibition_gui_pkg.scene_safety import ObstacleSafety, read_scene


EGO_START = (-1.15, -6.80, 0.0, 0.0)


class ExhibitionObstacleManager(Node):
    """Manage exhibition obstacles, one-click demos, and stuck recovery."""

    def __init__(self):
        super().__init__('exhibition_obstacle_manager')
        self.declare_parameter('auto_recovery', True)
        self.declare_parameter('startup_grace_sec', 25.0)
        self.declare_parameter('stuck_timeout_sec', 6.0)
        self.declare_parameter('progress_distance_m', 0.18)
        self.declare_parameter('recovery_cooldown_sec', 12.0)
        self.auto_recovery = bool(self.get_parameter('auto_recovery').value)
        self.startup_grace_sec = float(
            self.get_parameter('startup_grace_sec').value
        )
        self.stuck_timeout_sec = float(
            self.get_parameter('stuck_timeout_sec').value
        )
        self.progress_distance_m = float(
            self.get_parameter('progress_distance_m').value
        )
        self.recovery_cooldown_sec = float(
            self.get_parameter('recovery_cooldown_sec').value
        )

        model_path = os.path.join(
            get_package_share_directory('simulation_pkg'),
            'models',
            'mercedes_obstacle',
            'model.sdf',
        )
        with open(model_path, 'r', encoding='utf-8') as model_file:
            self.model_xml = model_file.read()

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.reset_client = self.create_client(SetEntityState, '/set_entity_state')
        self.enabled = {index: False for index in OBSTACLES}
        self.active_cases = {index: None for index in OBSTACLES}
        self.pending = {}
        self.pending_cases = {}
        self.clearing = False
        self.last_message = '장애물을 놓을 위치를 골라 보세요'

        self.started_at = time.monotonic()
        self.last_progress_at = self.started_at
        self.last_progress_pose = None
        self.latest_pose = None
        self.latest_pose_at = 0.0
        self.spawn_guard = ObstacleSpawnGuard()
        self.scene_safety = ObstacleSafety()
        self.motion_requested = False
        self.recovery_active = False
        self.recovery_count = 0
        self.last_recovery_at = 0.0
        self.demo_phase = None

        state_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.state_pub = self.create_publisher(
            String, '/exhibition/obstacle_state', state_qos
        )
        self.safety_pub = self.create_publisher(String, '/exhibition/safety', state_qos)
        self.command_pub = self.create_publisher(
            String, '/exhibition/command', 10
        )
        self.create_subscription(
            String, '/exhibition/command', self.command_callback, 10
        )
        self.create_subscription(
            ModelStates, '/model_states', self.model_states_callback, sensor_qos
        )
        self.create_subscription(
            MotionCommand, '/motion_command', self.motion_callback, 10
        )
        self.create_timer(0.2, self.publish_state)
        self.create_timer(0.05, self.publish_safety)
        self.create_timer(0.25, self._advance_demo)
        self.create_timer(1.0, self._watchdog)
        self.publish_state()

    def command_callback(self, msg: String):
        command = msg.data.strip().lower()
        if command == 'obstacles_clear':
            self.clearing = True
            self.demo_phase = None
            self.last_message = '장애물 차량을 정리하고 있어요'
            self._advance_demo()
            self.publish_state()
            return
        if command == 'demo_start':
            self._start_demo()
            return

        prefix = 'obstacle_toggle_'
        if not command.startswith(prefix):
            return
        try:
            index = int(command[len(prefix):])
        except ValueError:
            return
        if index not in OBSTACLES or index in self.pending or self.demo_phase or self.clearing:
            return
        self._request_obstacle(index, not self.enabled[index])

    def _request_obstacle(self, index, desired, force=False):
        if index in self.pending:
            return False
        if not force and self.enabled[index] == desired:
            return True

        # Recheck immediately before dispatch, including direct shortcuts,
        # scripted demos and clicks that crossed the distance boundary in flight.
        if desired and self.spawn_guard.reasons(
                self.latest_pose, self.latest_pose_at, time.monotonic())[index - 1]:
            self.publish_state()
            return False

        client = self.spawn_client if desired else self.delete_client
        if not client.service_is_ready():
            self.last_message = 'Gazebo 제어 서비스 연결 대기 중'
            self.publish_state()
            return False

        if desired:
            case_name, pose = random.choice(OBSTACLES[index][1])
            self.pending_cases[index] = case_name
            future = client.call_async(self._spawn_request(index, pose))
        else:
            request = DeleteEntity.Request()
            request.name = OBSTACLES[index][0]
            future = client.call_async(request)

        self.pending[index] = bool(desired)
        action = '생성' if desired else '제거'
        self.last_message = f'{index}번 장애물 {action} 중'
        future.add_done_callback(
            lambda result, i=index, target=desired: self._finish_obstacle(
                i, target, result
            )
        )
        self.publish_state()
        return True

    def _spawn_request(self, index, values):
        entity_name = OBSTACLES[index][0]
        x, y, z, yaw = values
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.z = math.sin(yaw * 0.5)
        pose.orientation.w = math.cos(yaw * 0.5)

        request = SpawnEntity.Request()
        request.name = entity_name
        request.xml = self.model_xml
        request.robot_namespace = ''
        request.initial_pose = pose
        request.reference_frame = 'world'
        return request

    def _finish_obstacle(self, index, desired, future):
        self.pending.pop(index, None)
        selected_case = self.pending_cases.pop(index, None)
        action = '생성' if desired else '제거'
        try:
            response = future.result()
            success = bool(response.success)
            detail = str(getattr(response, 'status_message', ''))
        except Exception as exc:
            success = False
            detail = str(exc)

        normalized = detail.lower()
        already_matches = (
            desired and ('already exist' in normalized or 'already spawn' in normalized)
        ) or (
            not desired
            and ('does not exist' in normalized or 'not exist' in normalized)
        )
        if success or already_matches:
            self.enabled[index] = bool(desired)
            self.active_cases[index] = selected_case if desired else None
            self.last_message = (f'{index}번 위치에 장애물 차량을 놓았어요' if desired
                                 else f'{index}번 장애물 차량을 치웠어요')
        else:
            self.last_message = f'{index}번 장애물 {action} 실패'
            self.get_logger().warning(
                f'Obstacle {index} {action} failed: {detail}'
            )
        self.publish_state()

    def _start_demo(self):
        if self.demo_phase or self.recovery_active or self.pending:
            self.last_message = '이전 명령 처리 후 다시 눌러 주세요'
            self.publish_state()
            return
        if not (
            self.spawn_client.service_is_ready()
            and self.delete_client.service_is_ready()
            and self.reset_client.service_is_ready()
        ):
            self.last_message = '시뮬레이터 준비 중 · 잠시 후 다시 눌러 주세요'
            self.publish_state()
            return

        self.demo_phase = 'clearing'
        self.last_message = '전시 데모 준비 중 · 기존 장애물 정리'
        for index in OBSTACLES:
            self._request_obstacle(index, False, force=True)
        self.publish_state()

    def _advance_demo(self):
        if self.clearing:
            # A spawn may still be completing when the visitor leaves DEMO.
            # Wait for it, then remove the resulting entity before finishing.
            for index in OBSTACLES:
                if self.enabled[index] and index not in self.pending:
                    self._request_obstacle(index, False)
            if not self.pending and not any(self.enabled.values()):
                self.clearing = False
                self.last_message = '장애물 차량을 모두 치웠어요'
                self.publish_state()
            return
        if not self.demo_phase or self.pending:
            return
        if self.demo_phase == 'clearing':
            if any(self.enabled.values()):
                self.demo_phase = None
                self.last_message = '데모 준비 실패 · 장애물 정리 상태를 확인해 주세요'
                self.publish_state()
                return
            self.demo_phase = 'spawning'
            self.last_message = '전시 데모 준비 중 · 장애물 3개 배치'
            for index in OBSTACLES:
                self._request_obstacle(index, True, force=True)
            self.publish_state()
        elif self.demo_phase == 'spawning':
            if not all(self.enabled.values()):
                self.demo_phase = None
                self.last_message = '데모 준비 실패 · 장애물 생성 상태를 확인해 주세요'
                self.publish_state()
                return
            self.demo_phase = 'resetting'
            self._request_reset('전시 데모 시작', demo=True)

    def motion_callback(self, msg: MotionCommand):
        self.motion_requested = (
            abs(float(msg.left_speed)) > 1.0
            or abs(float(msg.right_speed)) > 1.0
        )

    def model_states_callback(self, msg: ModelStates):
        self.scene_safety.update(*read_scene(msg), time.monotonic())
        try:
            index = msg.name.index('ego_vehicle')
        except ValueError:
            self.latest_pose = None
            self.latest_pose_at = 0.0
            return
        pose = msg.pose[index].position
        point = (float(pose.x), float(pose.y))
        self.latest_pose = point
        now = time.monotonic()
        self.latest_pose_at = now
        if self.last_progress_pose is None:
            self.last_progress_pose = point
            self.last_progress_at = now
            return
        distance = math.hypot(
            point[0] - self.last_progress_pose[0],
            point[1] - self.last_progress_pose[1],
        )
        if distance >= self.progress_distance_m:
            self.last_progress_pose = point
            self.last_progress_at = now

    def _watchdog(self):
        now = time.monotonic()
        if (
            not self.auto_recovery
            or self.recovery_active
            or self.demo_phase
            or self.latest_pose is None
            or now - self.started_at < self.startup_grace_sec
            or now - self.last_recovery_at < self.recovery_cooldown_sec
        ):
            return
        if not self.motion_requested:
            self.last_progress_pose = self.latest_pose
            self.last_progress_at = now
            return
        if now - self.last_progress_at >= self.stuck_timeout_sec:
            self._request_reset('차량 정지 감지', demo=False)

    def _request_reset(self, reason, demo=False):
        if self.recovery_active:
            return
        if not self.reset_client.service_is_ready():
            self.last_message = '자동 복구 서비스 연결 대기 중'
            if demo:
                self.demo_phase = None
            self.publish_state()
            return

        x, y, z, yaw = EGO_START
        state = EntityState()
        state.name = 'ego_vehicle'
        state.reference_frame = 'world'
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = float(z)
        state.pose.orientation.z = math.sin(yaw * 0.5)
        state.pose.orientation.w = math.cos(yaw * 0.5)
        request = SetEntityState.Request()
        request.state = state

        self.recovery_active = True
        self.last_message = f'{reason} · 시작 위치로 복구 중'
        future = self.reset_client.call_async(request)
        future.add_done_callback(
            lambda result, is_demo=demo: self._finish_reset(result, is_demo)
        )
        self.publish_state()

    def _finish_reset(self, future, demo):
        try:
            response = future.result()
            success = bool(response.success)
            # SetEntityState has only a success field in ROS 2 Humble, unlike
            # SpawnEntity/DeleteEntity which also return status_message.
            detail = str(getattr(response, 'status_message', ''))
        except Exception as exc:
            success = False
            detail = str(exc)

        self.recovery_active = False
        self.last_recovery_at = time.monotonic()
        self.last_progress_at = self.last_recovery_at
        self.last_progress_pose = EGO_START[:2]
        if success:
            self.command_pub.publish(String(data='demo_reset'))
            if demo:
                cases = ', '.join(
                    f'{index}번 {self.active_cases[index]}' for index in (2, 3)
                )
                self.last_message = f'전시 데모 시작 · 장애물 3개 ON · {cases}'
            else:
                self.recovery_count += 1
                self.last_message = (
                    f'자동 복구 완료 · 처음부터 재개 · 누적 {self.recovery_count}회'
                )
        else:
            self.last_message = '차량 복구 실패 · 운영자 확인 필요'
            self.get_logger().warning(f'Ego reset failed: {detail}')
        self.demo_phase = None
        self.publish_state()

    def publish_state(self):
        payload = {
            'obstacles': [self.enabled[index] for index in OBSTACLES],
            'pending': [index in self.pending for index in OBSTACLES],
            'spawn_blocked_reasons': self.spawn_guard.reasons(
                self.latest_pose, self.latest_pose_at, time.monotonic()),
            'ready': self.spawn_client.service_is_ready() and self.delete_client.service_is_ready(),
            'clearing': self.clearing,
            'positions': [list(dict(OBSTACLES[index][1]).get(self.active_cases[index],
                          OBSTACLES[index][1][0][1]))[:2] for index in OBSTACLES],
            'cases': [self.active_cases[index] for index in OBSTACLES],
            'auto_recovery': self.auto_recovery,
            'recovery_active': self.recovery_active,
            'recovery_count': self.recovery_count,
            'demo_preparing': bool(self.demo_phase),
            'message': self.last_message,
        }
        self.state_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )


    def publish_safety(self):
        self.safety_pub.publish(String(data=json.dumps(self.scene_safety.status(time.monotonic()))))


def main(args=None):
    rclpy.init(args=args)
    node = ExhibitionObstacleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
