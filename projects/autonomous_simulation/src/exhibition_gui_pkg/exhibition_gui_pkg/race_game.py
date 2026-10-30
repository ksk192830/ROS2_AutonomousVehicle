#!/usr/bin/env python3
"""Visitor-facing human-versus-AI ghost racing game."""

import json
import math
import os
import sys
import time
import xml.etree.ElementTree as ET
from typing import Dict, Optional, Tuple

SYSTEM_QT_PLATFORM_PATH = '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms'

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from gazebo_msgs.msg import ContactsState, EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from interfaces_pkg.msg import LaneInfo, MotionCommand
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image, Joy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Int32, String

from exhibition_gui_pkg.track_overlay import TrackBoundaryOverlay
from exhibition_gui_pkg.experience import LESSONS, ExperienceSession, LapProgress, result_text, practice_complete
from exhibition_gui_pkg.display_camera import LatestFrameConverter, display_image
from exhibition_gui_pkg.teaching_frames import TeachingFrames
from exhibition_gui_pkg.obstacle_placement import ObstacleSpawnGuard
from exhibition_gui_pkg.scene_safety import ContactConfirmation, ObstacleSafety, read_scene
from exhibition_gui_pkg.controller_navigation import ControllerNavigation
from exhibition_gui_pkg.duel import DUEL_DIFFICULTIES, RACE_OBSTACLES

# pip OpenCV can replace Qt's plugin path while importing cv2.
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = SYSTEM_QT_PLATFORM_PATH

from PyQt5.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QImage, QSurfaceFormat
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from exhibition_gui_pkg.dashboard import (
    BG,
    BLUE,
    CORAL,
    GREEN,
    MUTED,
    PANEL,
    RED,
    TEAL,
    TEXT,
    ImageCanvas,
    StatusPill,
    panel_style,
)


PLAYER_ENTITY = 'ego_vehicle'
GHOST_ENTITY = 'ai_ghost'
START_POSE = (-1.15, -6.80, 0.0)
GHOST_PARK_POSE = (30.0, 30.0, 0.0)
MAX_SPEED_MPS = 1.2345


class DisplayFrameNotifier(QObject):
    ready = pyqtSignal()
    action = pyqtSignal(str)
    quit = pyqtSignal()

DIFFICULTIES = {
    'beginner': ('입문', 0.70, GREEN),
    'intermediate': ('도전', 0.85, BLUE),
    'expert': ('고급', 1.0,  CORAL),
}

GHOST_SDF = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='ai_ghost'>
    <static>true</static>
    <link name='ghost_body'>
      <gravity>false</gravity>
      <visual name='ghost_car_visual'>
        <pose>0.505 0 0.12 1.5708 0 1.5708</pose>
        <geometry>
          <mesh>
            <uri>model://mercedes_split_tex/meshes/body.obj</uri>
            <scale>30 24 20.88</scale>
          </mesh>
        </geometry>
        <transparency>0.38</transparency>
        <cast_shadows>false</cast_shadows>
        <material>
          <ambient>0.05 0.85 1.0 0.42</ambient>
          <diffuse>0.05 0.85 1.0 0.42</diffuse>
          <emissive>0.0 0.28 0.36 0.30</emissive>
        </material>
      </visual>
      <sensor name='cam0' type='camera'>
        <pose>0.145 0 0.675 0 0.21 0</pose>
        <always_on>1</always_on>
        <update_rate>15</update_rate>
        <visualize>0</visualize>
        <camera name='cam0'>
          <horizontal_fov>1.0856</horizontal_fov>
          <image>
            <width>640</width>
            <height>360</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.03</near>
            <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0</mean>
            <stddev>0</stddev>
          </noise>
        </camera>
        <plugin name='ghost_front_camera_driver' filename='libgazebo_ros_camera.so'>
          <alwaysOn>1</alwaysOn>
          <updateRate>15</updateRate>
          <cameraName>ai_ghost/cam0</cameraName>
          <imageTopicName>/ai_ghost/cam0/image_raw</imageTopicName>
          <cameraInfoTopicName>/ai_ghost/cam0/camera_info</cameraInfoTopicName>
          <frameName>ai_ghost_front_camera</frameName>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>
"""


def yaw_to_quaternion(yaw: float):
    state = EntityState()
    state.pose.orientation.z = math.sin(yaw * 0.5)
    state.pose.orientation.w = math.cos(yaw * 0.5)
    return state.pose.orientation


class RaceGameNode(Node):
    """Own game state, controller input, ghost playback, and boundary rules."""

    def __init__(self):
        super().__init__('exhibition_race_game')
        self.declare_parameter('fullscreen', True)
        self.declare_parameter('display_fps', 15.0)
        self.declare_parameter('screen_index', 0)
        self.declare_parameter('guided_experience', False)
        self.declare_parameter('analysis_screen_enabled', False)
        self.declare_parameter('trigger_rest_value', 1.0)
        self.declare_parameter('duel_enabled', False)
        self.fullscreen = bool(self.get_parameter('fullscreen').value)
        self.display_fps = max(1.0, float(self.get_parameter('display_fps').value))
        self.screen_index = max(0, int(self.get_parameter('screen_index').value))
        self.guided = bool(self.get_parameter('guided_experience').value)
        self.analysis_screen_enabled = bool(self.get_parameter('analysis_screen_enabled').value)
        self.session = ExperienceSession()
        self.input_mode = 'keyboard'
        self.input_armed = False
        self.practice_tasks = set()
        self.practice_feedback = ''
        self.attempt_started = False
        self.paused_manual = False
        self.lesson_images = {}
        self.lesson_received_at = {}
        self.offtracks = 0
        self.last_states_at = 0.0
        self.last_clock_at = 0.0
        self.sim_time = 0.0
        self.race_sim_started = 0.0
        self.race_sim_finished = None
        self.autonomy_command = MotionCommand()
        self.last_autonomy_at = 0.0
        self.last_valid_lane_at = 0.0
        self.current_lane_valid = False
        self.lane_geometry = {}
        self.lane_geometry_at = 0.0
        self.target_lane = 2
        self.lane_change_active = False
        self.lane_request_at = 0.0
        self.obstacle_state = {}
        self.obstacle_state_at = 0.0
        self.obstacle_requests = {}
        self.demo_feedback = ''
        self.paused_at = 0.0
        self.paused_state = ''
        self.pause_reason = ''
        self.reset_generation = 0
        self.reset_started_at = 0.0
        self.reset_mode = 'COUNTDOWN'
        self.reset_future = None
        self.ghost_spawn_started_at = 0.0
        self.ghost_pose_started_at = 0.0
        self.ghost_spawn_future = None
        self.ghost_pose_future = None
        self.outside_since = None

        self.bridge = CvBridge()
        self.track = TrackBoundaryOverlay()
        self.image_sequence = 0
        self.image: Optional[Tuple[int, QImage]] = None
        self.last_image_at = 0.0
        self.camera_presented_frames = 0
        self.last_camera_submitted_at = 0.0
        self.camera_received_frames = 0
        self.display_frame_notifier = DisplayFrameNotifier()
        self.controller_navigation = ControllerNavigation()
        self.display_converter = LatestFrameConverter(self._convert_display_image,
                                                      self.display_frame_notifier.ready.emit)
        self.teaching_frames = TeachingFrames()
        self.obstacle_spawn_guard = ObstacleSpawnGuard()
        self.scene_safety = ObstacleSafety()
        self.contact_confirmation = ContactConfirmation()
        self.collision_recoveries = 0
        self.collision_hold_until = 0.0
        self.last_collision_recovery_at = 0.0
        self.hazard_since = None
        self.reset_kind = ''

        self.path_points, self.path_s, self.path_total = self._load_reference_lap()
        self.lap = LapProgress(self.path_total)
        self.difficulty = 'beginner'
        self.state = 'READY'
        self.message = '시작하기를 누르면 자율주행의 원리를 함께 살펴봐요' if self.guided else '난이도를 고르고 시작 버튼을 누르세요'
        self.winner = ''
        self.countdown_started_at = 0.0
        self.race_started_at = 0.0
        self.race_finished_at = 0.0
        self.player_progress_s = 0.0
        self.ai_progress_s = 0.0
        self.player_route_s = 0.0
        self.player_last_route_s = 0.0
        self.player_pose: Optional[Tuple[float, float, float]] = None
        self.player_speed_mps = 0.0
        self.ghost_pose = GHOST_PARK_POSE if self.guided else START_POSE
        self.boundary_violation_frames = 0

        self.last_joy_at = 0.0
        self.joy_axes = []
        self.joy_buttons = []
        self.previous_buttons = []
        rest = float(self.get_parameter('trigger_rest_value').value)
        self.trigger_rest: Dict[int, float] = {2: rest, 5: rest}
        self.keyboard = {
            'left': False,
            'right': False,
            'throttle': False,
            'brake': False,
            'view_left': False,
            'view_right': False,
            'view_up': False,
            'view_down': False,
        }
        self.last_command = MotionCommand()

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.create_subscription(Image, '/cam0/image_raw', self._on_image, sensor_qos)
        self.create_subscription(Image, '/cam0/perception/image_raw',
                                 lambda msg: self._on_lesson_image(msg, 'source'), sensor_qos)
        self.create_subscription(Image, '/cam0/lane_viz', self._on_lesson_image, sensor_qos)
        for key, topic in (('segmentation', '/cam0/seg_vis'), ('bev', '/cam0/bev_viz')):
            self.create_subscription(Image, topic, lambda msg, key=key: self._on_lesson_image(msg, key), sensor_qos)
        self.create_subscription(ModelStates, '/model_states', self._on_model_states, sensor_qos)
        self.create_subscription(ContactsState, '/exhibition/contacts', self._on_contacts,
                                 QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Joy, '/joy', self._on_joy, sensor_qos)
        self.create_subscription(Clock, '/clock', self._on_clock, sensor_qos)
        self.create_subscription(MotionCommand, '/autonomy/motion_command', self._on_autonomy, 1)
        self.create_subscription(LaneInfo, '/cam0/lane_info', self._on_lane, 1)
        self.create_subscription(String, '/cam0/lane_geometry', self._on_geometry, 1)
        self.motion_pub = self.create_publisher(MotionCommand, '/motion_command', 10)
        self.exhibition_command_pub = self.create_publisher(String, '/exhibition/command', 10)
        self.ghost_motion_pub = self.create_publisher(
            MotionCommand, '/ai_ghost/motion_command', 10
        )
        state_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.status_pub = self.create_publisher(String, '/race/status', state_qos)
        self.create_subscription(String, '/exhibition/obstacle_state', self._on_obstacles, state_qos)
        self.create_subscription(Int32, '/target_lane', self._on_target_lane, state_qos)
        self.create_subscription(Bool, '/lane_change_active', self._on_lane_change, state_qos)
        self.create_subscription(String, '/race/command', self._on_race_command, 10)
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.state_client = self.create_client(SetEntityState, '/set_entity_state')

        self.ghost_spawned = False
        self.ghost_spawn_pending = False
        self.ghost_pose_pending = False
        self.reset_pending = False
        self.create_timer(0.5, self._ensure_ghost)
        self.create_timer(0.02, self._control_tick)
        self.create_timer(0.05, self._ghost_tick)
        self.create_timer(0.2, self._publish_status)
        from exhibition_gui_pkg.live_duel import LiveDuel
        self.duel = LiveDuel(self) if self.get_parameter('duel_enabled').value else None
        from exhibition_gui_pkg.venue_camera import VenueCamera
        self.venue = VenueCamera(self) if self.guided else None
        from exhibition_gui_pkg.camera_budget import CameraBudget
        self.camera_budget = CameraBudget(self) if self.guided else None

    def _load_reference_lap(self):
        path = os.path.join(
            get_package_share_directory('exhibition_gui_pkg'),
            'assets',
            'ai_reference_lap.json',
        )
        with open(path, 'r', encoding='utf-8') as reference_file:
            payload = json.load(reference_file)
        points = np.array(
            [[sample['x'], sample['y']] for sample in payload['samples']],
            dtype=np.float64,
        )
        if len(points) < 8:
            raise RuntimeError('AI reference lap does not contain enough points.')
        if float(np.linalg.norm(points[-1] - points[0])) > 0.01:
            points = np.vstack((points, points[0]))
        segments = np.linalg.norm(np.diff(points, axis=0), axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(segments)))
        return points, cumulative, float(cumulative[-1])

    @property
    def controller_connected(self) -> bool:
        return time.monotonic() - self.last_joy_at < 1.5

    @property
    def difficulty_label(self) -> str:
        return DIFFICULTIES[self.difficulty][0]

    @property
    def difficulty_ratio(self) -> float:
        return DIFFICULTIES[self.difficulty][1]

    @property
    def elapsed(self) -> float:
        if self.race_started_at <= 0.0:
            return 0.0
        end = self.sim_time if self.race_sim_finished is None else self.race_sim_finished
        return max(0.0, end - self.race_sim_started)

    @property
    def countdown_text(self) -> str:
        if self.state != 'COUNTDOWN':
            return ''
        remaining = 3.0 - (time.monotonic() - self.countdown_started_at)
        return str(max(1, int(math.ceil(remaining))))

    @property
    def player_percent(self) -> float:
        return min(100.0, 100.0 * self.player_progress_s / self.path_total)

    @property
    def ai_percent(self) -> float:
        return min(100.0, 100.0 * self.ai_progress_s / self.path_total)

    def set_difficulty(self, difficulty: str) -> None:
        if self.guided:
            return
        if difficulty not in DIFFICULTIES or self.state in ('COUNTDOWN', 'RUNNING', 'RESETTING'):
            return
        self.difficulty = difficulty
        label, ratio, _ = DIFFICULTIES[difficulty]
        self.message = f'{label} 모드 · AI 최고 속도 {int(ratio * 100)}%'

    def set_keyboard(self, action: str, pressed: bool) -> None:
        if action in self.keyboard:
            self.keyboard[action] = bool(pressed)

    def _on_race_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command == 'start':
            self.request_start()
        elif command == 'cancel':
            self.cancel_session()
        elif command == 'secondary':
            self.request_secondary()
        elif command == 'practice':
            self.request_practice()
        elif command in ('venue_car', 'venue_home') and getattr(self, 'venue', None):
            self.venue.focus(command == 'venue_car')
        elif command in ('answer_stop', 'answer_go') and self.session.phase == 'RESULTS':
            self.session.answer = command
        elif command in ('lane_toggle', 'obstacle_toggle_1', 'obstacle_toggle_2', 'obstacle_toggle_3'):
            self.request_demo_command(command)
        elif command.startswith('key:') and self.input_mode == 'keyboard':
            parts = command.split(':')
            if len(parts) == 3 and parts[2] in ('down', 'up'):
                self.set_keyboard(parts[1], parts[2] == 'down')
        elif command == 'release_keys':
            self.clear_keyboard()
            self.input_armed = False
        elif command == 'duel_recover' and getattr(self, 'duel', None):
            if self.duel.snapshot()['recovery_available']:
                self._recover_collision('다시 달릴 수 있는 위치로 돌아가요.')
        elif command == 'duel_finish' and getattr(self, 'duel', None) and self.state == 'RUNNING':
            self._finish('PARTIAL', '여기까지 달린 기록을 비교해 보세요')
        elif command.startswith('difficulty:'):
            self.select_duel_difficulty(command.partition(':')[2])
        elif command in DIFFICULTIES:
            self.set_difficulty(command)

    def status_snapshot(self):
        self.collect_display_frame()
        payload = {
            'state': self.state,
            'difficulty': self.difficulty,
            'ai_speed_ratio': self.difficulty_ratio,
            'elapsed_sec': round(self.elapsed, 3),
            'player_progress_percent': round(self.player_percent, 2),
            'ai_progress_percent': round(self.ai_percent, 2),
            'player_speed_mps': round(self.player_speed_mps, 3),
            'controller_connected': self.controller_connected,
            'winner': self.winner,
            'message': self.message,
            'player_pose': list(self.player_pose) if self.player_pose else None,
            'pose_fresh': time.monotonic() - self.last_states_at < 1.0,
            'ghost_pose': list(self.ghost_pose),
            'session': self.session.snapshot(time.monotonic()) if self.guided else None,
            'practice_tasks': sorted(self.practice_tasks),
            'practice_feedback': self.practice_feedback,
            'paused_manual': self.paused_manual,
            'autonomy_steering': int(self.autonomy_command.steering),
            'autonomy_fresh': time.monotonic() - self.last_autonomy_at < 0.5,
            'lane_valid': self.current_lane_valid and time.monotonic() - self.last_valid_lane_at < 1.0,
            'lane_geometry': self.lane_geometry if time.monotonic() - self.lane_geometry_at < 1.0 else {},
            'motion_steering': int(self.last_command.steering),
            'lesson_fresh': time.monotonic() - self.lesson_received_at.get('fitting', 0) < 1.5,
            'countdown': self.countdown_text,
            'offtracks': self.offtracks,
            'input_mode': self.input_mode,
            'input_armed': self.input_armed,
            'manual_throttle': self._trigger(2) if self.input_mode == 'controller' else float(self.keyboard['throttle']),
            'manual_brake': self._trigger(5) if self.input_mode == 'controller' else float(self.keyboard['brake']),
            'camera_fresh': time.monotonic() - self.last_image_at < 1.5,
            'camera_presented_frames': self.camera_presented_frames,
            'camera_received_frames': self.camera_received_frames,
            'camera_converted_frames': self.image_sequence,
            'venue_presented_frames': getattr(self, 'venue_presented_frames', 0),
            'camera_rates': self.camera_budget.rates if getattr(self, 'camera_budget', None) else {},
            'collision_recoveries': self.collision_recoveries,
            'reset_kind': self.reset_kind,
            'demo_controls': self._demo_control_status(),
        }
        if getattr(self, 'duel', None):
            payload['duel'] = self.duel.snapshot()
        if getattr(self, 'venue', None):
            payload['venue'] = self.venue.snapshot()
        return payload

    def _on_target_lane(self, msg):
        if msg.data in (1, 2):
            self.target_lane = msg.data

    def _on_lane_change(self, msg):
        if self.lane_change_active and not msg.data:
            self.demo_feedback = '다른 차선으로 이동을 마쳤어요'
        self.lane_change_active = bool(msg.data)

    def _on_obstacles(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict) or any(
                not isinstance(value.get(key), list) or len(value[key]) != 3
                for key in ('obstacles', 'pending')
            ):
                return
        except (ValueError, TypeError):
            return
        self.obstacle_state = value
        self.obstacle_state_at = time.monotonic()
        for index, (desired, _) in list(self.obstacle_requests.items()):
            blocked = value.get('spawn_blocked_reasons', ['', '', ''])
            rejected = desired and isinstance(blocked, list) and len(blocked) == 3 and bool(blocked[index])
            if value['pending'][index] or value['obstacles'][index] == desired or rejected:
                self.obstacle_requests.pop(index, None)

    def _demo_control_status(self):
        now = time.monotonic()
        for index, (_, started) in list(self.obstacle_requests.items()):
            if now - started > 5.0:
                self.obstacle_requests.pop(index, None)
                self.demo_feedback = '장애물 응답이 늦어요. 잠시 후 다시 눌러 주세요.'
        state = self.obstacle_state
        live = now - self.obstacle_state_at < 2.5
        active = self.state == 'DEMO'
        safety = self.scene_safety.status(now)
        destination = 1 if self.target_lane == 2 else 2
        destination_blocked = safety[f'manual_lane{destination}_blocked']
        changing = self.lane_change_active or now - self.lane_request_at < 1.0
        local_blocks = self.obstacle_spawn_guard.reasons(self.player_pose, self.last_states_at, now)
        reported_blocks = state.get('spawn_blocked_reasons', ['', '', ''])
        if not isinstance(reported_blocks, list) or len(reported_blocks) != 3:
            reported_blocks = ['', '', '']
        return {
            'ready': live and bool(state.get('ready')),
            'obstacles': state.get('obstacles', [False] * 3),
            'pending': [bool(state.get('pending', [False] * 3)[i]) or i in self.obstacle_requests for i in range(3)],
            'positions': state.get('positions', []),
            'spawn_blocked_reasons': [local_blocks[i] or reported_blocks[i] for i in range(3)],
            'clearing': bool(state.get('clearing')),
            'target_lane': self.target_lane,
            'changing_lane': changing,
            'lane_blocked_reason': safety['reason'] or ('path' if destination_blocked else ''),
            'safety_stop': safety['stop'] and safety['reason'] != 'position',
            'lane_enabled': active and not changing and not destination_blocked and not safety['stop'] and self.current_lane_valid and now - self.last_autonomy_at < .5,
            'message': self.demo_feedback or state.get('message', ''),
        }

    def request_demo_command(self, command):
        # Only AI observation owns these controls. Paused/manual/lesson states
        # must never accept an invisible shortcut or a delayed button click.
        if self.state != 'DEMO':
            return
        controls = self._demo_control_status()
        if command == 'lane_toggle':
            if not controls['lane_enabled']:
                return
            self.lane_request_at = time.monotonic()
            self.demo_feedback = '다른 차선의 가운데로 이동하고 있어요'
        else:
            index = int(command[-1]) - 1
            if not controls['ready'] or controls['clearing'] or controls['pending'][index]:
                return
            if not controls['obstacles'][index] and controls['spawn_blocked_reasons'][index]:
                return
            self.obstacle_requests[index] = (not controls['obstacles'][index], time.monotonic())
            self.demo_feedback = ''
        self.exhibition_command_pub.publish(String(data=command))

    def _clear_demo_obstacles(self):
        self.exhibition_command_pub.publish(String(data='obstacles_clear'))
        self.obstacle_requests.clear()
        self.demo_feedback = ''

    def _publish_status(self) -> None:
        self.status_pub.publish(String(data=json.dumps(self.status_snapshot(), ensure_ascii=False)))

    def _on_clock(self, msg):
        value = msg.clock.sec + msg.clock.nanosec / 1e9
        if value != self.sim_time:
            self.last_clock_at = time.monotonic()
        self.sim_time = value

    def _on_autonomy(self, msg):
        self.autonomy_command = msg
        self.last_autonomy_at = time.monotonic()

    def _on_lane(self, msg):
        # The motion node stops on each invalid observation. Escalate to a
        # visitor pause only after a full second without any valid lane.
        self.current_lane_valid = msg.lane_num in (1, 2)
        if self.current_lane_valid:
            self.last_valid_lane_at = time.monotonic()

    def _on_geometry(self, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                self.lane_geometry = value
                self.lane_geometry_at = time.monotonic()
        except (ValueError, TypeError):
            return

    def clear_keyboard(self):
        for key in self.keyboard:
            self.keyboard[key] = False

    def _input_neutral(self):
        if self.input_mode == 'controller':
            if not self.controller_connected or len(self.joy_axes) < 6:
                return False
            return abs(self.joy_axes[0]) < 0.12 and self._trigger(2) < 0.05 and self._trigger(5) < 0.05
        return not any(self.keyboard.values())

    def _health_problem(self):
        now = time.monotonic()
        if now - self.last_image_at > 1.5:
            return '카메라 연결을 확인해 주세요'
        if now - self.last_states_at > 1.0 or now - self.last_clock_at > 1.0:
            return '시뮬레이터가 잠시 멈췄어요'
        if self.input_mode == 'controller' and not self.controller_connected:
            return '컨트롤러 연결이 끊겼어요'
        mode = self.paused_state if self.state == 'PAUSED' else self.state
        if mode in ('INTRO', 'DEMO') and now - self.last_valid_lane_at > 1.0:
            safety = self.scene_safety.status(now)
            # A close car can cover the road while the safety brake is holding.
            # Keep the obstacle-removal controls available; camera/clock losses
            # above still fault immediately and no motion is allowed here.
            if not safety['stop'] or safety['reason'] == 'position':
                return '차선을 확인하지 못해 AI가 멈췄어요'
        return ''

    def _ready_to_start(self):
        self.input_mode = 'controller' if self.controller_connected else 'keyboard'
        problem = self._health_problem()
        if not self.track.available:
            self.message = '도로 경계 자료를 불러오지 못했습니다 · 운영자에게 알려 주세요'
            return False
        vehicle_ready = self.duel.available if getattr(self, 'duel', None) else self.ghost_spawned
        if problem or not vehicle_ready or not self.state_client.service_is_ready():
            self.message = problem or '차량 준비 중 · 잠시 후 시작해 주세요'
            return False
        if not self.get_publishers_info_by_topic('/rear_wheel_controller/commands'):
            self.message = '차량 제어 준비 중 · 잠시만 기다려 주세요'
            return False
        if not self._input_neutral():
            self.message = self.neutral_instruction()
            return False
        return True

    def neutral_instruction(self):
        return ('스틱과 RT·LT를 모두 놓아 주세요' if self.input_mode == 'controller'
                else '누르고 있는 방향키를 모두 놓아 주세요')

    def _stage_instruction(self):
        return {
            'EXPLORE': '스틱으로 대회장을 둘러보고 차량의 카메라 화면으로 이동해 보세요.',
            'INTRO': '화면의 버튼을 눌러 도로 보기 → 차선 찾기 → 방향 정하기를 살펴보세요',
            'DEMO': '지금은 AI가 운전해요. 차선과 핸들 방향이 함께 변하는지 살펴보세요.',
            'PRACTICE': '이제 직접 조작해 보세요. 익힐 때까지 충분히 연습할 수 있어요.',
            'DIFFICULTY': 'AI와 함께 달리기 전에 난이도를 골라 보세요.',
            'COUNTDOWN': self.neutral_instruction(),
            'RUNNING': '목표는 한 바퀴 완주! 도로를 벗어나면 출발한 방향으로 돌려드려요.',
            'RESULTS': '결과를 읽고 다시 도전하거나 체험을 마칠 수 있어요.',
            'READY': '시작하기를 누르면 자율주행의 원리를 함께 살펴봐요',
        }.get(self.state, '차량을 준비하고 있어요. 잠시만 기다려 주세요.')

    def _enter_stage(self):
        self._publish_stop()
        self._clear_demo_obstacles()
        self.reset_generation += 1
        self.reset_pending = False
        if self.reset_future is not None and not self.reset_future.done():
            self.reset_future.cancel()
        self.clear_keyboard()
        self.input_armed = False
        phase = self.session.phase
        if getattr(self, 'venue', None):
            self.venue.activate(phase == 'EXPLORE')
        if getattr(self, 'duel', None) and phase != 'RESULTS':
            if phase == 'CHALLENGE':
                self.duel.prepare()
            else:
                self.duel.park()
        if phase in ('EXPLORE', 'INTRO', 'DEMO', 'PRACTICE', 'DIFFICULTY'):
            self._reset_player(phase)
        elif phase == 'CHALLENGE':
            self._reset_player('COUNTDOWN')
        elif phase == 'RESULTS':
            self.state = 'RESULTS'
            self.message = self.session.error or self._stage_instruction()
        else:
            self.state = 'READY'
            self.message = self._stage_instruction()
            self.winner = ''
            self.player_progress_s = self.ai_progress_s = 0.0
            self.race_started_at = self.race_finished_at = 0.0
            self.last_autonomy_at = 0.0

    def _record_attempt(self, valid=True):
        if not self.guided:
            return
        self.session.record(
            valid=valid, completed=self.winner == 'PLAYER',
            elapsed_sec=round(self.elapsed, 2),
            progress_percent=round(self.player_percent, 1), offtracks=self.offtracks,
            difficulty=self.difficulty, started=self.attempt_started,
        )

    def _session_tick(self, now):
        # Timeouts concern failed devices only; reading and driving never expire.
        if self.state == 'PAUSED' and not self.paused_manual and now - self.paused_at >= 10:
            self._fault('연결이 복구되지 않았어요. 다시 준비하거나 체험을 마칠 수 있어요.')

    def pause_session(self, reason='연결 상태를 확인해 주세요', manual=False):
        if self.state not in ('INTRO', 'DEMO', 'PRACTICE', 'RUNNING', 'COUNTDOWN'):
            return
        self.paused_state = self.state
        self.paused_sim_time = self.sim_time
        self.paused_at = time.monotonic()
        self.paused_manual = manual
        self.pause_reason = reason
        self.state = 'PAUSED'
        self.message = reason
        self.clear_keyboard()
        self.input_armed = False
        self._publish_stop()

    def request_secondary(self):
        if not self.guided:
            self.cancel_session()
        elif self.state == 'INTRO' and self.session.lesson > 0:
            self.session.lesson -= 1
        elif self.state == 'INTRO' and self.session.lesson == 0:
            self.session.go('EXPLORE', time.monotonic())
            self._enter_stage()
        elif self.state == 'DIFFICULTY':
            self.session.go('PRACTICE', time.monotonic())
            self._enter_stage()
        elif self.state in ('DEMO', 'PRACTICE', 'RUNNING', 'COUNTDOWN'):
            return
        elif self.state == 'PAUSED' and self.session.phase == 'CHALLENGE':
            self.race_sim_finished = self.paused_sim_time
            self.race_finished_at = time.monotonic()
            self._record_attempt(valid=self.paused_manual)
            self.session.go('RESULTS', time.monotonic())
            self._enter_stage()
        else:
            self.cancel_session()

    def request_practice(self):
        if not self.guided or self.state not in ('RESULTS', 'PRACTICE'):
            return
        if self.state == 'PRACTICE':
            self.session.go('DIFFICULTY', time.monotonic())
        else:
            if not self._ready_to_start():
                return
            self.session.practice_step = 0
            self.practice_tasks.clear()
            self.session.go('PRACTICE', time.monotonic())
        self._enter_stage()

    def select_duel_difficulty(self, difficulty):
        if (not self.guided or self.state != 'DIFFICULTY' or difficulty not in DUEL_DIFFICULTIES or
                not getattr(self, 'duel', None) or not self.duel.configure_difficulty(difficulty)):
            return
        self.difficulty = difficulty
        self.session.difficulty = difficulty
        self.session.begin_attempt(time.monotonic())
        self._enter_stage()

    def cancel_session(self):
        if getattr(self, 'venue', None):
            self.venue.activate(False)
        if getattr(self, 'duel', None):
            self.duel.park()
        self._publish_stop()
        self._clear_demo_obstacles()
        self.clear_keyboard()
        self.session.clear(time.monotonic())
        self.practice_tasks.clear()
        self.practice_feedback = ''
        self.paused_manual = False
        self._reset_player('READY')

    def _fault(self, message):
        if getattr(self, 'venue', None):
            self.venue.activate(False)
        if getattr(self, 'duel', None):
            self.duel.fail(message)
            if self.duel.player:
                self.duel.player.valid = False
                self.duel.player.error = message
        self.reset_generation += 1
        self.reset_pending = False
        self._record_attempt(valid=False)
        self._publish_stop()
        self._clear_demo_obstacles()
        self.clear_keyboard()
        self.input_armed = False
        self.message = message
        self.state = 'ERROR'
        if self.guided and self.session.phase != 'IDLE':
            self.session.error = message
            self.session.go('RESULTS', time.monotonic())
            self.state = 'RESULTS'

    def _check_requests(self, now):
        if self.reset_pending and now - self.reset_started_at > 5:
            future = self.reset_future
            self._fault('차량 초기화가 지연되어 체험을 멈췄어요')
            if future is not None:
                future.cancel()
        if self.ghost_spawn_pending and now - self.ghost_spawn_started_at > 5:
            future = self.ghost_spawn_future
            self.ghost_spawn_future = None
            self.ghost_spawn_pending = False
            if future is not None:
                future.cancel()
        if self.ghost_pose_pending and now - self.ghost_pose_started_at > 3:
            future = self.ghost_pose_future
            self.ghost_pose_future = None
            self.ghost_pose_pending = False
            if future is not None:
                future.cancel()
            if self.state == 'RUNNING' and not self.guided:
                self._fault('고스트 차량 연결이 지연되어 기록을 중단했어요')

    def request_start(self) -> None:
        if self.reset_pending:
            return
        if self.state == 'PAUSED':
            problem = self._health_problem()
            if problem or not self._input_neutral():
                self.message = problem or self.neutral_instruction()
                return
            self.race_sim_started += self.sim_time - self.paused_sim_time
            self.state = self.paused_state
            if self.state == 'COUNTDOWN':
                self.countdown_started_at = time.monotonic()
            self.input_armed = True
            self.message = self._stage_instruction()
            return
        if self.guided:
            now = time.monotonic()
            if self.session.phase == 'IDLE':
                if not self._ready_to_start():
                    return
                self.session.start(now)
                self.practice_tasks.clear()
                self._enter_stage()
            elif self.state == 'EXPLORE':
                if getattr(self, 'venue', None) and not self.venue.ready:
                    return
                self.session.go('INTRO', now)
                self._enter_stage()
            elif self.state == 'INTRO':
                if self.session.lesson < len(LESSONS) - 1:
                    self.session.lesson += 1
                else:
                    self.session.go('DEMO', now)
                    self._enter_stage()
            elif self.state == 'DEMO':
                self.practice_tasks.clear()
                self.session.practice_step = 0
                self.session.go('PRACTICE', now)
                self._enter_stage()
            elif self.state == 'PRACTICE':
                if not practice_complete(self.session.practice_step, self.practice_tasks):
                    self.practice_feedback = '먼저 안내된 조작을 해 보세요. 익숙하다면 바로 도전할 수도 있어요.'
                    return
                if self.session.practice_step < 2:
                    self.session.practice_step += 1
                    self.practice_feedback = ''
                else:
                    self.session.go('DIFFICULTY', now)
                    self._enter_stage()
            elif self.state == 'RESULTS':
                if self._ready_to_start():
                    self.session.go('DIFFICULTY', now)
                    self._enter_stage()
            return
        if self.state in ('COUNTDOWN', 'RUNNING') or not self._ready_to_start():
            return
        self._reset_player('COUNTDOWN')

    def _reset_player(self, mode, pose=START_POSE, recovery=False, recovery_reason=''):
        if recovery and getattr(self, 'duel', None):
            self.duel.player_recovery(pose)
        self.contact_confirmation.reset(round(self.sim_time * 10**9))
        self.reset_generation += 1
        generation = self.reset_generation
        self.reset_mode = mode
        self.reset_kind = 'safety' if recovery_reason else 'track' if recovery else ''
        self.state = 'RESETTING'
        self.message = (recovery_reason + ' 안전한 도로 위치로 옮기고 있어요.' if recovery_reason else
                        ('도로 위로 돌아가고 있어요. ' if recovery else '출발 위치를 준비하고 있어요. ') + self.neutral_instruction())
        self.input_armed = False
        self.clear_keyboard()
        self.outside_since = None
        if not recovery:
            self.collision_recoveries = 0
            self.hazard_since = None
            self.collision_hold_until = 0.0
            # Full resets always start in lane 2. Clear the previous demo's
            # avoidance target so perception can acquire that starting lane.
            self.exhibition_command_pub.publish(String(data='demo_reset'))
            self.winner = ''
            self.player_progress_s = self.player_route_s = self.player_last_route_s = 0.0
            self.ai_progress_s = 0.0
            self.lap = LapProgress(self.path_total)
            self.lap.rebase(self._route_at(*pose[:2]))
            self.offtracks = 0
            self.attempt_started = False
            self.race_started_at = self.race_finished_at = 0.0
            self.race_sim_finished = None
            self.ghost_pose = START_POSE if mode == 'COUNTDOWN' and not self.guided else GHOST_PARK_POSE
        else:
            self.lap.rebase(self._route_at(*pose[:2]))
        self._publish_stop()
        if not self.state_client.service_is_ready():
            self._fault('차량 초기화 연결을 확인해 주세요')
            return
        request = SetEntityState.Request()
        request.state = self._entity_state(PLAYER_ENTITY, *pose)
        self.reset_pending = True
        self.reset_started_at = time.monotonic()
        future = self.state_client.call_async(request)
        self.reset_future = future
        future.add_done_callback(lambda done: self._on_player_reset(done, generation))
        self._set_ghost_pose(self.ghost_pose)

    def _on_player_reset(self, future, generation) -> None:
        if generation != self.reset_generation:
            return
        self.reset_pending = False
        try:
            success = bool(future.result().success)
        except Exception as exc:
            self.get_logger().error(f'Player reset failed: {exc}')
            success = False
        if not success:
            self._fault('차량 초기화 실패 · 운영자에게 알려 주세요')
            return
        if self.reset_kind == 'safety':
            # Keep the recovery notice visible while fresh camera/pose/steering
            # results arrive. _control_tick completes this brief settling phase.
            self.collision_hold_until = time.monotonic() + 1.0
            self.last_autonomy_at = 0.0
            return
        self.state = self.reset_mode
        self.countdown_started_at = time.monotonic()
        self.message = self._stage_instruction()

    def _ensure_ghost(self) -> None:
        if getattr(self, 'duel', None):
            return
        if self.ghost_spawned or self.ghost_spawn_pending:
            return
        if not self.spawn_client.service_is_ready():
            return
        request = SpawnEntity.Request()
        request.name = GHOST_ENTITY
        request.xml = GHOST_SDF
        if self.guided:
            # The guided exhibit parks the legacy opponent off-track and uses
            # only the participant's camera. Do not render an unseen third feed.
            ghost = ET.fromstring(GHOST_SDF)
            for link in ghost.findall('.//link'):
                for sensor in list(link.findall('sensor')):
                    link.remove(sensor)
            request.xml = ET.tostring(ghost, encoding='unicode')
        request.robot_namespace = '/ai_ghost'
        request.initial_pose = self._entity_state(GHOST_ENTITY, *self.ghost_pose).pose
        request.reference_frame = 'world'
        future = self.spawn_client.call_async(request)
        self.ghost_spawn_pending = True
        self.ghost_spawn_started_at = time.monotonic()
        self.ghost_spawn_future = future
        future.add_done_callback(self._on_ghost_spawned)

    def _on_ghost_spawned(self, future) -> None:
        if future is not self.ghost_spawn_future:
            return
        self.ghost_spawn_pending = False
        try:
            result = future.result()
            detail = str(getattr(result, 'status_message', '')).lower()
            self.ghost_spawned = bool(result.success) or 'already' in detail
        except Exception as exc:
            self.get_logger().warning(f'Ghost spawn failed: {exc}')
            self.ghost_spawned = False

    @staticmethod
    def _entity_state(name: str, x: float, y: float, yaw: float) -> EntityState:
        state = EntityState()
        state.name = name
        state.reference_frame = 'world'
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = 0.02 if name == GHOST_ENTITY else 0.0
        state.pose.orientation = yaw_to_quaternion(yaw)
        return state

    def _set_ghost_pose(self, pose: Tuple[float, float, float]) -> None:
        if getattr(self, 'duel', None):
            return
        if not self.ghost_spawned or self.ghost_pose_pending or not self.state_client.service_is_ready():
            return
        request = SetEntityState.Request()
        request.state = self._entity_state(GHOST_ENTITY, *pose)
        future = self.state_client.call_async(request)
        self.ghost_pose_pending = True
        self.ghost_pose_started_at = time.monotonic()
        self.ghost_pose_future = future
        future.add_done_callback(self._on_ghost_pose_done)

    def _on_ghost_pose_done(self, future) -> None:
        if future is not self.ghost_pose_future:
            return
        self.ghost_pose_pending = False
        try:
            if not future.result().success:
                self.ghost_spawned = False
        except Exception:
            self.ghost_spawned = False

    def _pose_at_distance(self, distance: float) -> Tuple[float, float, float]:
        value = max(0.0, min(float(distance), self.path_total))
        index = int(np.searchsorted(self.path_s, value, side='right') - 1)
        index = max(0, min(index, len(self.path_points) - 2))
        start_s = self.path_s[index]
        end_s = self.path_s[index + 1]
        fraction = 0.0 if end_s <= start_s else (value - start_s) / (end_s - start_s)
        point = self.path_points[index] * (1.0 - fraction) + self.path_points[index + 1] * fraction
        direction = self.path_points[index + 1] - self.path_points[index]
        yaw = math.atan2(float(direction[1]), float(direction[0]))
        return float(point[0]), float(point[1]), yaw

    def _ghost_tick(self) -> None:
        now = time.monotonic()
        self._check_requests(now)
        self._session_tick(now)
        if self.state == 'COUNTDOWN':
            if getattr(self, 'duel', None) and not self.duel.start_ready:
                self.countdown_started_at = now
                self.message = self.duel.error or f'AI와 장애물 {sum(self.duel.obstacles)}대를 준비하고 있어요'
                if now - self.duel.prepared_at > 30:
                    self._fault(self.duel.error or 'AI 주행 준비를 완료하지 못했어요. 다시 시작해 주세요.')
            elif not self._input_neutral() and not (getattr(self, 'duel', None) and self.duel.countdown_armed):
                self.countdown_started_at = now
                self.message = self.neutral_instruction()
            elif getattr(self, 'duel', None) and not self.duel.countdown_armed:
                self.duel.countdown_armed = True
                self.countdown_started_at = now
            elif now - self.countdown_started_at >= 3.0:
                self.state = 'RUNNING'
                self.input_armed = True
                self.race_started_at = now
                self.race_sim_started = self.sim_time
                if getattr(self, 'duel', None):
                    self.duel.start()
                self.message = self._stage_instruction()
            self.ghost_pose = GHOST_PARK_POSE if self.guided else START_POSE
            self._set_ghost_pose(self.ghost_pose)
        elif self.state == 'RUNNING' and not self.guided:
            self.ai_progress_s = min(self.path_total, self.elapsed * self.difficulty_ratio * MAX_SPEED_MPS)
            self.ghost_pose = self._pose_at_distance(self.ai_progress_s)
            self._set_ghost_pose(self.ghost_pose)
            if now - self.race_started_at >= 80:
                self._finish('TIME', '제한 시간 종료 · 다시 도전해 보세요')
        else:
            self._set_ghost_pose(self.ghost_pose)
        self._publish_ghost_motion()

    def _publish_ghost_motion(self) -> None:
        if getattr(self, 'duel', None):
            return
        command = MotionCommand()
        if self.state == 'RUNNING' and not self.guided:
            future_pose = self._pose_at_distance(
                min(self.path_total, self.ai_progress_s + 0.30)
            )
            yaw_delta = math.atan2(
                math.sin(future_pose[2] - self.ghost_pose[2]),
                math.cos(future_pose[2] - self.ghost_pose[2]),
            )
            command.steering = -int(round(max(-1.0, min(1.0, yaw_delta / 0.24)) * 10.0))
            wheel = int(round(self.difficulty_ratio * 255.0))
            command.left_speed = wheel
            command.right_speed = wheel
        self.ghost_motion_pub.publish(command)

    def _on_joy(self, msg: Joy) -> None:
        if self.input_mode != 'controller':
            self.clear_keyboard()
            self.input_mode = 'controller'
            self.input_armed = False
        self.last_joy_at = time.monotonic()
        self.joy_axes = list(msg.axes)
        self.previous_buttons = self.joy_buttons
        self.joy_buttons = list(msg.buttons)
        pressed_a = bool(self.joy_buttons[0]) if self.joy_buttons else False
        previous_a = bool(self.previous_buttons[0]) if self.previous_buttons else False
        if pressed_a and not previous_a:
            self.request_start()
        pressed_b = len(self.joy_buttons) > 1 and bool(self.joy_buttons[1])
        previous_b = len(self.previous_buttons) > 1 and bool(self.previous_buttons[1])
        if pressed_b and not previous_b:
            self.request_secondary()
        for action in self.controller_navigation.update(self.joy_axes, self.joy_buttons):
            self.display_frame_notifier.action.emit(action)

    def _trigger(self, index: int) -> float:
        if index >= len(self.joy_axes):
            return 0.0
        value = float(self.joy_axes[index])
        rest = self.trigger_rest.get(index, 1.0)
        if rest >= 0.5:
            amount = (rest - value) / max(rest + 1.0, 1e-6)
        elif rest <= -0.5:
            amount = (value - rest) / max(1.0 - rest, 1e-6)
        else:
            amount = max(0.0, value)
        return max(0.0, min(1.0, amount))

    def _controller_command(self) -> Tuple[float, float]:
        steering_axis = float(self.joy_axes[0]) if self.joy_axes else 0.0
        if abs(steering_axis) < 0.08:
            steering_axis = 0.0
        # The exhibition Xbox controller's Bluetooth HID mapping exposes the
        # physical RT on axis 2 and LT on axis 5 after linux_joy normalization.
        throttle = self._trigger(2)
        brake = self._trigger(5)
        steering = steering_axis
        # Brake always wins, including when both triggers are held.
        speed = 0.0 if brake >= 0.03 else throttle
        return steering, speed

    def _keyboard_command(self) -> Tuple[float, float]:
        steering = float(self.keyboard['right']) - float(self.keyboard['left'])
        if self.keyboard['brake']:
            speed = 0.0
        elif self.keyboard['throttle']:
            speed = 0.78
        else:
            speed = 0.0
        return steering, speed

    def _control_tick(self) -> None:
        if getattr(self, 'camera_budget', None):
            self.camera_budget.update()
        if self.state == 'RESETTING' and self.reset_kind == 'safety':
            self._publish_stop()
            if not self.reset_pending and time.monotonic() >= self.collision_hold_until:
                self.state = self.reset_mode
                self.countdown_started_at = time.monotonic()
                self.message = self._stage_instruction()
            return
        if self.state in ('INTRO', 'RUNNING', 'PRACTICE', 'DEMO', 'COUNTDOWN'):
            problem = self._health_problem()
            if problem:
                self.pause_session(problem, manual=False)
        if self.state in ('INTRO', 'DEMO', 'RUNNING', 'PRACTICE'):
            if self._protect_from_collision():
                return
        if self.state in ('INTRO', 'DEMO'):
            if time.monotonic() - self.last_autonomy_at < 0.5:
                pace = 0.35 if self.state == 'INTRO' else 1.0
                command = MotionCommand()
                command.steering = self.autonomy_command.steering
                command.left_speed = int(self.autonomy_command.left_speed * 1.2345 / 1.82 * pace)
                command.right_speed = int(self.autonomy_command.right_speed * 1.2345 / 1.82 * pace)
                self.last_command = command
                self.motion_pub.publish(command)
            else:
                self._publish_stop()
            return
        if self.state not in ('RUNNING', 'PRACTICE'):
            self._publish_stop()
            return
        if not self.input_armed:
            self.input_armed = self._input_neutral()
            self._publish_stop()
            if not self.input_armed:
                self.message = self.neutral_instruction()
            return
        if self.input_mode == 'controller':
            steering, speed = self._controller_command()
        else:
            steering, speed = self._keyboard_command()
        if self.state == 'RUNNING' and getattr(self, 'duel', None):
            brake = self._trigger(5) >= .03 if self.input_mode == 'controller' else self.keyboard['brake']
            throttle = self._trigger(2) if self.input_mode == 'controller' else float(self.keyboard['throttle'])
            command = self.duel.player_command(steering, throttle, brake)
            self.last_command = command
            self.motion_pub.publish(command)
            return
        if self.state == 'PRACTICE':
            if steering < -0.3: self.practice_tasks.add('왼쪽 조향')
            if steering > 0.3: self.practice_tasks.add('오른쪽 조향')
            if speed > 0.1: self.practice_tasks.add('가속')
            if (self._trigger(5) > 0.1 if self.input_mode == 'controller' else self.keyboard['brake']):
                self.practice_tasks.add('제동')
            speed *= 0.55
            self.practice_feedback = ('익힌 조작: ' + ' · '.join(sorted(self.practice_tasks))
                            if self.practice_tasks else '좌우 조향 · 가속 · 제동을 해 보세요')
        command = MotionCommand()
        command.steering = int(round(max(-1.0, min(1.0, steering)) * 10.0))
        wheel = int(round(max(-1.0, min(1.0, speed)) * 255.0))
        command.left_speed = wheel
        command.right_speed = wheel
        self.last_command = command
        self.motion_pub.publish(command)

    def _publish_stop(self) -> None:
        command = MotionCommand()
        command.steering = 0
        command.left_speed = 0
        command.right_speed = 0
        self.last_command = command
        self.motion_pub.publish(command)

    def _on_model_states(self, msg: ModelStates) -> None:
        if getattr(self, 'venue', None) and self.venue.active and 'exhibition_venue_view' in msg.name:
            pose = msg.pose[msg.name.index('exhibition_venue_view')]
            q = pose.orientation
            self.venue.actual_pose = (pose.position.x, pose.position.y, pose.position.z,
                                     math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x)))),
                                     math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        self.scene_safety.update(*read_scene(msg), time.monotonic())
        try:
            index = msg.name.index(PLAYER_ENTITY)
        except ValueError:
            return
        self.last_states_at = time.monotonic()
        pose = msg.pose[index]
        twist = msg.twist[index]
        orientation = pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        x = float(pose.position.x)
        y = float(pose.position.y)
        self.player_pose = (x, y, yaw)
        self.player_speed_mps = math.hypot(float(twist.linear.x), float(twist.linear.y))
        if getattr(self, 'duel', None):
            self.duel.observe_player()
        if self.state == 'RUNNING' and self.player_speed_mps > 0.03:
            self.attempt_started = True
        if self.state not in ('RUNNING', 'PRACTICE'):
            return

        if self.track.contains_world_point(x, y):
            self.outside_since = None
        else:
            now = time.monotonic()
            if self.outside_since is None:
                self.outside_since = now
            if now - self.outside_since >= 0.5:
                self.offtracks += 1
                pose = self._pose_at_distance(max(0.0, self.lap.signed_distance))
                self._reset_player(self.state, pose, recovery=True)
                return
            return
        self._update_player_progress(x, y)

    def _protect_from_collision(self):
        now = time.monotonic()
        if now < self.collision_hold_until:
            self._publish_stop()
            return True
        safety = self.scene_safety.status(now)
        if getattr(self, 'duel', None):
            self.duel.player_risk(safety['stop'] and safety['reason'] != 'position')
        if safety['reason'] == 'position':
            self._publish_stop()
            return True
        if safety['stop'] or now < self.contact_confirmation.pending_until:
            self._publish_stop()
            self.hazard_since = now
            return True
        # A prediction may stop the car, but it can NEVER relocate it. Resume
        # once the path stays clear briefly (for example after obstacle removal).
        if self.hazard_since is not None and now - self.hazard_since < .15:
            self._publish_stop()
            return True
        self.hazard_since = None
        return False

    def _on_contacts(self, msg):
        now = time.monotonic()
        if now < self.collision_hold_until or self.state not in ('INTRO', 'DEMO', 'PRACTICE', 'RUNNING', 'PAUSED', 'COUNTDOWN'):
            return
        confirmed = self.contact_confirmation.observe(msg, now)
        if now < self.contact_confirmation.pending_until:
            self._publish_stop()
        if confirmed:
            self._recover_collision('차량이 실제로 닿았어요.')

    def _recover_collision(self, reason):
        if self.state not in ('INTRO', 'DEMO', 'PRACTICE', 'RUNNING', 'PAUSED', 'COUNTDOWN') or self.reset_pending:
            return
        self._publish_stop()
        now = time.monotonic()
        if now - self.last_collision_recovery_at < 1.5:
            return
        if self.player_pose is None:
            return
        route = self._route_at(*self.player_pose[:2])
        safe = None
        # Continue beyond the blocked section, avoiding a reset/impact loop at
        # the same obstacle. LapProgress.rebase prevents awarding skipped road.
        for ahead in np.arange(3.0, self.path_total + 3.0, .5):
            candidate = self._pose_at_distance((route + ahead) % self.path_total)
            if self.track.contains_world_point(*candidate[:2]) and all(
                    math.hypot(candidate[0] - p[0], candidate[1] - p[1]) > 3.0
                    for p in self.scene_safety.obstacles.values()):
                safe = candidate
                break
        if safe is None:
            self._fault('안전한 복귀 위치를 찾지 못했어요. 장애물을 치워 주세요.')
            return
        mode = self.state
        self.last_collision_recovery_at = now
        self.collision_recoveries += 1
        self.offtracks += 1
        self.hazard_since = None
        self.collision_hold_until = now + 1.0
        # The reference path is lane 2; clear an interrupted lane change.
        self.exhibition_command_pub.publish(String(data='demo_reset'))
        self.demo_feedback = '안전한 도로 위치로 돌아왔어요'
        self._reset_player(mode, safe, recovery=True, recovery_reason=reason)

    def _route_at(self, x, y):
        distances = np.square(self.path_points[:, 0] - x) + np.square(self.path_points[:, 1] - y)
        nearest = int(np.argmin(distances))
        return float(self.path_s[nearest])

    def _update_player_progress(self, x: float, y: float) -> None:
        self.player_progress_s = self.lap.update(self._route_at(x, y))
        if self.state == 'RUNNING' and self.elapsed >= 8.0 and self.player_progress_s >= self.path_total - 0.45:
            self._finish('PLAYER', '완주했어요! 결과를 보고 다음 도전을 준비하세요')

    def _finish(self, winner: str, message: str) -> None:
        if self.state not in ('RUNNING', 'COUNTDOWN'):
            return
        if getattr(self, 'duel', None):
            self.duel.player_finished(complete=winner == 'PLAYER')
        self.winner = winner
        self.message = message
        self.state = 'GAME_OVER' if winner == 'OUT' else 'FINISHED'
        self.race_finished_at = time.monotonic()
        self.race_sim_finished = self.sim_time
        self._record_attempt()
        self._publish_stop()
        if self.guided:
            self.session.go('RESULTS', time.monotonic())
            self.state = 'RESULTS'

    def _on_image(self, msg: Image) -> None:
        self.camera_received_frames += 1
        now = time.monotonic()
        if self.display_fps < 60 and now - self.last_camera_submitted_at < 1.0 / self.display_fps:
            return
        self.last_camera_submitted_at = now
        self.display_converter.submit(msg)

    @staticmethod
    def _convert_display_image(msg):
        return display_image(msg)

    def collect_display_frame(self):
        result = self.display_converter.latest
        if result is not None and result[0] != self.image_sequence:
            self.image_sequence, image, self.last_image_at = result
            self.image = (self.image_sequence, image)

    def _on_lesson_image(self, msg, key='fitting'):
        now = time.monotonic()
        teaching = self.state == 'INTRO'
        if key == 'source' and not teaching:
            return
        if (key == 'fitting' or not teaching) and now - self.lesson_received_at.get(key, 0) < 0.15:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = frame.shape[:2]
            image = QImage(frame.data, width, height, int(frame.strides[0]), QImage.Format_BGR888).copy()
        except Exception:
            return
        self.lesson_images[key] = (now, image)
        self.lesson_received_at[key] = now
        if teaching and key in ('source', 'segmentation', 'bev'):
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            self.teaching_frames.add(stamp, 'camera' if key == 'source' else key,
                                    image, frame.copy() if key == 'segmentation' else None, now)

    @staticmethod
    def _draw_marker(frame: np.ndarray, pose, color, label: str) -> None:
        height, width = frame.shape[:2]
        u, v = TrackBoundaryOverlay.world_to_normalized(pose[0], pose[1])
        point = np.array([u * width, v * height], dtype=np.float32)
        if not (0 <= point[0] < width and 0 <= point[1] < height):
            return

        # Small directional car marker. It stays legible without covering the
        # actual vehicle or nearby track boundary on the overhead view.
        yaw = float(pose[2])
        forward = np.array([-math.sin(yaw), -math.cos(yaw)], dtype=np.float32)
        right = np.array([forward[1], -forward[0]], dtype=np.float32)
        marker_length = max(7.0, width / 150.0)
        marker_width = marker_length * 0.58
        polygon = np.array(
            [
                point + forward * marker_length,
                point - forward * marker_length * 0.72 + right * marker_width,
                point - forward * marker_length * 0.72 - right * marker_width,
            ],
            dtype=np.int32,
        )
        cv2.polylines(frame, [polygon], True, (4, 12, 20), 4, cv2.LINE_AA)
        cv2.fillConvexPoly(frame, polygon, color, cv2.LINE_AA)

        anchor = tuple(
            np.rint(point + right * (marker_width + 5.0) - forward * 2.0).astype(np.int32)
        )
        cv2.putText(
            frame,
            label,
            anchor,
            cv2.FONT_HERSHEY_SIMPLEX,
            max(0.28, width / 2600.0),
            color,
            1,
            cv2.LINE_AA,
        )


class RaceGameWindow(QWidget):
    def __init__(self, node: RaceGameNode):
        super().__init__()
        self.node = node
        self.setWindowTitle('AI 고스트 레이스')
        self.setMinimumSize(1280, 720)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setObjectName('raceGame')
        self.setStyleSheet(
            f'QWidget#raceGame {{ background: {BG}; color: {TEXT}; }}'
            f'QWidget#raceGame QLabel {{ background: transparent; color: {TEXT}; }}'
        )

        root = QVBoxLayout(self)
        root.setContentsMargins(22, 16, 22, 18)
        root.setSpacing(13)
        root.addLayout(self._build_header())

        body = QHBoxLayout()
        body.setSpacing(14)
        body.addWidget(self._build_track_card(), 72)
        body.addWidget(self._build_control_card(), 28)
        root.addLayout(body, 1)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh)
        self.refresh_timer.start(40)

    def _build_header(self):
        row = QHBoxLayout()
        title_box = QVBoxLayout()
        title = QLabel('AI와 함께 배우고, 직접 달려보세요' if self.node.guided else 'AI보다 빠르게 달려라!')
        title.setFont(QFont('Noto Sans CJK KR', 28, QFont.Bold))
        subtitle = QLabel('약 5분 · AI 관찰 → 조작 연습 → 두 번의 도전 → 나의 결과' if self.node.guided else '기록 경로를 따라 달리는 AI 고스트와 한 바퀴에 도전하세요')
        subtitle.setFont(QFont('Noto Sans CJK KR', 11))
        subtitle.setStyleSheet(f'color: {MUTED};')
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        row.addLayout(title_box)
        row.addStretch(1)
        self.mode_pill = StatusPill('하수 · AI 50%', GREEN)
        self.controller_pill = StatusPill('○ 컨트롤러 대기', MUTED)
        self.state_pill = StatusPill('READY', BLUE)
        row.addWidget(self.mode_pill)
        row.addWidget(self.controller_pill)
        row.addWidget(self.state_pill)
        return row

    def _build_track_card(self):
        card = QFrame()
        card.setObjectName('card')
        card.setStyleSheet(panel_style(TEAL))
        layout = QVBoxLayout(card)
        layout.setContentsMargins(12, 10, 12, 12)
        layout.setSpacing(8)

        heading = QHBoxLayout()
        title = QLabel('운전자 시점 · 전면 카메라')
        title.setFont(QFont('Noto Sans CJK KR', 18, QFont.Bold))
        legend = QLabel('AI GHOST: 기록 경로 재생  ·  이탈하면 도로로 복귀')
        legend.setFont(QFont('Noto Sans CJK KR', 10, QFont.Bold))
        legend.setStyleSheet(f'color: {MUTED};')
        heading.addWidget(title)
        heading.addStretch(1)
        heading.addWidget(legend)
        layout.addLayout(heading)

        self.track_canvas = ImageCanvas('차량 전면 카메라 연결 대기')
        layout.addWidget(self.track_canvas, 1)
        self.result_panel = QLabel()
        self.result_panel.setAlignment(Qt.AlignCenter)
        self.result_panel.setWordWrap(True)
        self.result_panel.setFont(QFont('Noto Sans CJK KR', 25, QFont.Bold))
        self.result_panel.setStyleSheet(f'color: {TEAL}; padding: 24px;')
        self.result_panel.hide()
        layout.addWidget(self.result_panel, 1)

        self.banner = QLabel('난이도를 선택하고 시작하세요')
        self.banner.setAlignment(Qt.AlignCenter)
        self.banner.setWordWrap(True)
        self.banner.setMinimumHeight(58)
        self.banner.setFont(QFont('Noto Sans CJK KR', 19, QFont.Bold))
        self.banner.setStyleSheet(
            f'background: #071727; color: {TEAL}; border: 1px solid #285b76; '
            'border-radius: 12px; padding: 8px;'
        )
        layout.addWidget(self.banner)
        return card

    def _build_control_card(self):
        card = QFrame()
        card.setObjectName('card')
        card.setStyleSheet(panel_style(BLUE))
        layout = QVBoxLayout(card)
        layout.setContentsMargins(14, 12, 14, 14)
        layout.setSpacing(10)
        self.control_layout = layout

        title = QLabel('나의 5분 체험' if self.node.guided else '게임 설정')
        self.control_title = title
        title.setFont(QFont('Noto Sans CJK KR', 19, QFont.Bold))
        layout.addWidget(title)

        difficulty_label = QLabel('난이도 · AI 최고 속도')
        difficulty_label.setStyleSheet(f'color: {MUTED};')
        difficulty_label.setFont(QFont('Noto Sans CJK KR', 9, QFont.Bold))
        layout.addWidget(difficulty_label)
        difficulty_label.setVisible(not self.node.guided)
        self.difficulty_buttons = {}
        for key, (label, ratio, color) in DIFFICULTIES.items():
            button = QPushButton(f'{label}     AI {int(ratio * 100)}%')
            button.setMinimumHeight(43)
            button.setCursor(Qt.PointingHandCursor)
            button.setFont(QFont('Noto Sans CJK KR', 11, QFont.Bold))
            button.clicked.connect(lambda _checked=False, value=key: self.node.set_difficulty(value))
            self.difficulty_buttons[key] = button
            layout.addWidget(button)
            button.setVisible(not self.node.guided)

        self.session_guide = QLabel('안내 20초 → AI 관찰 40초\n연습 40초 → 도전 2회 각 80초\n결과 확인 → 다음 참가자 준비')
        self.session_guide.setWordWrap(True)
        self.session_guide.setFont(QFont('Noto Sans CJK KR', 11, QFont.Bold))
        self.session_guide.setStyleSheet(f'color: {TEAL}; padding: 8px;')
        self.session_guide.setVisible(self.node.guided)
        layout.addWidget(self.session_guide)

        self.timer_value = QLabel('00:00.00')
        self.timer_value.setAlignment(Qt.AlignCenter)
        self.timer_value.setFont(QFont('Noto Sans', 31, QFont.Bold))
        self.timer_value.setStyleSheet(f'color: {TEAL}; padding: 5px;')
        layout.addWidget(self.timer_value)

        self.player_label = QLabel('YOU   0%')
        self.ai_label = QLabel('AI GHOST   0%')
        self.player_progress = self._progress_bar('#ffd63d')
        self.ai_progress = self._progress_bar(TEAL)
        layout.addWidget(self.player_label)
        layout.addWidget(self.player_progress)
        layout.addWidget(self.ai_label)
        layout.addWidget(self.ai_progress)

        self.speed_value = QLabel('속도  0%')
        self.speed_value.setAlignment(Qt.AlignCenter)
        self.speed_value.setFont(QFont('Noto Sans CJK KR', 16, QFont.Bold))
        self.speed_value.setStyleSheet(
            'background: #091a2e; border: 1px solid #244b70; '
            'border-radius: 10px; padding: 8px;'
        )
        layout.addWidget(self.speed_value)

        guide = QLabel(
            '왼쪽 스틱  조향\n'
            'RT  가속 · LT  제동\n'
            'A  시작/계속 · B  체험 종료\n'
            '키보드: 방향키 · Space\n'
            'Esc  체험 종료'
        )
        self.input_guide = guide
        guide.setFont(QFont('Noto Sans CJK KR', 10, QFont.Bold))
        guide.setStyleSheet(
            f'color: {MUTED}; background: #071727; border: 1px solid #23435f; '
            'border-radius: 10px; padding: 10px; line-height: 1.4;'
        )
        layout.addWidget(guide)
        layout.addStretch(1)

        self.start_button = QPushButton('게임 시작')
        self.start_button.setMinimumHeight(58)
        self.start_button.setCursor(Qt.PointingHandCursor)
        self.start_button.setFont(QFont('Noto Sans CJK KR', 17, QFont.Bold))
        self.start_button.clicked.connect(self.node.request_start)
        layout.addWidget(self.start_button)
        self.cancel_button = QPushButton('체험 종료 · 다음 참가자')
        self.cancel_button.setMinimumHeight(40)
        self.cancel_button.setStyleSheet(self._button_style(MUTED, False))
        self.cancel_button.clicked.connect(self.node.cancel_session)
        layout.addWidget(self.cancel_button)
        return card

    @staticmethod
    def _progress_bar(color):
        bar = QProgressBar()
        bar.setRange(0, 1000)
        bar.setTextVisible(False)
        bar.setFixedHeight(17)
        bar.setStyleSheet(
            'QProgressBar { background: #071321; border: 1px solid #24435f; '
            'border-radius: 8px; }'
            f'QProgressBar::chunk {{ background: {color}; border-radius: 7px; }}'
        )
        return bar

    @staticmethod
    def _button_style(color: str, active: bool, enabled: bool = True):
        background = '#173d54' if active else '#0a2037'
        muted = color if enabled else '#40536a'
        return (
            f'QPushButton {{ color: {TEXT if enabled else MUTED}; background: {background}; '
            f'border: {2 if active else 1}px solid {muted}; border-radius: 10px; }}'
            f'QPushButton:hover {{ background: #173b59; border: 2px solid {muted}; }}'
        )

    def refresh(self):
        if self.node.image is not None:
            self.track_canvas.set_frame(*self.node.image)

        label, ratio, color = DIFFICULTIES[self.node.difficulty]
        self.mode_pill.set_state(f'{label} · AI {int(ratio * 100)}%', color)
        if self.node.controller_connected:
            self.controller_pill.set_state('● 컨트롤러 연결', GREEN)
        else:
            self.controller_pill.set_state('○ 키보드 조작', MUTED)

        state_text = {
            'READY': 'READY',
            'RESETTING': 'SETTING',
            'COUNTDOWN': self.node.countdown_text,
            'RUNNING': 'RACING',
            'GAME_OVER': 'GAME OVER',
            'FINISHED': 'FINISH',
        }.get(self.node.state, self.node.state)
        state_color = RED if self.node.state == 'GAME_OVER' else (
            GREEN if self.node.winner == 'PLAYER' else BLUE
        )
        if self.node.guided and self.node.session.phase != 'IDLE':
            state_text = self.node.session.label
        self.state_pill.set_state(state_text, state_color)

        if self.node.state == 'COUNTDOWN':
            banner = self.node.countdown_text
            banner_color = CORAL
        elif self.node.state == 'RUNNING':
            banner = self.node.message
            banner_color = TEAL
        elif self.node.winner == 'PLAYER':
            banner = '완주 성공!  ' + self.node.message
            banner_color = GREEN
        elif self.node.winner == 'AI':
            banner = 'AI WIN  ' + self.node.message
            banner_color = CORAL
        elif self.node.winner == 'OUT':
            banner = 'GAME OVER  ' + self.node.message
            banner_color = RED
        else:
            banner = self.node.message
            banner_color = TEAL
        self.banner.setText(banner)
        show_results = self.node.guided and self.node.state in ('RESULTS', 'FINISHED')
        self.track_canvas.setVisible(not show_results)
        self.result_panel.setVisible(show_results)
        if show_results:
            self.result_panel.setText('나의 도전 기록\n\n' + result_text(self.node.session.results)
                                      + '\n\n' + (self.node.session.error or '먼저 속도를 낮추고, 커브를 돌아 가속해 보세요'))
        self.banner.setStyleSheet(
            f'background: #071727; color: {banner_color}; border: 1px solid {banner_color}; '
            'border-radius: 12px; padding: 8px;'
        )

        elapsed = self.node.elapsed
        minutes = int(elapsed // 60)
        seconds = elapsed - minutes * 60
        self.timer_value.setText(f'{minutes:02d}:{seconds:05.2f}')
        if self.node.guided:
            remaining = math.ceil(self.node.session.remaining(time.monotonic()))
            self.timer_value.setText(f'{remaining // 60:02d}:{remaining % 60:02d}')
            self.session_guide.setText(
                self.node.session.label + (' · 남은 시간' if self.node.session.phase != 'IDLE' else '')
                + ('\nAI 관찰 → 연습 → 도전 2회' if self.height() < 850
                   else '\n안내 → AI 관찰 → 연습\n도전 1 → 도전 2 → 결과')
            )
        self.player_progress.setValue(int(round(self.node.player_percent * 10.0)))
        self.ai_progress.setValue(int(round(self.node.ai_percent * 10.0)))
        self.player_label.setText(f'YOU   {self.node.player_percent:.0f}%')
        self.ai_label.setText(f'AI GHOST   {self.node.ai_percent:.0f}%')
        speed_percent = min(100.0, 100.0 * self.node.player_speed_mps / MAX_SPEED_MPS)
        self.speed_value.setText(f'속도  {speed_percent:.0f}%')

        controls_locked = self.node.state in ('COUNTDOWN', 'RUNNING', 'RESETTING')
        for key, button in self.difficulty_buttons.items():
            button.setEnabled(not controls_locked)
            button.setStyleSheet(
                self._button_style(DIFFICULTIES[key][2], key == self.node.difficulty, not controls_locked)
            )
        self.start_button.setEnabled(not self.node.reset_pending and self.node.state not in ('COUNTDOWN', 'RUNNING'))
        self.start_button.setText('다시 도전' if self.node.state in ('GAME_OVER', 'FINISHED') else '게임 시작')
        if self.node.guided:
            enabled = (self.node.session.phase == 'IDLE' or self.node.state == 'PAUSED'
                       or (self.node.state == 'FINISHED' and time.monotonic() - self.node.race_finished_at >= 5))
            self.start_button.setEnabled(enabled and not self.node.reset_pending)
            self.start_button.setText('연결 확인 후 계속' if self.node.state == 'PAUSED' else (
                '다음 단계로' if self.node.state == 'FINISHED' else '5분 체험 시작'
            ))
        self.cancel_button.setEnabled(self.node.session.phase not in ('IDLE', 'RESET') if self.node.guided else True)
        self.start_button.setStyleSheet(self._button_style(GREEN, True, self.start_button.isEnabled()))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if not hasattr(self, 'input_guide') or not self.node.guided:
            return
        compact = self.height() < 850
        self.control_layout.setSpacing(5 if compact else 10)
        self.control_title.setFont(QFont('Noto Sans CJK KR', 14 if compact else 19, QFont.Bold))
        self.session_guide.setFont(QFont('Noto Sans CJK KR', 9 if compact else 11, QFont.Bold))
        self.session_guide.setMinimumHeight(42 if compact else 72)
        self.timer_value.setFont(QFont('Noto Sans', 24 if compact else 31, QFont.Bold))
        self.input_guide.setFont(QFont('Noto Sans CJK KR', 8 if compact else 10, QFont.Bold))
        self.input_guide.setText('← → / 스틱  조향\n↑ / RT  가속 · ↓ / LT  제동\nSpace / A  시작·계속\nEsc / B  체험 종료')
        self.input_guide.setMinimumHeight(84 if compact else 104)
        self.start_button.setMinimumHeight(42 if compact else 58)
        self.start_button.setFont(QFont('Noto Sans CJK KR', 13 if compact else 17, QFont.Bold))
        self.cancel_button.setMinimumHeight(32 if compact else 40)

    def focusOutEvent(self, event):
        self.node.clear_keyboard()
        self.node.input_armed = False
        super().focusOutEvent(event)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Q:
            self.close()
            return
        if event.key() == Qt.Key_Escape:
            self.node.cancel_session()
            return
        if event.key() == Qt.Key_F11:
            self.showNormal() if self.isFullScreen() else self.showFullScreen()
            return
        if event.isAutoRepeat():
            return
        if event.key() in (Qt.Key_Space, Qt.Key_R):
            self.node.request_start()
            return
        difficulty_keys = {
            Qt.Key_1: 'beginner',
            Qt.Key_2: 'intermediate',
            Qt.Key_3: 'expert',
        }
        if event.key() in difficulty_keys:
            self.node.set_difficulty(difficulty_keys[event.key()])
            return
        actions = {
            Qt.Key_Left: 'left', Qt.Key_A: 'left',
            Qt.Key_Right: 'right', Qt.Key_D: 'right',
            Qt.Key_Up: 'throttle', Qt.Key_W: 'throttle',
            Qt.Key_Down: 'brake', Qt.Key_S: 'brake',
        }
        if event.key() in actions:
            self.node.set_keyboard(actions[event.key()], True)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        actions = {
            Qt.Key_Left: 'left', Qt.Key_A: 'left',
            Qt.Key_Right: 'right', Qt.Key_D: 'right',
            Qt.Key_Up: 'throttle', Qt.Key_W: 'throttle',
            Qt.Key_Down: 'brake', Qt.Key_S: 'brake',
        }
        if event.key() in actions:
            self.node.set_keyboard(actions[event.key()], False)


def main(args=None):
    rclpy.init(args=args)
    # Small display filters are faster and more predictable without starting
    # a large OpenCV worker group for every high-rate frame.
    cv2.setNumThreads(1)
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    if os.environ.get('E2E_GPU_CAMERA_DISPLAY') == '1':
        fmt = QSurfaceFormat()
        fmt.setSwapInterval(0)
        QSurfaceFormat.setDefaultFormat(fmt)
    app = QApplication(sys.argv)
    app.setApplicationName('E2E AI Ghost Race')
    app.setFont(QFont('Noto Sans CJK KR', 10))

    node = RaceGameNode()
    if node.guided:
        from exhibition_gui_pkg.visitor_ui import VisitorDriverWindow
        window = VisitorDriverWindow(node)
    else:
        window = RaceGameWindow(node)
    from exhibition_gui_pkg.ros_worker import RosWorker
    node.display_frame_notifier.quit.connect(app.quit, Qt.QueuedConnection)
    ros_worker = RosWorker(node, node.display_frame_notifier.quit.emit)
    ros_worker.start()
    screens = app.screens()
    screen = screens[min(node.screen_index, len(screens) - 1)]
    window.winId()
    if window.windowHandle() is not None:
        window.windowHandle().setScreen(screen)
    window.setGeometry(screen.geometry())
    window.showFullScreen() if node.fullscreen else window.show()
    try:
        result = app.exec_()
    finally:
        ros_worker.close()
        node.display_converter.close()
        if node.venue:
            node.venue.close()
        node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(result)


if __name__ == '__main__':
    main()
