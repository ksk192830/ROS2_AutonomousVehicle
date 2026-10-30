#!/usr/bin/env python3

import json
import math
import os
import re
import sys
import time
from collections import deque
from typing import Dict, Optional, Tuple

SYSTEM_QT_PLATFORM_PATH = '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms'

import rclpy
from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
from interfaces_pkg.msg import LaneInfo, MotionCommand
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String

from exhibition_gui_pkg.track_overlay import TrackBoundaryOverlay
from exhibition_gui_pkg.display_camera import FrameNotifier, LatestFrameConverter, display_image

# The pip OpenCV build overwrites this variable while importing cv2. Restore
# Ubuntu's PyQt5 plugin path afterwards so QApplication can load xcb safely.
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = SYSTEM_QT_PLATFORM_PATH

from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import (
    QColor,
    QFont,
    QImage,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
    QSurfaceFormat,
)
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)


BG = '#06101e'
PANEL = '#0b1b31'
PANEL_2 = '#0d2039'
TEXT = '#f4f8ff'
MUTED = '#9eb0c8'
BLUE = '#2588ff'
VIOLET = '#a855f7'
TEAL = '#25d8d0'
CORAL = '#ff996e'
GREEN = '#46e37c'
RED = '#ff5c6c'


def panel_style(border: str) -> str:
    return (
        f'QFrame#card {{ background: {PANEL}; border: 2px solid {border}; '
        'border-radius: 18px; }'
    )


class ImageCanvas(QWidget):
    def __init__(
        self,
        placeholder: str = '데이터 준비 중',
        display_mode: str = 'plain',
        parent=None,
    ):
        super().__init__(parent)
        self.placeholder = placeholder
        self.display_mode = display_mode
        self._pixmap: Optional[QPixmap] = None
        self._sequence = -1
        self._painted_sequence = None
        self.frames_presented = 0
        self._scaled_frame = None
        self._scaled_key = None
        self.setMinimumSize(120, 110)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_frame(self, sequence: int, image: Optional[QImage]) -> None:
        if image is None or sequence == self._sequence:
            return
        self._sequence = sequence
        self._pixmap = QPixmap.fromImage(image)
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        bounds = QRectF(self.rect()).adjusted(1, 1, -1, -1)

        clip = QPainterPath()
        clip.addRoundedRect(bounds, 10, 10)
        painter.setClipPath(clip)
        painter.fillRect(bounds, QColor('#071321'))

        content_bounds = bounds
        if self._pixmap is None or self._pixmap.isNull():
            painter.setPen(QColor(MUTED))
            painter.setFont(QFont('Noto Sans CJK KR', 11))
            painter.drawText(bounds, Qt.AlignCenter, self.placeholder)
        else:
            key = (self._pixmap.cacheKey(), self.width(), self.height())
            if key != self._scaled_key:
                self._scaled_frame = self._pixmap.scaled(
                    self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
                )
                self._scaled_key = key
            scaled = self._scaled_frame
            x = (self.width() - scaled.width()) // 2
            y = (self.height() - scaled.height()) // 2
            painter.drawPixmap(x, y, scaled)
            if self._sequence != self._painted_sequence:
                self._painted_sequence = self._sequence
                self.frames_presented += 1
            content_bounds = QRectF(x, y, scaled.width(), scaled.height())

            if self.display_mode in ('bev', 'fitting'):
                painter.save()
                self._paint_exhibition_legend(painter, content_bounds)
                painter.restore()

        painter.setClipping(False)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor('#21466d'), 1))
        painter.drawRoundedRect(bounds, 10, 10)

    def _paint_exhibition_legend(
        self, painter: QPainter, content_bounds: QRectF
    ) -> None:
        """Keep the technical overlay readable without covering the road."""
        if content_bounds.width() < 190 or content_bounds.height() < 90:
            return

        margin = 10.0
        height = 29.0
        if self.display_mode == 'bev':
            width = min(224.0, content_bounds.width() - margin * 2)
            labels = (('차선 1', TEAL), ('차선 2', CORAL))
        else:
            width = min(250.0, content_bounds.width() - margin * 2)
            labels = (('실제 검출점', '#eef5ff'), ('차선 중앙선', GREEN))

        legend = QRectF(
            content_bounds.right() - width - margin,
            content_bounds.bottom() - height - margin,
            width,
            height,
        )
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(4, 14, 24, 210))
        painter.drawRoundedRect(legend, 8, 8)
        painter.setFont(QFont('Noto Sans CJK KR', 8, QFont.Bold))

        x = legend.left() + 12
        center_y = legend.center().y()
        for index, (label, color) in enumerate(labels):
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(color))
            if self.display_mode == 'fitting' and index == 0:
                painter.drawEllipse(QPointF(x + 4, center_y), 3, 3)
            else:
                painter.setPen(QPen(QColor(color), 3, Qt.SolidLine, Qt.RoundCap))
                painter.drawLine(QPointF(x, center_y), QPointF(x + 12, center_y))
            painter.setPen(QColor('#eef5ff'))
            painter.drawText(
                QRectF(x + 18, legend.top(), 82, legend.height()),
                Qt.AlignVCenter | Qt.AlignLeft,
                label,
            )
            x += 99 if self.display_mode == 'bev' else 108


class StageImage(QFrame):
    def __init__(
        self,
        step: str,
        title: str,
        technical: str,
        accent: str = BLUE,
        display_mode: str = 'plain',
        emphasis: bool = False,
        parent=None,
    ):
        super().__init__(parent)
        self.setObjectName('stage')
        self.setStyleSheet(
            f'QFrame#stage {{ background: {PANEL_2}; '
            f'border: 1px solid {"#31506c" if emphasis else "#203c58"}; '
            'border-radius: 14px; }'
            f'QLabel {{ color: {TEXT}; background: transparent; border: none; }}'
        )
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 7, 8, 8)
        layout.setSpacing(5)

        accent_bar = QFrame()
        accent_bar.setFixedHeight(3)
        accent_bar.setStyleSheet(
            f'background: {accent}; border: none; border-radius: 1px;'
        )
        layout.addWidget(accent_bar)

        header = QHBoxLayout()
        header.setSpacing(7)
        badge = QLabel(step)
        badge.setFixedWidth(27)
        badge.setAlignment(Qt.AlignCenter)
        badge.setFont(QFont('Noto Sans', 11, QFont.Bold))
        badge.setStyleSheet(f'color: {accent};')
        label = QLabel(title)
        label.setFont(
            QFont('Noto Sans CJK KR', 13 if emphasis else 11, QFont.Bold)
        )
        tech = QLabel(technical)
        tech.setFont(QFont('Noto Sans CJK KR', 8))
        tech.setStyleSheet(f'color: {MUTED};')
        header.addWidget(badge)
        header.addWidget(label)
        header.addStretch(1)
        header.addWidget(tech)

        self.canvas = ImageCanvas(display_mode=display_mode)
        layout.addLayout(header)
        layout.addWidget(self.canvas, 1)


class FlowArrow(QWidget):
    def __init__(self, color: str = BLUE, width: int = 32, parent=None):
        super().__init__(parent)
        self.color = QColor(color)
        self.setFixedWidth(width)
        self.setMinimumHeight(40)

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        y = self.height() / 2
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(self.color, 2, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        painter.drawLine(QPointF(3, y), QPointF(self.width() - 6, y))
        painter.drawLine(
            QPointF(self.width() - 12, y - 7), QPointF(self.width() - 5, y)
        )
        painter.drawLine(
            QPointF(self.width() - 12, y + 7), QPointF(self.width() - 5, y)
        )


class StatusPill(QLabel):
    def __init__(self, text: str, color: str, parent=None):
        super().__init__(text, parent)
        self.color = color
        self.setAlignment(Qt.AlignCenter)
        self.setFont(QFont('Noto Sans CJK KR', 10, QFont.Bold))
        self.setMinimumWidth(105)
        self.setFixedHeight(34)
        self.set_state(text, color)

    def set_state(self, text: str, color: Optional[str] = None) -> None:
        if color is not None:
            self.color = color
        self.setText(text)
        self.setStyleSheet(
            f'color: {self.color}; background: rgba(5, 20, 30, 210); '
            f'border: 1px solid {self.color}; border-radius: 16px; padding: 3px 12px;'
        )


class DecisionGraphic(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.lane_num = 1
        self.position_error = 0
        self.state = '차선 유지'
        self.setMinimumSize(180, 220)

    def set_values(self, lane_num: int, position_error: int) -> None:
        self.lane_num = lane_num
        self.position_error = position_error
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        r = QRectF(self.rect()).adjusted(8, 4, -8, -4)

        road = QPainterPath()
        road.moveTo(r.left() + r.width() * 0.20, r.bottom())
        road.lineTo(r.left() + r.width() * 0.38, r.top())
        road.lineTo(r.left() + r.width() * 0.62, r.top())
        road.lineTo(r.left() + r.width() * 0.80, r.bottom())
        road.closeSubpath()
        painter.fillPath(road, QColor('#10172a'))

        lane_pen = QPen(QColor('#7940c7'), 3)
        painter.setPen(lane_pen)
        painter.drawLine(
            QPointF(r.left() + r.width() * 0.20, r.bottom()),
            QPointF(r.left() + r.width() * 0.38, r.top()),
        )
        painter.drawLine(
            QPointF(r.left() + r.width() * 0.80, r.bottom()),
            QPointF(r.left() + r.width() * 0.62, r.top()),
        )

        path = QPainterPath(QPointF(r.center().x(), r.bottom() - 35))
        path.cubicTo(
            r.center().x() - self.position_error * 0.04,
            r.center().y(),
            r.center().x(),
            r.top() + 45,
            r.center().x(),
            r.top() + 18,
        )
        painter.setPen(QPen(QColor(VIOLET), 5, Qt.SolidLine, Qt.RoundCap))
        painter.drawPath(path)

        car_w = max(24.0, r.width() * 0.16)
        car_h = max(42.0, r.height() * 0.20)
        car = QRectF(
            r.center().x() - car_w / 2,
            r.bottom() - car_h - 18,
            car_w,
            car_h,
        )
        painter.setBrush(QColor('#f7fbff'))
        painter.setPen(QPen(QColor('#8ca4bd'), 2))
        painter.drawRoundedRect(car, 8, 8)
        painter.setBrush(QColor('#163554'))
        painter.drawRoundedRect(car.adjusted(5, 7, -5, -car_h * 0.48), 4, 4)


class SteeringGraphic(QWidget):
    """Rotate a photographic steering-wheel asset from the live command."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.target_deg = 0.0
        self.display_deg = 0.0
        self.display_velocity = 0.0
        self.animation_at = time.monotonic()
        asset_path = os.path.join(
            get_package_share_directory('exhibition_gui_pkg'),
            'assets',
            'steering_wheel.png',
        )
        self.wheel_pixmap = QPixmap(asset_path)
        if self.wheel_pixmap.isNull():
            self.wheel_pixmap = QPixmap(1, 1)
            self.wheel_pixmap.fill(Qt.transparent)
        self.setMinimumSize(220, 240)
        self.animation_timer = QTimer(self)
        self.animation_timer.setTimerType(Qt.PreciseTimer)
        self.animation_timer.timeout.connect(self._animate)

    def set_value(self, steering: int) -> None:
        self.target_deg = max(-25.18, min(25.18, steering / 10.0 * 25.18))
        if not self.isVisible():
            self.display_deg = self.target_deg
            self.display_velocity = 0.0
        elif (not self.animation_timer.isActive() and
              (abs(self.target_deg - self.display_deg) >= .01 or abs(self.display_velocity) >= .05)):
            self.animation_at = time.monotonic()
            self.animation_timer.start(16)

    def showEvent(self, event) -> None:
        super().showEvent(event)
        self.animation_at = time.monotonic()
        self.animation_timer.start(16)

    def hideEvent(self, event) -> None:
        self.animation_timer.stop()
        self.display_velocity = 0.0
        super().hideEvent(event)

    def _advance_animation(self, elapsed: float) -> None:
        # A critically damped response preserves velocity when a new command
        # arrives. One-step corrections no longer restart a sharp rotation.
        # This is presentation only; the vehicle receives the original command.
        elapsed = max(0.0, min(elapsed, .05))
        omega = 11.0
        error = self.display_deg - self.target_deg
        momentum = self.display_velocity + omega * error
        decay = math.exp(-omega * elapsed)
        position = self.target_deg + (error + momentum * elapsed) * decay
        velocity = (self.display_velocity - omega * momentum * elapsed) * decay
        step = max(-55.0 * elapsed, min(55.0 * elapsed, position - self.display_deg))
        self.display_deg = max(-25.18, min(25.18, self.display_deg + step))
        self.display_velocity = max(-55.0, min(55.0, velocity))

    def _animate(self) -> None:
        now = time.monotonic()
        self._advance_animation(now - self.animation_at)
        self.animation_at = now
        if abs(self.target_deg - self.display_deg) < .01 and abs(self.display_velocity) < .05:
            self.display_deg = self.target_deg
            self.display_velocity = 0.0
            self.animation_timer.stop()
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)
        w, h = self.width(), self.height()
        center = QPointF(w * 0.5, h * 0.43)
        radius = min(w * 0.31, h * 0.30)

        # Stationary command scale behind the wheel.
        painter.setPen(QPen(QColor('#173853'), 3, Qt.SolidLine, Qt.RoundCap))
        for index in range(33):
            tick_angle = math.radians(-220 + index * (260 / 32))
            inner = radius * (1.18 if index % 4 == 0 else 1.23)
            outer = radius * 1.31
            p1 = QPointF(
                center.x() + math.cos(tick_angle) * inner,
                center.y() + math.sin(tick_angle) * inner,
            )
            p2 = QPointF(
                center.x() + math.cos(tick_angle) * outer,
                center.y() + math.sin(tick_angle) * outer,
            )
            painter.drawLine(p1, p2)

        # Rotate the complete wheel. A visual steering ratio makes small road
        # wheel angles obvious to visitors while the numeric value stays exact.
        painter.save()
        painter.translate(center)
        painter.rotate(self.display_deg * 8.0)

        # The source PNG is square and transparent, so rotating the complete
        # image keeps its realistic lighting and spoke geometry intact.
        wheel_size = radius * 2.18
        target = QRectF(-wheel_size / 2, -wheel_size / 2, wheel_size, wheel_size)
        source = QRectF(self.wheel_pixmap.rect())
        painter.drawPixmap(target, self.wheel_pixmap, source)
        painter.restore()

        if self.display_deg < -0.4:
            direction = '좌회전'
        elif self.display_deg > 0.4:
            direction = '우회전'
        else:
            direction = '직진'

        painter.setPen(QColor(TEAL))
        painter.setFont(QFont('Noto Sans CJK KR', 30, QFont.Bold))
        painter.drawText(
            QRectF(0, h * 0.76, w, h * 0.13),
            Qt.AlignCenter,
            f'{self.display_deg:+.1f}°',
        )
        painter.setPen(QColor(TEXT))
        painter.setFont(QFont('Noto Sans CJK KR', 15, QFont.Bold))
        painter.drawText(QRectF(0, h * 0.88, w, h * 0.09), Qt.AlignCenter, direction)


class RosDashboardNode(Node):
    def __init__(self):
        super().__init__('exhibition_dashboard')
        self.declare_parameter('fullscreen', True)
        self.declare_parameter('display_fps', 12.0)
        self.declare_parameter('screen_index', 0)
        self.declare_parameter('topic_prefix', '')
        self.declare_parameter('motion_topic', '/motion_command')
        self.declare_parameter('race_mode', False)
        self.race_mode = bool(self.get_parameter('race_mode').value)
        self.race_status = {}
        self.last_race_at = 0.0
        self.image_received_at = {}
        self.last_motion_at = 0.0
        self.last_lane_at = 0.0
        self.fullscreen = bool(self.get_parameter('fullscreen').value)
        self.display_fps = max(1.0, float(self.get_parameter('display_fps').value))
        self.screen_index = max(0, int(self.get_parameter('screen_index').value))
        topic_prefix = str(self.get_parameter('topic_prefix').value).strip().rstrip('/')
        if topic_prefix and not topic_prefix.startswith('/'):
            topic_prefix = '/' + topic_prefix
        self.topic_prefix = topic_prefix
        self.motion_topic = str(self.get_parameter('motion_topic').value)

        def prefixed(topic: str) -> str:
            return f'{self.topic_prefix}{topic}' if self.topic_prefix else topic

        self.bridge = CvBridge()
        self.track_overlay = TrackBoundaryOverlay()
        self.images: Dict[str, Tuple[int, QImage]] = {}
        self.frame_notifier = FrameNotifier()
        self.overview_converter = LatestFrameConverter(display_image, self.frame_notifier.ready.emit)
        self.overview_sequence = 0
        self.image_sequence: Dict[str, int] = {}
        self.last_conversion: Dict[str, float] = {}
        self.last_live_time = 0.0
        self.perception_times = deque(maxlen=60)

        self.lane_num = 0
        self.steering_angle = 0
        self.position_error = 0
        self.motion = MotionCommand()
        self.traffic_color = 'none'
        self.target_lane = 2
        self.lane_change_active = False
        self.obstacle_states = [False, False, False]
        self.obstacle_pending = [False, False, False]
        self.auto_recovery_enabled = False
        self.recovery_active = False
        self.recovery_count = 0
        self.demo_preparing = False
        self.command_status = '명령을 선택해 주세요'

        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        topics = {
            'camera': prefixed('/cam0/image_raw'),
            'segmentation': prefixed('/cam0/seg_vis'),
            'bev': prefixed('/cam0/bev_viz'),
            'fitting': prefixed('/cam0/lane_viz'),
            'overview': '/top_camera/image_raw',
        }
        if self.race_mode:
            # The companion monitor shows the whole track and perception stages.
            # The forward camera belongs to the driver monitor alone.
            topics = {key: topic for key, topic in topics.items() if key != 'camera'}
            topics.update({'ai_' + key: '/ai' + topic for key, topic in list(topics.items()) if key != 'overview'})
        self._image_subscriptions = []
        for key, topic in topics.items():
            self._image_subscriptions.append(
                self.create_subscription(
                    Image,
                    topic,
                    lambda msg, image_key=key: self._on_image(image_key, msg),
                    image_qos,
                )
            )

        self.create_subscription(
            LaneInfo, prefixed('/cam0/lane_info'), self._on_lane, 10
        )
        self.create_subscription(
            MotionCommand, self.motion_topic, self._on_motion, 10
        )
        self.create_subscription(String, '/traffic_light_result', self._on_traffic, 10)
        self.command_pub = self.create_publisher(
            String, '/exhibition/command', 10
        )
        state_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(String, '/race/status', self._on_race_status, state_qos)
        self.race_command_pub = self.create_publisher(String, '/race/command', 10)
        self.create_subscription(
            Int32, '/target_lane', self._on_target_lane, state_qos
        )
        self.create_subscription(
            Bool, '/lane_change_active', self._on_lane_change, state_qos
        )
        self.create_subscription(
            String,
            '/exhibition/obstacle_state',
            self._on_obstacle_state,
            state_qos,
        )

    def _on_image(self, key: str, msg: Image) -> None:
        if key == 'overview' and self.race_mode:
            self.overview_converter.submit(msg)
            return
        now = time.monotonic()
        if key == 'segmentation':
            # Measure the pipeline message rate, independently of how often
            # this GUI converts frames for display.
            self.perception_times.append(now)

        target_fps = min(self.display_fps, 10.0) if key == 'overview' else self.display_fps
        if now - self.last_conversion.get(key, 0.0) < 1.0 / target_fps:
            return
        self.last_conversion[key] = now

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if key == 'overview' and not self.race_mode:
                bgr = self.track_overlay.draw(bgr)
            if key != 'overview':
                # Display-only unsharp mask: improves definition on the four
                # exhibition cards without increasing camera/YOLO resolution
                # or adding latency to the driving pipeline.
                softened = cv2.GaussianBlur(bgr, (0, 0), 0.7)
                bgr = cv2.addWeighted(bgr, 1.30, softened, -0.30, 0)
            height, width = bgr.shape[:2]
            image = QImage(
                bgr.data,
                width,
                height,
                int(bgr.strides[0]),
                QImage.Format_BGR888,
            ).copy()
        except Exception as exc:
            self.get_logger().warn(f'Image conversion failed for {key}: {exc}')
            return

        sequence = self.image_sequence.get(key, 0) + 1
        self.image_sequence[key] = sequence
        self.images[key] = (sequence, image)
        self.image_received_at[key] = now
        self.last_live_time = now

    def collect_overview_frame(self):
        latest = self.overview_converter.latest
        if latest and latest[0] != self.overview_sequence:
            self.overview_sequence, image, received = latest
            self.images['overview'] = (self.overview_sequence, image)
            self.image_received_at['overview'] = received
            self.last_live_time = received

    def _on_race_status(self, msg):
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                self.race_status = payload
                self.last_race_at = time.monotonic()
        except (ValueError, TypeError):
            return

    def _on_lane(self, msg: LaneInfo) -> None:
        self.last_lane_at = time.monotonic()
        self.lane_num = int(msg.lane_num)
        self.steering_angle = int(msg.steering_angle)
        self.position_error = int(msg.vehicle_position_x)

    def _on_motion(self, msg: MotionCommand) -> None:
        self.last_motion_at = time.monotonic()
        self.motion = msg

    def _on_traffic(self, msg: String) -> None:
        match = re.search(r'Color:\s*(\w+)', msg.data)
        if match:
            self.traffic_color = match.group(1).lower()

    def _on_target_lane(self, msg: Int32) -> None:
        lane = int(msg.data)
        if lane in (1, 2):
            self.target_lane = lane

    def _on_lane_change(self, msg: Bool) -> None:
        self.lane_change_active = bool(msg.data)

    def _on_obstacle_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            states = list(payload.get('obstacles', []))
            pending = list(payload.get('pending', []))
            if len(states) == 3:
                self.obstacle_states = [bool(value) for value in states]
            if len(pending) == 3:
                self.obstacle_pending = [bool(value) for value in pending]
            self.auto_recovery_enabled = bool(
                payload.get('auto_recovery', self.auto_recovery_enabled)
            )
            self.recovery_active = bool(payload.get('recovery_active', False))
            self.recovery_count = int(
                payload.get('recovery_count', self.recovery_count)
            )
            self.demo_preparing = bool(payload.get('demo_preparing', False))
            message = str(payload.get('message', '')).strip()
            if message:
                self.command_status = message
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f'Invalid obstacle state: {exc}')

    def send_command(self, command: str) -> None:
        if self.race_mode:
            return
        self.command_pub.publish(String(data=command))
        if command == 'lane_toggle':
            next_lane = 1 if self.target_lane == 2 else 2
            self.command_status = f'{next_lane}차선 변경 명령 전송'
        elif command.startswith('obstacle_toggle_'):
            try:
                index = int(command.rsplit('_', 1)[-1])
                self.command_status = f'{index}번 장애물 전환 명령 전송'
            except ValueError:
                pass
        elif command == 'demo_start':
            self.command_status = '전시 데모를 처음부터 준비하고 있습니다'

    @property
    def perception_fps(self) -> float:
        if len(self.perception_times) < 2 or time.monotonic() - self.perception_times[-1] > 2.0:
            return 0.0
        elapsed = self.perception_times[-1] - self.perception_times[0]
        return (len(self.perception_times) - 1) / elapsed if elapsed > 0 else 0.0

    @property
    def online(self) -> bool:
        now = time.monotonic()
        return (all(now - self.image_received_at.get(key, 0.0) < 2.0
                    for key in ('camera', 'segmentation', 'bev', 'fitting'))
                and now - self.last_motion_at < 2.0 and now - self.last_lane_at < 2.0)


class DashboardWindow(QWidget):
    def __init__(self, ros_node: RosDashboardNode):
        super().__init__()
        self.ros_node = ros_node
        self._last_sequences: Dict[str, int] = {}
        self.setWindowTitle('자율주행이 움직이는 과정')
        self.setMinimumSize(1280, 720)
        self.setObjectName('dashboard')
        self.setStyleSheet(
            f'QWidget#dashboard {{ background: {BG}; color: {TEXT}; }}'
            f'QWidget#dashboard QLabel {{ background: transparent; color: {TEXT}; }}'
        )

        root = QVBoxLayout(self)
        root.setContentsMargins(18, 14, 18, 16)
        root.setSpacing(12)
        root.addLayout(self._build_header())

        workflow = QHBoxLayout()
        workflow.setSpacing(10)
        perception = self._build_perception_card()
        decision = self._build_decision_card()
        control = self._build_control_card()
        workflow.addWidget(perception, 58)
        workflow.addWidget(FlowArrow(VIOLET, 26), 0)
        workflow.addWidget(decision, 18)
        workflow.addWidget(FlowArrow(TEAL, 26), 0)
        workflow.addWidget(control, 24)
        root.addLayout(workflow, 62)
        root.addWidget(self._build_overview_card(), 38)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh)
        self.refresh_timer.start(80)

    def _build_header(self) -> QHBoxLayout:
        layout = QHBoxLayout()
        title = QLabel('자율주행이 움직이는 과정')
        title.setFont(QFont('Noto Sans CJK KR', 25, QFont.Bold))
        subtitle = QLabel('영상이 조향 명령으로 바뀌는 전 과정을 실시간으로 보여드립니다')
        subtitle.setStyleSheet(f'color: {MUTED};')
        subtitle.setFont(QFont('Noto Sans CJK KR', 10))

        title_box = QVBoxLayout()
        title_box.setSpacing(0)
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        layout.addLayout(title_box)
        layout.addStretch(1)

        self.fps_pill = StatusPill('인지 -- FPS', BLUE)
        self.online_pill = StatusPill('● 연결 대기', RED)
        self.live_pill = StatusPill('● LIVE', RED)
        layout.addWidget(self.fps_pill)
        layout.addWidget(self.online_pill)
        layout.addWidget(self.live_pill)
        return layout

    def _section_heading(self, number: str, title: str, color: str, subtitle: str) -> QVBoxLayout:
        outer = QVBoxLayout()
        row = QHBoxLayout()
        badge = QLabel(number)
        badge.setAlignment(Qt.AlignCenter)
        badge.setFixedSize(42, 42)
        badge.setFont(QFont('Noto Sans CJK KR', 18, QFont.Bold))
        badge.setStyleSheet(
            f'background: #0a2849; color: {TEXT}; border: 2px solid {color}; '
            'border-radius: 21px;'
        )
        label = QLabel(title)
        label.setFont(QFont('Noto Sans CJK KR', 20, QFont.Bold))
        row.addWidget(badge)
        row.addWidget(label)
        row.addStretch(1)
        outer.addLayout(row)
        sub = QLabel(subtitle)
        sub.setStyleSheet(f'color: {MUTED};')
        sub.setFont(QFont('Noto Sans CJK KR', 9))
        sub.setWordWrap(True)
        outer.addWidget(sub)
        return outer

    def _build_perception_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName('card')
        card.setStyleSheet(panel_style(BLUE))
        layout = QVBoxLayout(card)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(8)
        layout.addLayout(
            self._section_heading(
                '1',
                '인지',
                BLUE,
                '영상의 픽셀을 주행 가능한 차선 정보로 바꿉니다',
            )
        )

        stage_row = QHBoxLayout()
        stage_row.setSpacing(5)
        self.stage_canvases: Dict[str, ImageCanvas] = {}
        stages = [
            ('camera', '카메라 입력'),
            ('segmentation', 'Segmentation'),
            ('bev', 'BEV·후처리'),
            ('fitting', '다항식 피팅'),
        ]
        for index, (key, title) in enumerate(stages):
            stage = StageImage(str(index + 1), title, title, BLUE)
            self.stage_canvases[key] = stage.canvas
            stage_row.addWidget(stage, 1)
            if index < len(stages) - 1:
                stage_row.addWidget(FlowArrow(BLUE, 22))
        layout.addLayout(stage_row, 1)

        output = QFrame()
        output.setStyleSheet(
            'background: #0a2543; border: 1px solid #276aa7; border-radius: 11px;'
        )
        output_row = QHBoxLayout(output)
        output_row.setContentsMargins(12, 5, 12, 5)
        title = QLabel('차선 정보 출력')
        title.setFont(QFont('Noto Sans CJK KR', 10, QFont.Bold))
        self.lane_value = QLabel('차선 --')
        self.angle_value = QLabel('각도 --°')
        self.error_value = QLabel('중심 오차 -- px')
        self.signal_value = QLabel('신호등 --')
        for widget in (self.lane_value, self.angle_value, self.error_value, self.signal_value):
            widget.setAlignment(Qt.AlignCenter)
            widget.setFont(QFont('Noto Sans CJK KR', 10, QFont.Bold))
            widget.setStyleSheet(f'color: {TEXT}; border: none;')
        output_row.addWidget(title)
        output_row.addStretch(1)
        output_row.addWidget(self.lane_value)
        output_row.addWidget(self.angle_value)
        output_row.addWidget(self.error_value)
        output_row.addWidget(self.signal_value)
        layout.addWidget(output)
        return card

    def _build_decision_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName('card')
        card.setStyleSheet(panel_style(VIOLET))
        layout = QVBoxLayout(card)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.addLayout(
            self._section_heading(
                '2', '판단', VIOLET, '차선 정보로 주행 방향을 결정합니다'
            )
        )
        self.decision_graphic = DecisionGraphic()
        layout.addWidget(self.decision_graphic, 1)
        state = QLabel('차선 유지')
        state.setAlignment(Qt.AlignCenter)
        state.setFont(QFont('Noto Sans CJK KR', 17, QFont.Bold))
        state.setStyleSheet(
            f'background: #231443; color: {TEXT}; border: 1px solid {VIOLET}; '
            'border-radius: 11px; padding: 7px;'
        )
        layout.addWidget(state)
        return card

    def _build_control_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName('card')
        card.setStyleSheet(panel_style(TEAL))
        layout = QVBoxLayout(card)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.addLayout(
            self._section_heading(
                '3', '제어', TEAL, '조향과 속도를 차량에 전달합니다'
            )
        )
        self.control_graphic = ControlGraphic()
        layout.addWidget(self.control_graphic, 1)
        return card

    def _build_overview_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName('card')
        card.setStyleSheet(panel_style('#1f6976'))
        layout = QVBoxLayout(card)
        layout.setContentsMargins(10, 8, 10, 10)
        layout.setSpacing(5)
        row = QHBoxLayout()
        title = QLabel('실시간 주행 위치')
        title.setFont(QFont('Noto Sans CJK KR', 16, QFont.Bold))
        hint = QLabel('ESC: 종료   F11: 전체화면 전환')
        hint.setStyleSheet(f'color: {MUTED};')
        hint.setFont(QFont('Noto Sans CJK KR', 8))
        row.addWidget(title)
        row.addStretch(1)
        row.addWidget(hint)
        self.overview_canvas = ImageCanvas('상단 카메라 연결 대기')
        layout.addLayout(row)
        layout.addWidget(self.overview_canvas, 1)
        return card

    def refresh(self) -> None:
        for key, canvas in self.stage_canvases.items():
            frame = self.ros_node.images.get(key)
            if frame is not None:
                canvas.set_frame(*frame)

        overview = self.ros_node.images.get('overview')
        if overview is not None:
            self.overview_canvas.set_frame(*overview)

        lane_num = self.ros_node.lane_num
        self.lane_value.setText(f'차선 {lane_num}' if lane_num else '차선 --')
        self.angle_value.setText(f'각도 {self.ros_node.steering_angle:+d}°')
        self.error_value.setText(f'중심 오차 {self.ros_node.position_error:+d} px')
        color_text = {
            'red': '빨강',
            'yellow': '노랑',
            'green': '초록',
            'none': '없음',
            'unknown': '미확인',
        }.get(self.ros_node.traffic_color, self.ros_node.traffic_color)
        self.signal_value.setText(f'신호등 {color_text}')

        self.decision_graphic.set_values(lane_num, self.ros_node.position_error)
        motion = self.ros_node.motion
        self.control_graphic.set_values(
            int(motion.steering), int(motion.left_speed), int(motion.right_speed)
        )

        fps = self.ros_node.perception_fps
        self.fps_pill.set_state(f'인지 {fps:.1f} FPS', BLUE)
        if self.ros_node.online:
            self.online_pill.set_state('● SYSTEM ONLINE', GREEN)
            self.live_pill.set_state('● LIVE', RED)
        else:
            self.online_pill.set_state('● 연결 대기', RED)
            self.live_pill.set_state('○ STANDBY', MUTED)

    def keyPressEvent(self, event) -> None:
        if event.key() in (Qt.Key_Escape, Qt.Key_Q):
            self.close()
            return
        if event.key() == Qt.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
            return
        super().keyPressEvent(event)


class ExhibitionWindow(QWidget):
    """Visitor-facing horizontal dashboard with no artificial decision step."""

    def __init__(self, ros_node: RosDashboardNode):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle('AI 자율주행 실시간 분석')
        self.setMinimumSize(1280, 720)
        self.setObjectName('dashboardV2')
        self.setStyleSheet(
            f'QWidget#dashboardV2 {{ background: {BG}; color: {TEXT}; }}'
            f'QWidget#dashboardV2 QLabel {{ background: transparent; color: {TEXT}; }}'
        )

        root = QVBoxLayout(self)
        root.setContentsMargins(20, 14, 20, 16)
        root.setSpacing(12)
        root.addLayout(self._build_header())

        pipeline = self._build_pipeline()
        screens = QApplication.screens()
        screen_height = screens[min(ros_node.screen_index, len(screens) - 1)].availableGeometry().height()
        pipeline.setFixedHeight(max(205, min(285, int(screen_height * 0.25))))
        root.addWidget(pipeline)
        root.addWidget(self._build_data_strip())

        bottom = QHBoxLayout()
        bottom.setSpacing(12)
        bottom.addWidget(self._build_command_card(), 22)
        bottom.addWidget(self._build_overview_card(), 54)
        bottom.addWidget(self._build_steering_card(), 24)
        root.addLayout(bottom, 1)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh)
        self.refresh_timer.start(50)

    def _build_header(self) -> QHBoxLayout:
        layout = QHBoxLayout()
        title = QLabel('AI 자율주행 실시간 분석')
        title.setFont(QFont('Noto Sans CJK KR', 25, QFont.Bold))
        subtitle = QLabel('도로를 보고, 차선을 이해하고, 조향하는 흐름을 실시간으로 보여드립니다')
        subtitle.setStyleSheet(f'color: {MUTED};')
        subtitle.setFont(QFont('Noto Sans CJK KR', 10))

        title_box = QVBoxLayout()
        title_box.setSpacing(0)
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        layout.addLayout(title_box)
        layout.addStretch(1)

        self.fps_pill = StatusPill('PIPELINE -- FPS', BLUE)
        self.online_pill = StatusPill('● 연결 대기', RED)
        self.live_pill = StatusPill('● LIVE', RED)
        layout.addWidget(self.fps_pill)
        layout.addWidget(self.online_pill)
        layout.addWidget(self.live_pill)
        return layout

    def _build_pipeline(self) -> QWidget:
        container = QWidget()
        row = QHBoxLayout(container)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(7)
        self.stage_canvases: Dict[str, ImageCanvas] = {}
        stages = [
            ('camera', '01', '카메라 입력', '실제 도로 영상', BLUE, 'plain', 84),
            (
                'segmentation', '02', '도로 요소 분리', 'SEGMENTATION',
                BLUE, 'plain', 88,
            ),
            (
                'bev', '03', '도로를 위에서 보기', 'BEV · 노이즈 제거',
                TEAL, 'bev', 112,
            ),
            (
                'fitting', '04', '차선 형태 정리', '흰 검출점 → 초록 중앙선',
                CORAL, 'fitting', 116,
            ),
        ]
        for index, (
            key, step, title, technical, color, display_mode, stretch
        ) in enumerate(stages):
            stage = StageImage(
                step,
                title,
                technical,
                color,
                display_mode=display_mode,
                emphasis=key in ('bev', 'fitting'),
            )
            self.stage_canvases[key] = stage.canvas
            row.addWidget(stage, stretch)
            if index < len(stages) - 1:
                next_color = stages[index + 1][4]
                row.addWidget(FlowArrow(next_color, 20))
        return container

    def _build_data_strip(self) -> QFrame:
        strip = QFrame()
        strip.setFixedHeight(62)
        strip.setStyleSheet(
            'background: #0a1c31; border: 1px solid #2678b9; border-radius: 12px;'
        )
        layout = QHBoxLayout(strip)
        layout.setContentsMargins(22, 5, 22, 5)

        self.drive_state_value = QLabel('주행 상태  준비 중')
        self.lane_value = QLabel('현재 --  →  목표 2차선')
        self.angle_value = QLabel('차선 중심  -- px')
        self.recovery_value = QLabel('자동 복구  대기 중')
        for index, widget in enumerate(
            (
                self.drive_state_value,
                self.lane_value,
                self.angle_value,
                self.recovery_value,
            )
        ):
            widget.setAlignment(Qt.AlignCenter)
            widget.setFont(QFont('Noto Sans CJK KR', 13, QFont.Bold))
            widget.setStyleSheet(f'color: {TEAL}; border: none;')
            layout.addWidget(widget, 1)
            if index < 3:
                divider = QFrame()
                divider.setFixedWidth(1)
                divider.setStyleSheet('background: #28577c; border: none;')
                layout.addWidget(divider)
        return strip

    def _build_overview_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName('card')
        card.setStyleSheet(panel_style('#1f6976'))
        layout = QVBoxLayout(card)
        layout.setContentsMargins(10, 8, 10, 10)
        layout.setSpacing(5)
        row = QHBoxLayout()
        title = QLabel('실시간 주행 경로')
        title.setFont(QFont('Noto Sans CJK KR', 16, QFont.Bold))
        hint = QLabel('ESC: 종료   F11: 전체화면 전환')
        hint.setStyleSheet(f'color: {MUTED};')
        hint.setFont(QFont('Noto Sans CJK KR', 8))
        row.addWidget(title)
        row.addStretch(1)
        row.addWidget(hint)
        self.overview_canvas = ImageCanvas('주행 위치 연결 대기')
        layout.addLayout(row)
        layout.addWidget(self.overview_canvas, 1)
        return card

    def _build_command_card(self) -> QFrame:
        if self.ros_node.race_mode:
            return self._build_experience_card()
        card = QFrame()
        card.setObjectName('card')
        card.setStyleSheet(panel_style(BLUE))
        layout = QVBoxLayout(card)
        layout.setContentsMargins(11, 9, 11, 10)
        layout.setSpacing(7)

        title = QLabel('전시 명령')
        title.setFont(QFont('Noto Sans CJK KR', 17, QFont.Bold))
        subtitle = QLabel('버튼 클릭 또는 표시된 키를 누르세요')
        subtitle.setStyleSheet(f'color: {MUTED};')
        subtitle.setFont(QFont('Noto Sans CJK KR', 8))
        layout.addWidget(title)
        layout.addWidget(subtitle)

        self.command_buttons = {}
        commands = (
            ('demo', 'demo_start'),
            ('lane', 'lane_toggle'),
            ('obstacle_1', 'obstacle_toggle_1'),
            ('obstacle_2', 'obstacle_toggle_2'),
            ('obstacle_3', 'obstacle_toggle_3'),
        )
        for key, command in commands:
            button = QPushButton()
            button.setCursor(Qt.PointingHandCursor)
            button.setMinimumHeight(48 if key == 'demo' else 51)
            button.setFont(QFont('Noto Sans CJK KR', 10, QFont.Bold))
            button.clicked.connect(
                lambda _checked=False, value=command: self._send_command(value)
            )
            self.command_buttons[key] = button
            layout.addWidget(button)

        layout.addStretch(1)
        self.command_status = QLabel('명령을 선택해 주세요')
        self.command_status.setWordWrap(True)
        self.command_status.setAlignment(Qt.AlignCenter)
        self.command_status.setFont(QFont('Noto Sans CJK KR', 8, QFont.Bold))
        self.command_status.setStyleSheet(
            f'color: {MUTED}; background: #071727; border: 1px solid #24435f; '
            'border-radius: 8px; padding: 6px;'
        )
        layout.addWidget(self.command_status)
        self._refresh_command_card(force=True)
        return card

    def _build_experience_card(self):
        card = QFrame()
        card.setObjectName('card')
        card.setStyleSheet(panel_style(TEAL))
        layout = QVBoxLayout(card)
        layout.setContentsMargins(14, 12, 14, 12)
        self.experience_title = QLabel('5분 체험 · 준비 중')
        self.experience_title.setWordWrap(True)
        self.experience_title.setFont(QFont('Noto Sans CJK KR', 18, QFont.Bold))
        self.experience_timer = QLabel()
        self.experience_timer.setFont(QFont('Noto Sans CJK KR', 24, QFont.Bold))
        self.experience_timer.setStyleSheet(f'color: {TEAL};')
        self.experience_message = QLabel()
        self.experience_message.setWordWrap(True)
        self.experience_message.setFont(QFont('Noto Sans CJK KR', 12))
        self.experience_results = QLabel()
        self.experience_results.setWordWrap(True)
        self.experience_results.setFont(QFont('Noto Sans CJK KR', 12, QFont.Bold))
        for widget in (self.experience_title, self.experience_timer, self.experience_message,
                       self.experience_results):
            layout.addWidget(widget)
        layout.addStretch(1)
        hint = QLabel('AI는 카메라로 도로를 인식합니다.\n게임의 고스트는 기록된 경로를 재생합니다.')
        hint.setWordWrap(True)
        hint.setStyleSheet(f'color: {MUTED};')
        hint.setFont(QFont('Noto Sans CJK KR', 10))
        layout.addWidget(hint)
        return card

    def _refresh_experience(self):
        if not self.ros_node.race_mode:
            return
        from exhibition_gui_pkg.experience import result_text
        status = self.ros_node.race_status
        session = status.get('session') or {}
        if time.monotonic() - self.ros_node.last_race_at > 2:
            self.experience_title.setText('체험 화면 연결 대기')
            self.experience_timer.setText('--:--')
            self.experience_message.setText('운전 화면이 준비되면 함께 시작합니다')
            return
        self.experience_title.setText(session.get('label', '체험 준비'))
        remaining = int(math.ceil(session.get('remaining_sec', 0)))
        self.experience_timer.setText(f'{remaining // 60:02d}:{remaining % 60:02d}')
        self.experience_message.setText(status.get('message', ''))
        phase = session.get('phase', 'IDLE')
        tasks = status.get('practice_tasks', [])
        if phase == 'PRACTICE':
            content = '\n'.join(('✓ ' if name in tasks else '○ ') + name
                                for name in ('왼쪽 조향', '오른쪽 조향', '가속', '제동'))
        elif phase.startswith('CHALLENGE'):
            content = (f"참가자 {status.get('player_progress_percent', 0):.0f}%\n"
                       f"AI 고스트 {status.get('ai_progress_percent', 0):.0f}%\n"
                       f"이탈 후 복귀 {status.get('offtracks', 0)}회\n\n"
                       + result_text(session.get('results', [])))
        else:
            content = result_text(session.get('results', [])) if phase in ('RESULTS', 'RESET') else (
                '카메라 → 도로 요소 분리\n차선 형태 → 조향 명령' if phase == 'DEMO'
                else 'AI 관찰 → 연습\n도전 1 → 도전 2 → 결과'
            )
        self.experience_results.setText(content)
        self.drive_state_value.setText('체험 단계   ' + session.get('label', '준비'))
        driver = 'AI 자율주행' if phase == 'DEMO' else (
            '참가자 직접 조작' if phase == 'PRACTICE' or phase.startswith('CHALLENGE') else '정지 / 준비'
        )
        self.recovery_value.setText(driver)

    @staticmethod
    def _command_button_style(color: str, active: bool = False) -> str:
        background = '#17324a' if active else '#0b2138'
        return (
            f'QPushButton {{ color: {TEXT}; background: {background}; '
            f'border: 1px solid {color}; border-radius: 10px; '
            'padding: 5px 9px; text-align: left; }'
            f'QPushButton:hover {{ background: #173b59; border: 2px solid {color}; }}'
            'QPushButton:pressed { background: #071522; }'
            'QPushButton:disabled { color: #71849a; border-color: #34485c; }'
        )

    def _send_command(self, command: str) -> None:
        self.ros_node.send_command(command)
        self._refresh_command_card(force=True)

    def _refresh_command_card(self, force: bool = False) -> None:
        if not hasattr(self, 'command_buttons'):
            return
        target = self.ros_node.target_lane
        next_lane = 1 if target == 2 else 2
        snapshot = (
            target,
            self.ros_node.lane_change_active,
            tuple(self.ros_node.obstacle_states),
            tuple(self.ros_node.obstacle_pending),
            self.ros_node.recovery_active,
            self.ros_node.recovery_count,
            self.ros_node.demo_preparing,
            self.ros_node.command_status,
        )
        if not force and snapshot == getattr(self, '_command_snapshot', None):
            return
        self._command_snapshot = snapshot

        demo_button = self.command_buttons['demo']
        if self.ros_node.demo_preparing:
            demo_button.setText('D   전시 데모 준비 중\n     차량과 장애물을 초기화합니다')
        else:
            demo_button.setText('D   전시 데모 처음부터\n     차량 초기화 · 장애물 3개 ON')
        demo_button.setEnabled(not self.ros_node.demo_preparing)
        demo_button.setStyleSheet(
            self._command_button_style(
                GREEN if not self.ros_node.demo_preparing else MUTED,
                active=self.ros_node.demo_preparing,
            )
        )

        lane_button = self.command_buttons['lane']
        if self.ros_node.lane_change_active:
            lane_button.setText(f'C   차선 변경 중\n     목표: {target}차선')
        else:
            lane_button.setText(
                f'C   차선 변경\n     {target}차선 → {next_lane}차선'
            )
        lane_button.setEnabled(not self.ros_node.demo_preparing)
        lane_button.setStyleSheet(self._command_button_style(TEAL))

        names = ('첫 번째', '두 번째', '세 번째')
        for index, name in enumerate(names, start=1):
            button = self.command_buttons[f'obstacle_{index}']
            pending = self.ros_node.obstacle_pending[index - 1]
            enabled = self.ros_node.obstacle_states[index - 1]
            if pending:
                state_text = '처리 중...'
                color = MUTED
            elif enabled:
                state_text = 'ON · 클릭하면 제거'
                color = CORAL
            else:
                state_text = 'OFF · 클릭하면 생성'
                color = BLUE
            button.setText(
                f'{index}   {name} 장애물\n     {state_text}'
            )
            button.setEnabled(not pending and not self.ros_node.demo_preparing)
            button.setStyleSheet(
                self._command_button_style(color, active=enabled)
            )

        self.command_status.setText(self.ros_node.command_status)

    def _build_steering_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName('card')
        card.setStyleSheet(panel_style(TEAL))
        layout = QVBoxLayout(card)
        layout.setContentsMargins(12, 9, 12, 10)
        title = QLabel('조향 명령')
        title.setFont(QFont('Noto Sans CJK KR', 17, QFont.Bold))
        subtitle = QLabel('차량에 전달되는 실제 조향각')
        subtitle.setStyleSheet(f'color: {MUTED};')
        subtitle.setFont(QFont('Noto Sans CJK KR', 9))
        layout.addWidget(title)
        layout.addWidget(subtitle)
        self.steering_graphic = SteeringGraphic()
        layout.addWidget(self.steering_graphic, 1)
        return card

    def refresh(self) -> None:
        for key, canvas in self.stage_canvases.items():
            frame = self.ros_node.images.get(key)
            if frame is not None:
                canvas.set_frame(*frame)

        overview = self.ros_node.images.get('overview')
        if overview is not None:
            self.overview_canvas.set_frame(*overview)

        lane_num = self.ros_node.lane_num
        if self.ros_node.recovery_active:
            drive_state = '자동 복구 중'
        elif self.ros_node.demo_preparing:
            drive_state = '데모 준비 중'
        elif self.ros_node.lane_change_active:
            drive_state = f'{self.ros_node.target_lane}차선으로 변경 중'
        elif self.ros_node.online and lane_num not in (1, 2):
            drive_state = '차선 확인 중'
        elif self.ros_node.online:
            drive_state = f'{self.ros_node.target_lane}차선 주행 중'
        else:
            drive_state = '연결 대기'
        self.drive_state_value.setText(f'주행 상태   {drive_state}')
        current_lane = f'{lane_num}차선' if lane_num else '--'
        self.lane_value.setText(
            f'현재 {current_lane}  →  목표 {self.ros_node.target_lane}차선'
        )
        self.angle_value.setText(
            f'차선 중심   {self.ros_node.position_error:+d} px'
        )
        if self.ros_node.recovery_active:
            recovery_text = '작동 중'
        elif self.ros_node.auto_recovery_enabled:
            recovery_text = f'ON · {self.ros_node.recovery_count}회 복구'
        else:
            recovery_text = 'OFF'
        self.recovery_value.setText(f'자동 복구   {recovery_text}')

        self.steering_graphic.set_value(int(self.ros_node.motion.steering))
        self._refresh_command_card()
        self._refresh_experience()

        fps = self.ros_node.perception_fps
        self.fps_pill.set_state(f'PIPELINE {fps:.1f} FPS', BLUE)
        if self.ros_node.online:
            self.online_pill.set_state('● SYSTEM ONLINE', GREEN)
            self.live_pill.set_state('● LIVE', RED)
        else:
            self.online_pill.set_state('● 연결 대기', RED)
            self.live_pill.set_state('○ STANDBY', MUTED)

    def keyPressEvent(self, event) -> None:
        if self.ros_node.race_mode:
            if event.key() == Qt.Key_Escape:
                self.ros_node.race_command_pub.publish(String(data='cancel'))
                return
            if event.key() == Qt.Key_Q:
                self.close()
                return
            if event.key() != Qt.Key_F11:
                return
        if event.key() in (Qt.Key_Escape, Qt.Key_Q):
            self.close()
            return
        command_keys = {
            Qt.Key_D: 'demo_start',
            Qt.Key_C: 'lane_toggle',
            Qt.Key_1: 'obstacle_toggle_1',
            Qt.Key_2: 'obstacle_toggle_2',
            Qt.Key_3: 'obstacle_toggle_3',
        }
        command = command_keys.get(event.key())
        if command is not None and not event.isAutoRepeat():
            self._send_command(command)
            return
        if event.key() == Qt.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
            return
        super().keyPressEvent(event)


def main(args=None):
    rclpy.init(args=args)
    cv2.setNumThreads(1)
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    if os.environ.get('E2E_GPU_CAMERA_DISPLAY') == '1':
        fmt = QSurfaceFormat()
        fmt.setSwapInterval(0)
        QSurfaceFormat.setDefaultFormat(fmt)
    app = QApplication(sys.argv)
    app.setApplicationName('E2E Autonomous Driving Exhibition')
    app.setFont(QFont('Noto Sans CJK KR', 10))

    node = RosDashboardNode()
    if node.race_mode:
        from exhibition_gui_pkg.visitor_ui import VisitorObserverWindow
        window = VisitorObserverWindow(node)
    else:
        window = ExhibitionWindow(node)

    from exhibition_gui_pkg.ros_worker import RosWorker
    node.frame_notifier.quit.connect(app.quit, Qt.QueuedConnection)
    ros_worker = RosWorker(node, node.frame_notifier.quit.emit)
    ros_worker.start()

    screens = app.screens()
    screen = screens[min(node.screen_index, len(screens) - 1)]
    window.winId()
    if window.windowHandle() is not None:
        window.windowHandle().setScreen(screen)
    window.setGeometry(screen.geometry())
    if node.fullscreen:
        window.showFullScreen()
    else:
        window.show()

    try:
        exit_code = app.exec_()
    finally:
        ros_worker.close()
        node.overview_converter.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
