"""Self-service exhibition screens, sharing the coordinator's actual state."""

import math
import os
import time
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from PyQt5.QtCore import QEvent, QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPainterPath, QPen, QPixmap, QPolygonF, QSurfaceFormat, QTransform
from PyQt5.QtWidgets import (
    QApplication, QFrame, QGridLayout, QHBoxLayout, QLabel, QProgressBar, QPushButton,
    QOpenGLWidget, QSizePolicy, QStackedWidget, QVBoxLayout, QWidget,
)
from std_msgs.msg import String

from exhibition_gui_pkg.dashboard import (
    BG, PANEL, TEXT, MUTED, TEAL, CORAL, GREEN, ImageCanvas, SteeringGraphic,
)
from exhibition_gui_pkg.experience import LESSONS, LESSON_GUIDES, LESSON_TERMS, PRACTICE_TASKS, practice_complete, result_text
from exhibition_gui_pkg.perception_example import PerceptionExample
from exhibition_gui_pkg.lane_center_example import CENTER_STEPS, LaneCenterExample
from exhibition_gui_pkg.duel_ui import DuelPanel
from exhibition_gui_pkg.duel import DUEL_DIFFICULTIES, RACE_OBSTACLES
from exhibition_gui_pkg.venue_ui import VenueMap, VenueCameraCanvas


FONT = 'Noto Sans CJK KR'


from exhibition_gui_pkg.camera_canvas import FrontCameraCanvas, front_camera_canvas


def label(text='', size=18, color=TEXT, bold=False):
    widget = QLabel(text)
    widget.setWordWrap(True)
    widget.setFont(QFont(FONT, size, QFont.Bold if bold else QFont.Normal))
    widget.setProperty('visitorFontSize', size)
    widget.setStyleSheet(f'color: {color}; background: transparent; border: none;')
    widget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
    return widget


def button(text, callback, primary=False):
    widget = QPushButton(text)
    widget.setFont(QFont(FONT, 17, QFont.Bold if primary else QFont.Normal))
    widget.setMinimumHeight(56)
    widget.setCursor(Qt.PointingHandCursor)
    widget.setStyleSheet(
        f'QPushButton {{ color: {BG if primary else TEXT}; '
        f'background: {TEAL if primary else PANEL}; border: 1px solid #345069; '
        'border-radius: 12px; padding: 10px 16px; }'
        'QPushButton:disabled { color: #8096ab; background: #142838; }'
        'QPushButton:focus { border: 3px solid #f0ca73; }'
        'QPushButton:hover { border: 2px solid #f0ca73; }'
    )
    widget.clicked.connect(callback)
    return widget


def quiet_button(text, callback):
    widget = button(text, callback)
    widget.setFont(QFont(FONT, 13))
    widget.setMinimumHeight(40)
    widget.setStyleSheet(
        f'QPushButton {{ color: {MUTED}; background: transparent; '
        'border: 1px solid transparent; border-radius: 8px; padding: 8px 12px; }'
        f'QPushButton:hover {{ color: {TEXT}; border-color: #345069; }}'
        'QPushButton:focus { border: 2px solid #f0ca73; }'
    )
    return widget


class JourneyBar(QWidget):
    """Read-only progress with one visibly active activity, no duplicate title."""
    PHASES = (('EXPLORE', '대회장 둘러보기'), ('INTRO', '원리 배우기'), ('DEMO', 'AI 운전 보기'),
              ('PRACTICE', '운전 연습'), ('DIFFICULTY', '난이도 고르기'),
              ('CHALLENGE', 'AI와 함께 운전'), ('RESULTS', '결과 비교'))

    def __init__(self):
        super().__init__()
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(8)
        row.addWidget(label('체험 순서', 11, MUTED))
        self.items = {}
        for index, (key, text) in enumerate(self.PHASES):
            if index:
                row.addWidget(label('›', 14, MUTED))
            item = label(text, 12)
            item.setAlignment(Qt.AlignCenter)
            item.setMinimumHeight(32)
            row.addWidget(item, 1)
            self.items[key] = item

    def set_phase(self, phase):
        if getattr(self, '_phase', None) == phase:
            return
        self._phase = phase
        for key, item in self.items.items():
            active = phase == key
            item.setStyleSheet(
                f'color: {TEAL if active else MUTED}; background: {"#123b43" if active else "transparent"}; '
                f'border: 1px solid {TEAL if active else "transparent"}; border-radius: 8px; padding: 4px;'
            )


def surface():
    widget = QFrame()
    widget.setObjectName('visitorPanel')
    widget.setStyleSheet('QFrame#visitorPanel { background: #0b1b31; border: 1px solid #233e56; border-radius: 16px; }')
    layout = QVBoxLayout(widget)
    layout.setContentsMargins(20, 18, 20, 18)
    layout.setSpacing(14)
    return widget, layout


def state_label(status):
    state = status.get('state', 'READY')
    if state == 'PAUSED':
        return '잠시 멈췄어요', CORAL
    if state in ('RESETTING', 'COUNTDOWN'):
        return '출발을 준비하고 있어요', CORAL
    if state == 'ERROR':
        return '준비 상태를 확인해 주세요', CORAL
    if state == 'DEMO':
        return 'AI가 운전 중이에요', TEAL
    if state == 'DIFFICULTY':
        return '난이도를 고르는 중이에요', TEAL
    if state in ('PRACTICE', 'RUNNING'):
        if not status.get('input_armed', False):
            return '조작을 놓으면 출발할 수 있어요', CORAL
        return '내가 운전 중이에요', GREEN
    if state == 'INTRO':
        return '원리 설명 중 · AI가 천천히 주행해요', TEAL
    if state == 'EXPLORE':
        return '대회장을 둘러보고 있어요', TEAL
    if state == 'RESULTS':
        return '체험을 돌아봐요', TEAL
    return '시작을 기다리고 있어요', TEAL


def primary_text(status):
    state = status.get('state', 'READY')
    session = status.get('session') or {}
    if state == 'PAUSED':
        return '이어서 체험하기'
    if state == 'INTRO':
        return LESSONS[session.get('lesson', 0)][3]
    if state == 'EXPLORE':
        return '차량 카메라로 보기'
    if state == 'DEMO':
        return '내가 운전하기 · 연습부터'
    if state == 'PRACTICE':
        return ('다음: 가속 연습', '다음: 브레이크 연습', 'AI와 함께 출발' if status.get('duel') else '한 바퀴 운전 시작')[session.get('practice_step', 0)]
    if state == 'DIFFICULTY':
        return '난이도 선택'
    if state == 'RESULTS':
        return '한 바퀴 다시 운전'
    return '대회장 둘러보기'


def shortcut_hint(status):
    state = status.get('state')
    session = status.get('session') or {}
    controller = status.get('input_mode') == 'controller'
    forward, back = ('A', 'B') if controller else ('Space', 'Esc')
    if state == 'EXPLORE':
        hint = '왼쪽 스틱 이동 · 오른쪽 스틱 둘러보기 · X 내 차 찾기 · A 차량 카메라' if controller else '방향키 이동 · I J K L 둘러보기 · X 내 차 찾기 · Space 차량 카메라'
    elif state == 'INTRO':
        next_action = 'AI 운전 시작' if session.get('lesson', 0) == len(LESSONS) - 1 else '다음 설명'
        previous = '대회장 다시 보기' if session.get('lesson', 0) == 0 else '이전 설명'
        hint = f'{forward} {next_action}   ·   {back} {previous}'
        if controller and session.get('lesson') == 3:
            hint = '방향패드 ← → 만드는 순서   ·   ' + hint
    elif state == 'DEMO':
        hint = f'1·2·3 장애물   ·   C 차선 변경   ·   {forward} 운전 연습'
        if controller: hint = '방향패드 ← ↑ → 장애물 1·2·3   ·   X 차선 변경   ·   A 운전 연습'
    elif state in ('PRACTICE', 'RUNNING'):
        controls = '스틱 핸들 · RT 가속 · LT 브레이크' if controller else '← → 핸들 · ↑ 가속 · ↓ 브레이크'
        hint = controls
        if state == 'RUNNING' and status.get('duel'):
            hint = '왼쪽 스틱 핸들 · RT 가속 · LT 브레이크' if controller else '← → 핸들 · ↑ 가속 · ↓ 브레이크'
        elif state == 'PRACTICE' and controller:
            hint += '   ·   Y 연습 건너뛰기'
    elif state == 'DIFFICULTY':
        hint = '방향패드 ← → 난이도 · A 선택 · B 연습으로' if controller else '난이도를 클릭해 선택하세요 · Esc 연습으로'
    elif state == 'PAUSED':
        finish = '결과 보기' if session.get('phase') == 'CHALLENGE' else '시작 화면'
        hint = f'{forward} 이어서 운전   ·   {back} {finish}'
    elif state == 'RESULTS':
        hint = f'{forward} 한 바퀴 다시 운전   ·   답은 마우스로 선택' if controller else '1 · 2 답 선택   ·   Space 한 바퀴 다시 운전'
        if status.get('duel'):
            hint = f'{forward} 같은 코스로 다시 도전'
            if controller: hint += '   ·   X 운전 연습   ·   B 시작 화면'
    elif state in ('RESETTING', 'COUNTDOWN', 'ERROR'):
        hint = '화면의 안내를 확인해 주세요'
    else:
        hint = f'{forward} 시작   ·   화면 버튼은 마우스로 선택'
    return hint + '   ·   Q 프로그램 종료'


def neutral_text(status):
    return ('스틱과 RT·LT를 모두 놓아 주세요.' if status.get('input_mode') == 'controller'
            else '누르고 있는 방향키를 모두 놓아 주세요.')


def direction_text(status):
    if not status.get('autonomy_fresh') or not status.get('lane_valid'):
        return '차선을 다시 확인해요', '차선 정보가 없으면 AI는 차를 멈춰요.'
    value = status.get('autonomy_steering', 0)
    direction = '왼쪽으로' if value < -1 else ('오른쪽으로' if value > 1 else '거의 곧게')
    applied = status.get('state') in ('INTRO', 'DEMO')
    return direction, ('AI가 이 방향으로 핸들을 움직이고 있어요.' if applied
                       else 'AI가 계산한 방향이에요. 지금 차량에 적용되는 명령은 아니에요.')


def lane_explanation(status):
    geometry = status.get('lane_geometry') or {}
    if geometry.get('reason') == 'acquiring_target':
        return '차선을 옮기며 목표 차선을 찾고 있어요.\n확인된 구간부터 가운데를 표시해요.'
    if geometry and not geometry.get('control_valid', True) and geometry.get('quality') != 'unavailable':
        return '가까운 차선 정보가 부족하면 AI는 멈춰요.\n보이는 구간만 가운데를 표시해요.'
    if not geometry or geometry.get('quality') == 'unavailable' or not status.get('lesson_fresh'):
        return '차선의 가운데를 다시 확인하고 있어요.\n정보가 부족하면 AI는 차를 멈춰요.'
    if geometry.get('changing_lane'):
        return '초록 선: 옮겨 갈 차선의 가운데\n점선 구간: 가려진 경계를 보완했어요.'
    if geometry.get('quality') == 'inferred':
        return '흰 점: 달릴 공간의 양쪽 끝\n초록 점선: 차선 너비로 보완한 가운데'
    return '흰 점: 달릴 공간의 양쪽 끝\n초록 실선: 두 끝 사이의 가운데'


class FriendlyWheel(SteeringGraphic):
    VISUAL_RATIO = 5.0

    def __init__(self):
        super().__init__()
        self.setMinimumSize(110, 110)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)
        size = min(self.width(), self.height()) * .78
        painter.translate(self.width() / 2, self.height() / 2)
        painter.rotate(self.display_deg * self.VISUAL_RATIO)
        painter.drawPixmap(QRectF(-size / 2, -size / 2, size, size),
                           self.wheel_pixmap, QRectF(self.wheel_pixmap.rect()))


class TrackOverviewCanvas(FrontCameraCanvas if os.environ.get('E2E_GPU_CAMERA_DISPLAY') == '1' else ImageCanvas):
    """The real overhead camera, with the participant vehicle's world pose."""
    def __init__(self):
        super().__init__('트랙 전체 영상을 연결하고 있어요')
        self.pose = None
        self.projection = self.load_projection()

    @staticmethod
    @lru_cache(maxsize=1)
    def load_projection():
        """Use the actual camera span and body center, not the rear-origin pose.

        The track texture's 16 x 12 m dimensions are not the camera footprint.
        The vehicle origin is at its rear, so projecting that point would put
        the marker behind the visible car, especially near the image edges.
        """
        try:
            assets = Path(get_package_share_directory('simulation_pkg'))
            camera = ET.parse(assets / 'worlds/track.world').find(".//model[@name='top_camera']")
            camera_pose = tuple(map(float, camera.findtext('pose').split()))
            fov = float(camera.findtext('.//horizontal_fov'))
            body = ET.parse(assets / 'models/mercedes_split_tex/model.sdf').find(".//collision[@name='chassis_collision']")
            center = tuple(map(float, body.findtext('pose').split()))[:3]
            roof = center[2] + float(body.findtext('geometry/box/size').split()[2]) / 2
            span = 2 * (camera_pose[2] - roof) * math.tan(fov / 2)
            if span <= 0 or not math.isfinite(span):
                return None
            return camera_pose[:2], center[:2], span
        except (OSError, ET.ParseError, AttributeError, TypeError, ValueError, IndexError):
            return None

    def set_pose(self, pose):
        value = None
        if self.projection is not None and pose is not None and len(pose) >= 3:
            candidate = tuple(float(x) for x in pose[:3])
            if all(math.isfinite(x) for x in candidate):
                value = candidate
        if self.pose != value:
            self.pose = value
            self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        if not isinstance(self, FrontCameraCanvas):
            self.paint_annotation()

    def paintGL(self):
        super().paintGL()
        self.paint_annotation()

    def paint_annotation(self):
        if self._pixmap is None or self._pixmap.isNull() or self.pose is None:
            return
        camera_xy, body_xy, span = self.projection
        yaw = self.pose[2]
        body_x = self.pose[0] + body_xy[0] * math.cos(yaw) - body_xy[1] * math.sin(yaw) - camera_xy[0]
        body_y = self.pose[1] + body_xy[0] * math.sin(yaw) + body_xy[1] * math.cos(yaw) - camera_xy[1]
        u = .5 - body_y / span
        v = .5 - body_x / (span * self._pixmap.height() / self._pixmap.width())
        if not (0 <= u <= 1 and 0 <= v <= 1):
            return
        size = self._pixmap.size().scaled(self.size(), Qt.KeepAspectRatio)
        left, top = (self.width() - size.width()) // 2, (self.height() - size.height()) // 2
        x, y = left + u * size.width(), top + v * size.height()
        radius = max(12, min(21, size.width() * .023))
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setClipRect(QRectF(left, top, size.width(), size.height()))
        painter.setPen(QPen(QColor(TEAL), 3))
        painter.setBrush(QColor('#102431'))
        painter.drawEllipse(QPointF(x, y), radius, radius)
        painter.save()
        painter.translate(x, y)
        # In the overhead view +world-x is up and +world-y is left.
        painter.rotate(-math.degrees(self.pose[2]))
        painter.setPen(QPen(QColor('#06101e'), 2))
        painter.setBrush(QColor(TEAL))
        painter.drawPolygon(QPolygonF([
            QPointF(0, -radius * 1.25), QPointF(radius * .65, radius * .65),
            QPointF(0, radius * .3), QPointF(-radius * .65, radius * .65),
        ]))
        painter.restore()
        label_x = x - radius - 60 if u > .5 else x + radius + 6
        bounds = QRectF(label_x, y - 14, 54, 28)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor('#06101e'))
        painter.drawRoundedRect(bounds, 6, 6)
        painter.setPen(QColor(TEXT))
        painter.setFont(QFont(FONT, 11, QFont.Bold))
        painter.drawText(bounds, Qt.AlignCenter, '내 차')


class DemoObstacleMap(QWidget):
    """A small location key for the three obstacle buttons, using track coordinates."""
    def __init__(self):
        super().__init__()
        self.setMinimumSize(145, 116)
        texture = Path(get_package_share_directory('simulation_pkg')) / 'models/race_track/materials/textures/track.png'
        self.track = QPixmap(str(texture)).transformed(QTransform().rotate(180))
        self.status = {}

    def refresh(self, status):
        self.status = status
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.SmoothPixmapTransform)
        width = min(self.width() - 8, (self.height() - 8) * 4 / 3)
        area = QRectF((self.width() - width) / 2, (self.height() - width * .75) / 2, width, width * .75)
        p.setOpacity(.68)
        p.drawPixmap(area, self.track, QRectF(self.track.rect()))
        p.setOpacity(1)
        controls = self.status.get('demo_controls', {})
        locations = controls.get('positions') or [(3.4, -5.1), (4.9, -.9), (3.6, 2.5)]
        enabled = controls.get('obstacles', [False] * 3)
        for index, (x, y) in enumerate(locations):
            center = QPointF(area.left() + (.5 - y / 16) * width,
                             area.top() + (.5 - x / 12) * area.height())
            p.setPen(QPen(QColor(CORAL if enabled[index] else TEXT), 2))
            p.setBrush(QColor(CORAL if enabled[index] else BG))
            p.drawEllipse(center, 11, 11)
            p.setPen(QColor(BG if enabled[index] else TEXT))
            p.setFont(QFont(FONT, 10, QFont.Bold))
            p.drawText(QRectF(center.x() - 11, center.y() - 11, 22, 22), Qt.AlignCenter, str(index + 1))
        pose = self.status.get('player_pose') if self.status.get('pose_fresh') else None
        if pose:
            x = area.left() + (.5 - pose[1] / 16) * width
            y = area.top() + (.5 - pose[0] / 12) * area.height()
            p.setPen(QPen(QColor(BG), 2))
            p.setBrush(QColor(TEAL))
            p.drawEllipse(QPointF(x, y), 6, 6)
            bounds = QRectF(x - 43 if x > area.center().x() else x + 9, y - 10, 36, 20)
            p.fillRect(bounds, QColor(BG))
            p.setPen(QColor(TEAL))
            p.setFont(QFont(FONT, 9, QFont.Bold))
            p.drawText(bounds, Qt.AlignCenter, '내 차')


class DemoControls(QWidget):
    def __init__(self, send):
        super().__init__()
        box = QVBoxLayout(self)
        box.setContentsMargins(0, 0, 0, 0)
        box.setSpacing(10)
        self.heading = label('AI에게 변화를 줘 보세요', 20, TEXT, True)
        self.question = label('장애물을 보면 어떻게 피할까요?', 15, MUTED)
        box.addWidget(self.heading)
        box.addWidget(self.question)
        box.addSpacing(4)
        box.addWidget(label('장애물 차량 놓기', 16, TEAL, True))
        locations_widget = QWidget()
        locations_widget.setMinimumHeight(156)
        locations = QHBoxLayout(locations_widget)
        locations.setContentsMargins(0, 0, 0, 0)
        self.map = DemoObstacleMap()
        locations.addWidget(self.map, 1)
        choices = QVBoxLayout()
        choices.setSpacing(6)
        self.obstacles = []
        for index in range(1, 4):
            choice = button('', lambda checked=False, i=index: send(f'obstacle_toggle_{i}'))
            choice.setFont(QFont(FONT, 13))
            choice.setMinimumHeight(42)
            choice.setCheckable(True)
            choice.setStyleSheet(choice.styleSheet() +
                f'QPushButton:checked {{ border: 2px solid {CORAL}; color: {CORAL}; background: #352836; }}')
            choices.addWidget(choice)
            self.obstacles.append(choice)
        locations.addLayout(choices, 1)
        box.addWidget(locations_widget)
        self.obstacle_hint = label('지도의 번호 위치에 놓아요. 다시 누르면 치워요.', 12, MUTED)
        box.addWidget(self.obstacle_hint)
        divider = QFrame()
        divider.setFixedHeight(1)
        divider.setStyleSheet('background: #345069;')
        box.addWidget(divider)
        box.addWidget(label('차선 바꿔 보기', 16, TEAL, True))
        self.lane = button('C  다른 차선으로 이동', lambda: send('lane_toggle'))
        self.lane.setFont(QFont(FONT, 14))
        self.lane.setMinimumHeight(48)
        box.addWidget(self.lane)
        self.lane_hint = label('', 13, MUTED)
        box.addWidget(self.lane_hint)
        self.feedback = label('', 13, TEAL)
        self.feedback.setMinimumHeight(42)
        box.addWidget(self.feedback)
        box.addStretch(1)

    def set_compact(self, compact):
        self.heading.setVisible(not compact)
        self.question.setVisible(not compact)
        self.layout().setSpacing(6 if compact else 10)
        self.feedback.setMinimumHeight(32 if compact else 42)

    def refresh(self, status):
        controls = status.get('demo_controls', {})
        ready = controls.get('ready', False)
        clearing = controls.get('clearing', False)
        active = controls.get('obstacles', [False] * 3)
        pending = controls.get('pending', [False] * 3)
        blocked = controls.get('spawn_blocked_reasons', ['', '', ''])
        for index, choice in enumerate(self.obstacles):
            reason = blocked[index] if not active[index] else ''
            action = ('치우는 중…' if active[index] else '놓는 중…') if pending[index] else ('치우기' if active[index] else '놓기')
            if reason and not pending[index]:
                action = '가까워요' if reason == 'nearby' else '대기 중'
            key = ('←', '↑', '→')[index] if status.get('input_mode') == 'controller' else str(index+1)
            choice.setText(f'{key}  {index + 1}번 {action}')
            choice.setChecked(active[index])
            choice.setEnabled(ready and not pending[index] and not clearing and not reason)
            choice.setToolTip('차가 멀어지면 놓을 수 있어요' if reason == 'nearby' else
                              '차량 위치를 확인하고 있어요' if reason else '')
            choice.setAccessibleName(f'{index + 1}번 위치 장애물 차량 {action}')
        self.map.refresh(status)
        self.obstacle_hint.setText('장애물 기능을 연결하고 있어요' if not ready else
            '이전 장애물을 치우고 있어요' if clearing else
            '차가 가까운 위치에는 놓을 수 없어요.' if any(reason == 'nearby' and not active[i] for i, reason in enumerate(blocked)) else
            '차량 위치를 확인하고 있어요.' if any(reason == 'position' and not active[i] for i, reason in enumerate(blocked)) else
            '지도의 번호 위치에 놓아요. 다시 누르면 치워요.')
        changing = controls.get('changing_lane', False)
        lane_block = controls.get('lane_blocked_reason', '')
        key = 'X' if status.get('input_mode') == 'controller' else 'C'
        self.lane.setText('차선을 옮기고 있어요…' if changing else
                          '장애물 통과 중' if lane_block == 'obstacle' else
                          '옮길 차선에 장애물' if lane_block == 'path' else
                          '차량 위치 확인 중' if lane_block == 'position' else f'{key}  다른 차선으로 이동')
        self.lane.setEnabled(controls.get('lane_enabled', False))
        target = controls.get('target_lane', 2)
        self.lane_hint.setText(f'목표: {target}차선 · 새 가운데 선을 따라가요.' if changing else
                              '옆의 장애물을 지나면 바꿀 수 있어요.' if lane_block == 'obstacle' else
                              '앞 장애물을 지나면 바꿀 수 있어요.' if lane_block == 'path' else
                              '차량 위치를 확인하면 바꿀 수 있어요.' if lane_block == 'position' else
                              f'지금은 {target}차선이에요. 누르면 옆 차선으로 가요.' if status.get('lane_valid') else
                              '차선을 다시 찾으면 이동할 수 있어요.')
        self.feedback.setText('장애물이 가까워 멈췄어요. 치우면 다시 출발해요.' if controls.get('safety_stop') else
                              controls.get('message') or '차량이 장애물에 가까워질 때 반응을 살펴보세요.')


class LearningVisual(QWidget):
    """A persistent context view beside the actual perception-to-control pipeline."""
    def __init__(self, overview=False):
        super().__init__()
        self.overview_mode = overview
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(12)
        self.camera_card, camera_layout = surface()
        camera_layout.setContentsMargins(14, 12, 14, 12)
        camera_layout.setSpacing(8)
        camera_layout.addWidget(label('트랙 전체' if overview else '① 카메라 · AI의 입력', 13 if overview else 16, TEAL, True))
        if not overview:
            camera_layout.addWidget(label('이 영상이 오른쪽 분석의 출발점이에요.', 12, MUTED))
        self.camera = TrackOverviewCanvas() if overview else ImageCanvas('카메라 영상을 준비하고 있어요')
        camera_layout.addWidget(self.camera, 1)
        self.context_caption = label('', 12, MUTED)
        camera_layout.addWidget(self.context_caption)
        if not overview:
            row.addWidget(self.camera_card, 3)
        self.analysis_panel = QWidget()
        self.analysis_layout = QVBoxLayout(self.analysis_panel)
        self.analysis_layout.setContentsMargins(0, 0, 0, 0)
        self.analysis_layout.setSpacing(8)
        self.flow = label('', 13, TEAL, True)
        self.analysis_layout.addWidget(self.flow)
        grid = QGridLayout()
        grid.setSpacing(10)
        self.analysis_layout.addLayout(grid, 1)
        row.addWidget(self.analysis_panel, 7)
        self.stage_cards = {}
        self.stage_badges = {}
        for key, number, title, caption, position in (
            ('segmentation', 2, '달릴 공간 찾기', '사진에서 AI가 찾은 달릴 공간이에요.\n파랑과 분홍은 서로 다른 주행 공간이에요.', (0, 0)),
            ('bev', 3, '위에서 본 도로', '색칠한 도로를 위에서 본 모습으로 펼쳐요.\n양쪽 끝과 가운데를 비교하기 쉬워져요.', (0, 1)),
            ('fitting', 4, '가운데 선 만들기', '', (1, 0)),
        ):
            card, layout = surface()
            card.setMinimumWidth(0)
            layout.setContentsMargins(12, 10, 12, 10)
            layout.setSpacing(6)
            header = QHBoxLayout()
            header.addWidget(label(f'{number}  {title}', 14, TEXT, True), 1)
            badge = label('', 10, TEAL)
            header.addWidget(badge)
            layout.addLayout(header)
            canvas = ImageCanvas('이 단계의 영상을 기다리고 있어요')
            canvas.setMinimumSize(120, 85)
            layout.addWidget(canvas, 1)
            description = label(caption, 12, MUTED)
            layout.addWidget(description)
            grid.addWidget(card, *position)
            self.stage_cards[key] = card
            self.stage_badges[key] = badge
            setattr(self, key, canvas)
            if key == 'fitting':
                self.lane_card, self.lane, self.lane_key = card, canvas, description
        self.direction_card, direction_layout = surface()
        direction_layout.setContentsMargins(12, 10, 12, 10)
        direction_layout.setSpacing(6)
        direction_layout.addWidget(label('5  핸들 방향 정하기', 14, TEXT, True))
        self.wheel = FriendlyWheel()
        self.wheel.setMinimumSize(85, 85)
        self.direction = label('', 20, TEAL, True)
        self.explanation = label('', 12, MUTED)
        content = QHBoxLayout()
        content.addWidget(self.wheel, 2)
        text = QVBoxLayout()
        text.setSpacing(6)
        text.addWidget(label('가운데 선의 위치와 방향을 보고 계산해요.', 12, MUTED))
        text.addWidget(self.direction)
        content.addLayout(text, 3)
        if overview:
            content.addWidget(self.camera_card, 4)
        direction_layout.addLayout(content, 1)
        direction_layout.addWidget(self.explanation)
        grid.addWidget(self.direction_card, 1, 1)
        self.stage_cards['steering'] = self.direction_card
        for index in (0, 1):
            grid.setColumnStretch(index, 1)
            grid.setRowStretch(index, 1)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # Preserve useful image area on 720p displays; 1080p keeps large type.
        scale = .83 if self.height() < 600 else 1.0
        for widget in self.findChildren(QLabel):
            size = widget.property('visitorFontSize')
            if size:
                font = widget.font()
                font.setPointSize(max(9, round(size * scale)))
                if font != widget.font():
                    widget.setFont(font)

    @staticmethod
    def frame(canvas, frame, fresh):
        if frame is not None and fresh:
            canvas.set_frame(*frame)
        elif canvas._pixmap is not None:
            canvas._pixmap = None
            canvas._sequence = -1
            canvas.update()

    def refresh(self, status, frames, lesson=4, analysis_visible=True):
        # Keep every intermediate stage visible, including while the car is
        # stopped. The visitor advances the explanation, not the live feed.
        self.flow.setText('① 카메라 → ② 달릴 공간 → ③ 위에서 본 도로 → ④ 가운데 선 → ⑤ 핸들 방향')
        focus = ('camera', 'segmentation', 'bev', 'fitting', 'steering')[min(lesson, 4)] if status.get('state') == 'INTRO' else None
        if getattr(self, '_focus', '') != focus:
            self._focus = focus
            for key, card in self.stage_cards.items():
                card.setStyleSheet('QFrame#visitorPanel { background: #0b1b31; border: '
                                  + ('2px solid #25d8d0' if key == focus else '1px solid #233e56')
                                  + '; border-radius: 16px; }')
        if self.overview_mode:
            fresh = status.get('overview_fresh', False)
            self.frame(self.camera, frames.get('overview'), fresh)
            pose = status.get('player_pose') if fresh and status.get('pose_fresh') else None
            self.camera.set_pose(pose)
            caption = '내 차 위치·방향\n체험자용 안내 지도' if self.camera.pose else '차량 위치를 확인하고 있어요.'
            self.context_caption.setText(caption if fresh else '전체 트랙 영상을 기다리고 있어요.')
        else:
            self.frame(self.camera, frames.get('camera'), status.get('camera_fresh', False))
            self.context_caption.setText('실제 전방 카메라 영상 · 분석 전의 입력')
        for key in ('segmentation', 'bev', 'fitting'):
            field = 'lesson_fresh' if key == 'fitting' else f'{key}_fresh'
            fresh = analysis_visible and status.get(field, False) and frames.get(key) is not None
            if key == 'fitting' and (status.get('lane_geometry') or {}).get('reason') == 'stale':
                fresh = False
            self.frame(getattr(self, key), frames.get(key), fresh)
            self.stage_badges[key].setText('실시간' if fresh else '연결 대기')
            self.stage_badges[key].setStyleSheet(f'color: {TEAL if fresh else CORAL};')
        geometry = status.get('lane_geometry') or {}
        self.frame(self.lane, frames.get('fitting'), analysis_visible and status.get('lesson_fresh', False) and geometry.get('reason') != 'stale')
        self.lane_key.setText(lane_explanation(status))
        heading, detail = direction_text(status)
        self.direction.setText(heading)
        self.explanation.setText(detail)
        valid = status.get('autonomy_fresh') and status.get('lane_valid')
        self.wheel.set_value(status.get('autonomy_steering', 0) if valid else 0)
        self.wheel.setVisible(bool(valid))


class MainAnalysisLesson(QWidget):
    """Build up a midpoint diagram, then explain steering."""
    changed = pyqtSignal()

    def __init__(self):
        super().__init__()
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(16)
        lane_column = QVBoxLayout()
        self.lane = ImageCanvas('차선 분석 영상을 기다리고 있어요')
        lane_column.addWidget(self.lane, 1)
        self.center_example = LaneCenterExample()
        lane_column.addWidget(self.center_example, 1)
        self.center_controls = QWidget()
        controls = QVBoxLayout(self.center_controls)
        controls.setContentsMargins(0, 0, 0, 0)
        controls.setSpacing(8)
        steps = QHBoxLayout()
        steps.setSpacing(6)
        self.center_steps = []
        self.center_previous = button('←', lambda: self.select_center_step(max(0, self.center_example.step-1)))
        self.center_previous.setFixedWidth(48)
        self.center_previous.setMinimumHeight(40)
        self.center_previous.setAccessibleName('방향패드 왼쪽 · 이전 과정')
        steps.addWidget(self.center_previous)
        for index, (name, _, _) in enumerate(CENTER_STEPS):
            control = button(name, lambda _checked=False, index=index: self.select_center_step(index))
            control.setFont(QFont(FONT, 12))
            control.setMinimumHeight(40)
            control.setCheckable(True)
            control.setAutoExclusive(True)
            control.setStyleSheet(control.styleSheet() + 'QPushButton:checked { color: #25d8d0; border: 2px solid #25d8d0; background: #123b43; }')
            steps.addWidget(control, 1)
            self.center_steps.append(control)
        self.center_next = button('→', lambda: self.select_center_step(min(3, self.center_example.step+1)))
        self.center_next.setFixedWidth(48)
        self.center_next.setMinimumHeight(40)
        self.center_next.setAccessibleName('방향패드 오른쪽 · 다음 과정')
        steps.addWidget(self.center_next)
        controls.addLayout(steps)
        lane_column.addWidget(self.center_controls)
        self.legend = label('', 14, MUTED)
        lane_column.addWidget(self.legend)
        self.legend.hide()  # Explained in the main screen's right-hand box.
        row.addLayout(lane_column, 3)
        self.decision = QWidget()
        decision_column = QVBoxLayout(self.decision)
        decision_column.setContentsMargins(12, 0, 0, 0)
        decision_column.setSpacing(8)
        self.wheel = FriendlyWheel()
        decision_column.addWidget(self.wheel, 1)
        self.direction = label('', 24, TEAL, True)
        self.direction.setAlignment(Qt.AlignCenter)
        decision_column.addWidget(self.direction)
        self.explanation = label('', 14, MUTED)
        decision_column.addWidget(self.explanation)
        self.explanation.hide()
        row.addWidget(self.decision, 2)

    def reset_center(self):
        self.center_example.set_step(0)

    def select_center_step(self, step):
        self.center_example.set_step(step)
        self.changed.emit()

    def center_guide(self):
        return CENTER_STEPS[self.center_example.step][1:]

    def refresh(self, status, frame, steering=False):
        self.decision.setVisible(steering)
        self.center_controls.setVisible(not steering)
        self.center_previous.setEnabled(self.center_example.step > 0)
        self.center_next.setEnabled(self.center_example.step < 3)
        self.center_example.setVisible(not steering)
        self.lane.setVisible(steering)
        for index, control in enumerate(self.center_steps):
            control.setAutoExclusive(False)
            control.setChecked(index == self.center_example.step)
            control.setAutoExclusive(True)
        fresh = status.get('lesson_fresh') and (status.get('lane_geometry') or {}).get('reason') != 'stale'
        LearningVisual.frame(self.lane, frame if steering else None, fresh and steering)
        self.legend.setText(lane_explanation(status))
        heading, explanation = direction_text(status)
        self.direction.setText(heading)
        self.explanation.setText(explanation)
        valid = bool(status.get('autonomy_fresh') and status.get('lane_valid'))
        self.wheel.set_value(status.get('autonomy_steering', 0) if valid else 0)
        self.wheel.setVisible(valid)


class ControlGuide(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)
        self.labels = []
        for _ in range(3):
            item = label('', 19)
            item.setMinimumHeight(66)
            layout.addWidget(item)
            self.labels.append(item)

    def refresh(self, status, practice=False):
        self.setFixedHeight(110 if practice else 204)
        controller = status.get('input_mode') == 'controller'
        controls = ('왼쪽 스틱  ← →', 'RT  오른쪽 트리거', 'LT  왼쪽 트리거') if controller else ('←  →  방향키', '↑  위쪽 방향키', '↓  아래쪽 방향키')
        short_controls = ('왼쪽 스틱', 'RT', 'LT') if controller else ('← →', '↑', '↓')
        names = ('핸들 돌리기', '가속하기', '브레이크 밟기')
        step = (status.get('session') or {}).get('practice_step', 0)
        tasks = status.get('practice_tasks', [])
        for i, item in enumerate(self.labels):
            done = practice_complete(i, tasks)
            prefix = '✓ ' if practice and done else ''
            item.setVisible(not practice or i == step)
            item.setMinimumHeight(96 if practice else 60)
            item.setFont(QFont(FONT, 19 if practice else 16))
            item.setText(f'{prefix}{controls[i]}\n{names[i]}' if practice else f'{short_controls[i]}  {names[i]}')
            active = practice and step == i
            item.setStyleSheet(f'color: {TEAL if active else MUTED}; background: {"#123b43" if active else "#0b1b31"}; '
                              f'border: 1px solid {TEAL if active else "#233e56"}; border-radius: 10px; padding: 10px;')


class VisitorBase(QWidget):
    def __init__(self, send):
        super().__init__()
        self.send = send
        self.last_status = {}
        self.setMinimumSize(1280, 720)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setObjectName('visitorWindow')
        self.setStyleSheet(f'QWidget#visitorWindow {{ background: {BG}; color: {TEXT}; }}')
        QApplication.instance().installEventFilter(self)

    def eventFilter(self, watched, event):
        if not isinstance(watched, QWidget) or watched.window() is not self:
            return False
        if event.type() == QEvent.WindowDeactivate:
            if self.last_status.get('input_mode') == 'keyboard':
                self.send('release_keys')
            return False
        if event.type() not in (QEvent.KeyPress, QEvent.KeyRelease):
            return False
        key = event.key()
        down = event.type() == QEvent.KeyPress
        if self.last_status.get('state') == 'DIFFICULTY':
            if down and not event.isAutoRepeat() and key in (Qt.Key_Left, Qt.Key_A):
                if hasattr(self, 'move_difficulty_selection'):
                    self.move_difficulty_selection(-1)
                return True
            if down and not event.isAutoRepeat() and key in (Qt.Key_Right, Qt.Key_D):
                if hasattr(self, 'move_difficulty_selection'):
                    self.move_difficulty_selection(1)
                return True
            if down and not event.isAutoRepeat() and key in (Qt.Key_Space, Qt.Key_Return, Qt.Key_Enter):
                if hasattr(self, 'confirm_difficulty_selection'):
                    self.confirm_difficulty_selection()
                return True
        actions = {Qt.Key_Left: 'left', Qt.Key_A: 'left', Qt.Key_Right: 'right', Qt.Key_D: 'right',
                   Qt.Key_Up: 'throttle', Qt.Key_W: 'throttle', Qt.Key_Down: 'brake', Qt.Key_S: 'brake'}
        if self.last_status.get('state') == 'EXPLORE':
            actions.update({Qt.Key_J: 'view_left', Qt.Key_L: 'view_right',
                            Qt.Key_I: 'view_up', Qt.Key_K: 'view_down'})
        if key in actions:
            if not event.isAutoRepeat():
                self.send('key:' + actions[key] + (':down' if down else ':up'))
            return True
        if not down or event.isAutoRepeat():
            return False
        if key == Qt.Key_Q:
            self.close()
            return True
        if key == Qt.Key_F11:
            self.showNormal() if self.isFullScreen() else self.showFullScreen()
            return True
        if key == Qt.Key_Escape:
            self.send('secondary')
            return True
        if key in (Qt.Key_1, Qt.Key_2) and self.last_status.get('state') == 'RESULTS':
            self.send('answer_stop' if key == Qt.Key_1 else 'answer_go')
            return True
        if self.last_status.get('state') == 'DEMO' and key in (Qt.Key_1, Qt.Key_2, Qt.Key_3, Qt.Key_C):
            self.send('lane_toggle' if key == Qt.Key_C else f'obstacle_toggle_{key - Qt.Key_0}')
            return True
        if self.last_status.get('state') == 'EXPLORE' and key in (Qt.Key_X, Qt.Key_Y):
            self.send('venue_car' if key == Qt.Key_X else 'venue_home')
            return True
        if key == Qt.Key_Space:
            # Space is the advertised next-step shortcut, including after
            # clicking Back; Enter still activates a focused button normally.
            self.send('start')
            return True
        if key in (Qt.Key_Return, Qt.Key_Enter):
            focused = QApplication.focusWidget()
            if isinstance(focused, QPushButton) and focused.window() is self:
                focused.click()
            else:
                self.send('start')
            return True
        return False

    def closeEvent(self, event):
        self.send('release_keys')
        QApplication.instance().removeEventFilter(self)
        super().closeEvent(event)


class VisitorDriverWindow(VisitorBase):
    def __init__(self, node):
        super().__init__(lambda command: node._on_race_command(String(data=command)))
        self.node = node
        self.difficulty_preview = None
        self.dual_display = bool(getattr(node, 'analysis_screen_enabled', len(QApplication.instance().screens()) > 1))
        self.setWindowTitle('자율주행 탐험 · 운전 체험')
        root = QVBoxLayout(self)
        root.setContentsMargins(26, 18, 26, 22)
        root.setSpacing(14)
        header = QHBoxLayout()
        header.addWidget(label('01  자율주행 체험', 13, MUTED, True), 1)
        self.role = label('', 25, TEAL, True)
        self.role.setParent(self)
        self.role.hide()
        self.device = label('', 12, MUTED)
        self.device.setAlignment(Qt.AlignRight)
        header.addWidget(self.device)
        root.addLayout(header)
        self.journey = JourneyBar()
        root.addWidget(self.journey)
        self.pages = QStackedWidget()
        root.addWidget(self.pages, 1)
        self.pages.addWidget(self._welcome())
        self.pages.addWidget(self._learning())
        self.pages.addWidget(self._driving())
        self.pages.addWidget(self._results())
        self.pages.addWidget(self._paused())
        self.pages.addWidget(self._explore())
        self.pages.addWidget(self._difficulty())
        footer = QHBoxLayout()
        self.hint = label('', 14, MUTED)
        footer.addWidget(self.hint, 1)
        footer.addStretch(1)
        self.end = quiet_button('시작 화면으로', lambda: self.send('cancel'))
        footer.addWidget(self.end)
        root.addLayout(footer)
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh)
        self.refresh_timer.start(70)
        self.refresh()
        # Camera presentation must not wait for the slower lesson text/layout
        # refresh. Only update visible front-camera canvases, never analysis.
        self.camera_timer = QTimer(self)
        self.camera_timer.setTimerType(Qt.PreciseTimer)
        self.camera_timer.timeout.connect(self.refresh_camera)
        self.camera_timer.start(33)  # Freshness check; new frames arrive by signal.
        if hasattr(self.node, 'display_frame_notifier'):
            self.node.display_frame_notifier.ready.connect(self.refresh_camera, Qt.QueuedConnection)
            self.node.display_frame_notifier.action.connect(self.controller_action, Qt.QueuedConnection)

    def controller_action(self, action):
        status = self.node.status_snapshot()
        state = status.get('state')
        lesson = (status.get('session') or {}).get('lesson', 0)
        if action == 'menu':
            self.send('cancel')
        elif state == 'EXPLORE':
            if action == 'x_action': self.send('venue_car')
            elif action == 'alternate': self.send('venue_home')
        elif state == 'INTRO':
            if lesson in (1, 2) and self.lesson_example.mode:
                if action in ('previous', 'up'): self.select_example(0)
                elif action in ('next', 'down'): self.select_example(1)
                elif action == 'x_action': self.toggle_example()
                elif action == 'alternate' and self.lesson_example.paused: self.restart_example()
            elif lesson == 3 and action in ('previous', 'next', 'up', 'down'):
                delta = -1 if action in ('previous', 'up') else 1
                self.lesson_analysis.select_center_step(max(0, min(3, self.lesson_analysis.center_example.step + delta)))
        elif state == 'DEMO':
            index = {'previous': 0, 'up': 1, 'next': 2}.get(action)
            if index is not None and self.demo_controls.obstacles[index].isEnabled():
                self.demo_controls.obstacles[index].click()
            elif action == 'x_action' and self.demo_controls.lane.isEnabled():
                self.demo_controls.lane.click()
        elif state == 'PRACTICE' and action == 'alternate':
            self.send('practice')
        elif state == 'DIFFICULTY':
            if action in ('previous', 'up'):
                self.move_difficulty_selection(-1)
            elif action in ('next', 'down'):
                self.move_difficulty_selection(1)
            elif action == 'action':
                self.confirm_difficulty_selection()
            elif action == 'alternate':
                self.send('secondary')
        elif state == 'RUNNING' and status.get('duel'):
            if action == 'x_action': self.send('duel_recover')
            elif action == 'alternate': self.send('duel_finish')
        elif state == 'RESULTS' and action == 'x_action':
            self.send('practice')

    def refresh_camera(self):
        if not self.isVisible():
            return
        if hasattr(self.node, 'collect_display_frame'):
            self.node.collect_display_frame()
        fresh = time.monotonic() - getattr(self.node, 'last_image_at', 0) < 1.5
        for canvas in (self.lesson_camera, self.track_canvas, self.learning_visual.camera):
            if canvas.isVisibleTo(self):
                LearningVisual.frame(canvas, self.node.image, fresh)
        if self.venue_canvas.isVisibleTo(self):
            venue = getattr(self.node, 'venue', None)
            if venue:
                venue.collect_display_frame()
                LearningVisual.frame(self.venue_canvas, venue.image, venue.ready)
                self.venue_canvas.refresh({'player_pose': venue.frame_car,
                    'venue': {'pose': venue.frame_pose, 'ready': venue.ready}})
        self.node.camera_presented_frames = sum(canvas.frames_presented for canvas in
            (self.lesson_camera, self.track_canvas, self.learning_visual.camera))
        self.node.venue_presented_frames = self.venue_canvas.frames_presented

    def resizeEvent(self, event):
        super().resizeEvent(event)
        compact = self.height() < 850
        for widget in self.findChildren(QLabel):
            size = widget.property('visitorFontSize')
            if size:
                font = widget.font()
                font.setPointSize(max(10, round(size * (.8 if compact else 1))))
                widget.setFont(font)
        self.layout().setSpacing(8 if compact else 14)
        self.lesson_guide.setMinimumWidth(430 if compact else 390)
        self.lesson_guide.layout().setContentsMargins(*(16, 12, 16, 12) if compact else (20, 18, 20, 18))
        self.lesson_guide.layout().setSpacing(8 if compact else 12)
        font = self.lesson_description.font()
        font.setPointSize(13 if compact else 17)
        self.lesson_description.setFont(font)
        self.demo_controls.set_compact(compact)
        self.venue_guide.layout().setContentsMargins(*(14, 10, 14, 10) if compact else (20, 18, 20, 18))
        self.venue_guide.layout().setSpacing(6 if compact else 14)
        self.venue_map.setMinimumHeight(100 if compact else 150)
        self.venue_message.setFixedHeight(42 if compact else 52)
        self._size_waiting_text()

    def _size_waiting_text(self):
        # Reserve fixed line slots; QLabel.heightForWidth() includes its minimum
        # height, so feeding it back with padding makes the panel grow each frame.
        for widget, lines in ((self.pause_title, 2), (self.pause_detail, 5),
                              (self.welcome_message, 2)):
            widget.setFixedHeight(widget.fontMetrics().lineSpacing() * lines + 8)

    def _welcome(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(65, 10, 65, 10)
        layout.setSpacing(14)
        layout.addStretch(1)
        title = label('대회에 나가기 전, 여기서 연습했어요', 32, TEXT, True)
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        subtitle = label('자율주행 경진대회를 준비하며 만든 시뮬레이터예요.\n대회장의 트랙과 주변 환경을 최대한 비슷하게 재현했어요.', 20, MUTED)
        subtitle.setAlignment(Qt.AlignCenter)
        layout.addWidget(subtitle)
        row = QHBoxLayout()
        for number, title, text in (
            ('01', '대회장 둘러보기', '시점을 움직여 트랙의 모양과 차량 위치를 살펴봐요.'),
            ('02', '카메라로 길 찾기', '카메라 한 대의 영상이 핸들 움직임으로 이어지는 과정을 봐요.'),
            ('03', 'AI와 함께 운전', '직접 운전하며 같은 코스를 달리는 AI와 비교해요.'),
        ):
            card, box = surface()
            box.setSpacing(6)
            box.setContentsMargins(18, 12, 18, 12)
            box.addWidget(label(number, 16, TEAL, True))
            box.addWidget(label(title, 21, TEXT, True))
            detail = label(text, 16, MUTED)
            detail.setMinimumHeight(46)
            box.addWidget(detail)
            box.addStretch(1)
            row.addWidget(card, 1)
        layout.addLayout(row)
        self.welcome_start = button('시작하기', lambda: self.send('start'), True)
        self.welcome_start.setMinimumHeight(72)
        layout.addWidget(self.welcome_start)
        self.welcome_message = label('', 16, MUTED)
        self.welcome_message.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        layout.addWidget(self.welcome_message)
        layout.addStretch(1)
        return page

    def _explore(self):
        page = QWidget()
        root = QVBoxLayout(page)
        root.setContentsMargins(0,0,0,0)
        root.addWidget(label('먼저, 차가 달릴 대회장을 둘러보세요', 27, TEXT, True))
        row = QHBoxLayout()
        view, box = surface()
        box.addWidget(label('실제 주행에 사용하는 가상 대회장', 14, TEAL, True))
        self.venue_canvas = VenueCameraCanvas()
        box.addWidget(self.venue_canvas, 1)
        self.venue_controls = label('', 15, TEXT, True)
        box.addWidget(self.venue_controls)
        actions = QHBoxLayout()
        self.venue_car = button('X  내 차 찾기', lambda: self.send('venue_car'))
        self.venue_home = quiet_button('Y  처음 위치로', lambda: self.send('venue_home'))
        actions.addWidget(self.venue_car)
        actions.addWidget(self.venue_home)
        actions.addStretch(1)
        box.addLayout(actions)
        row.addWidget(view, 7)
        guide, box = surface()
        self.venue_guide = guide
        guide.setMinimumWidth(330)
        box.addWidget(label('이곳에서 무엇을 연습했을까요?', 19, TEAL, True))
        box.addWidget(label('차선을 따라 커브를 돌고,\n장애물을 피하는 방법을\n이곳에서 먼저 연습했어요.', 17))
        box.addWidget(label('트랙과 차량 위치를 찾아보세요', 17, TEXT, True))
        self.venue_map = VenueMap()
        box.addWidget(self.venue_map, 1)
        box.addWidget(label('노랑 · 자율주행차     청록 · 둘러보는 위치', 12, MUTED))
        box.addWidget(label('차 앞쪽 카메라 한 대가\n길을 찾는 과정을 이어서 볼까요?', 16))
        self.venue_message = label('', 12, MUTED)
        self.venue_message.setFixedHeight(52)
        box.addWidget(self.venue_message)
        row.addWidget(guide, 3)
        root.addLayout(row, 1)
        footer = QHBoxLayout()
        self.venue_back = button('B  처음 설명으로', lambda: self.send('secondary'))
        self.venue_next = button('A  차량 카메라로 보기', lambda: self.send('start'), True)
        footer.addWidget(self.venue_back)
        footer.addStretch(1)
        footer.addWidget(self.venue_next, 1)
        self.venue_next.setMaximumWidth(560)
        root.addLayout(footer)
        return page

    def _learning(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        self.lesson_title = label('', 27, TEXT, True)
        layout.addWidget(self.lesson_title)
        self.learning_visual = LearningVisual()
        layout.addWidget(self.learning_visual, 1)
        self.learning_visual.setVisible(not self.dual_display)
        self.driver_lesson_view = QWidget()
        row = QHBoxLayout(self.driver_lesson_view)
        row.setContentsMargins(0, 0, 0, 0)
        camera, box = surface()
        self.lesson_media_layout = box
        self.lesson_media_title = label('운전자 시점', 19, TEAL, True)
        box.addWidget(self.lesson_media_title)
        self.lesson_media_title.hide()
        self.lesson_camera = front_camera_canvas('전방 카메라를 준비하고 있어요')
        box.addWidget(self.lesson_camera, 1)
        self.lesson_example = PerceptionExample()
        self.lesson_example.hide()
        box.addWidget(self.lesson_example, 1)
        # Example playback belongs to the image, separate from example choices
        # and from the next-lesson action at the bottom of the page.
        self.example_playback = QWidget()
        playback = QHBoxLayout(self.example_playback)
        playback.setContentsMargins(0, 0, 0, 0)
        playback.setSpacing(16)
        self.example_pause = button('일시정지', self.toggle_example)
        self.example_pause.setFont(QFont(FONT, 13))
        self.example_pause.setMinimumHeight(44)
        self.example_pause.setMinimumWidth(132)
        playback.addWidget(self.example_pause)
        playback.addStretch(1)
        self.example_restart = button('이 예시 처음부터', self.restart_example)
        self.example_restart.setFont(QFont(FONT, 12))
        self.example_restart.setMinimumHeight(44)
        self.example_restart.setStyleSheet(
            f'QPushButton {{ color: {MUTED}; background: transparent; '
            'border: 1px solid transparent; border-radius: 8px; padding: 8px 12px; }'
            f'QPushButton:hover {{ color: {TEXT}; border-color: #345069; }}'
            'QPushButton:focus { border: 2px solid #f0ca73; }'
        )
        self.example_restart.setAccessibleName('현재 예시 영상을 처음부터 다시 보기')
        playback.addWidget(self.example_restart)
        self.example_restart.hide()
        box.addWidget(self.example_playback)
        self.example_playback.hide()
        self.lesson_analysis = MainAnalysisLesson()
        self.lesson_analysis.changed.connect(self.refresh)
        self.active_analysis_lesson = None
        self.lesson_analysis.hide()
        box.addWidget(self.lesson_analysis, 1)
        row.addWidget(camera, 7)
        self.lesson_guide, box = surface()
        self.lesson_guide.setMinimumWidth(390)
        self.lesson_guide.setMaximumWidth(590)
        box.setSpacing(12)
        self.lesson_step = label('', 14, TEAL, True)
        box.addWidget(self.lesson_step)
        self.lesson_context = label('', 12, MUTED)
        box.addWidget(self.lesson_context)
        self.lesson_description = label('', 17, TEXT)
        box.addWidget(self.lesson_description)
        if not self.dual_display:
            layout.insertWidget(1, self.lesson_description)
        self.lesson_task = label('', 17, TEAL, True)
        self.lesson_cue = label('', 15, MUTED)
        self.lesson_cue.setStyleSheet('color: #c4d4e3; background: #102b3b; border-left: 3px solid #25d8d0; border-radius: 5px; padding: 16px;')
        box.addWidget(self.lesson_task)
        box.addWidget(self.lesson_cue)
        self.lesson_term = label('', 11, MUTED)
        box.addWidget(self.lesson_term)
        self.example_controls = QWidget()
        controls = QVBoxLayout(self.example_controls)
        controls.setContentsMargins(0, 0, 0, 0)
        selector_header = QHBoxLayout()
        selector_header.addWidget(label('어떤 예시로 볼까요?', 14, TEXT, True))
        self.example_input_hint = label('', 12, TEAL)
        selector_header.addWidget(self.example_input_hint, 1, Qt.AlignRight)
        controls.addLayout(selector_header)
        choices = QHBoxLayout()
        self.example_choices = []
        for index in range(2):
            choice = button('', lambda _checked=False, index=index: self.select_example(index))
            choice.setFont(QFont(FONT, 16, QFont.Bold))
            choice.setMinimumHeight(58)
            choice.setCheckable(True)
            choice.setAutoExclusive(True)
            choice.setStyleSheet(choice.styleSheet() + 'QPushButton:checked { color: #25d8d0; border: 2px solid #25d8d0; background: #123b43; }')
            choices.addWidget(choice, 1)
            self.example_choices.append(choice)
        controls.addLayout(choices)
        self.example_credit = label('', 11, MUTED)
        box.addWidget(self.example_credit)
        self.lesson_media_layout.insertWidget(0, self.example_controls)
        self.example_controls.hide()
        self.demo_direction = label('', 17, TEAL, True)
        box.addWidget(self.demo_direction)
        self.demo_direction.hide()
        self.demo_controls = DemoControls(self.send)
        box.addWidget(self.demo_controls, 1)
        self.demo_controls.hide()
        box.addStretch(1)
        row.addWidget(self.lesson_guide, 3)
        self.driver_lesson_view.setVisible(self.dual_display)
        layout.addWidget(self.driver_lesson_view, 1)
        actions = QHBoxLayout()
        self.lesson_back = button('이전 설명', lambda: self.send('secondary'))
        self.lesson_next = button('', lambda: self.send('start'), True)
        self.lesson_next.setMinimumWidth(360)
        self.lesson_next.setMaximumWidth(560)
        actions.addWidget(self.lesson_back)
        actions.addStretch(1)
        actions.addWidget(self.lesson_next, 1)
        layout.addLayout(actions)
        return page

    def toggle_example(self):
        self.lesson_example.toggle_pause()
        self.refresh_example_playback()

    def restart_example(self):
        self.lesson_example.restart()
        self.refresh_example_playback()
        self.example_pause.setFocus()

    def select_example(self, index):
        self.lesson_example.set_variant(index)
        self.refresh()

    def refresh_example_playback(self):
        paused = self.lesson_example.paused
        controller = getattr(self, 'last_status', {}).get('input_mode') == 'controller'
        self.example_pause.setText(('X  ' if controller else '') + ('영상 이어 보기' if paused else '영상 잠깐 멈추기'))
        self.example_restart.setText(('Y  ' if controller else '') + '이 예시 처음부터')
        self.example_pause.setAccessibleName('예시 영상 이어서 보기' if paused else '예시 영상 일시정지')
        # Replay is secondary and only offered while inspecting a paused image.
        self.example_restart.setVisible(bool(self.lesson_example.mode) and paused)

    def _driving(self):
        page = QWidget()
        row = QHBoxLayout(page)
        row.setContentsMargins(0, 0, 0, 0)
        camera, left = surface()
        self.drive_title = label('', 27, TEXT, True)
        left.addWidget(self.drive_title)
        self.track_canvas = front_camera_canvas('전방 카메라를 준비하고 있어요')
        left.addWidget(self.track_canvas, 1)
        self.drive_banner = label('', 20, TEAL, True)
        left.addWidget(self.drive_banner)
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setMinimumHeight(30)
        self.progress.setFont(QFont(FONT, 14))
        self.progress.setStyleSheet('QProgressBar { color: #f4f8ff; background: #12293d; border: none; border-radius: 7px; text-align: center; } QProgressBar::chunk { background: #126d70; }')
        left.addWidget(self.progress)
        row.addWidget(camera, 7)
        side, right = surface()
        side.setMinimumWidth(340)
        side.setMaximumWidth(430)
        self.practice_title = label('', 22, TEAL, True)
        right.addWidget(self.practice_title)
        self.controls = ControlGuide()
        right.addWidget(self.controls)
        self.practice_response = label('', 17, TEXT)
        right.addWidget(self.practice_response)
        self.duel_panel = DuelPanel(self.send)
        right.addWidget(self.duel_panel, 1)
        self.duel_panel.hide()
        self.driving_spacer = QWidget()
        right.addWidget(self.driving_spacer, 1)
        self.drive_next = button('', lambda: self.send('start'), True)
        self.skip_practice = quiet_button('연습 건너뛰고 AI와 함께 출발', lambda: self.send('practice'))
        right.addWidget(self.drive_next)
        right.addWidget(self.skip_practice)
        row.addWidget(side, 3)
        return page

    def _difficulty(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(70, 30, 70, 30)
        layout.addStretch(1)
        title = label('AI와 함께 달릴 난이도를 골라 주세요', 32, TEXT, True)
        title.setAlignment(Qt.AlignHCenter)
        layout.addWidget(title)
        detail = label('선택한 난이도에 따라 AI 속도와 코스의 장애물 수가 달라져요.', 18, MUTED)
        detail.setAlignment(Qt.AlignHCenter)
        layout.addWidget(detail)
        choices = QHBoxLayout()
        choices.setSpacing(20)
        self.difficulty_buttons = {}
        self.difficulty_index = 0
        for key, profile in DUEL_DIFFICULTIES.items():
            # All choices share the same neutral appearance.  The checked
            # state below is the only source of the highlighted colour.
            choice = button('', lambda _checked=False, key=key: self.send(f'difficulty:{key}'))
            choice.setCheckable(True)
            choice.setAutoExclusive(True)
            choice.setMinimumSize(250, 190)
            choice.setStyleSheet(choice.styleSheet() +
                                 'QPushButton:focus { border: 1px solid #345069; }'
                                 'QPushButton:checked { color: #25d8d0; border: 3px solid #25d8d0; background: #123b43; }'
                                 'QPushButton:checked:focus { border: 3px solid #25d8d0; }')
            self.difficulty_buttons[key] = choice
            choices.addWidget(choice, 1)
        layout.addLayout(choices)
        self.difficulty_hint = label('', 15, TEAL, True)
        self.difficulty_hint.setAlignment(Qt.AlignHCenter)
        layout.addWidget(self.difficulty_hint)
        self.difficulty_back = quiet_button('연습으로 돌아가기', lambda: self.send('secondary'))
        self.difficulty_back.setMaximumWidth(250)
        layout.addWidget(self.difficulty_back, 0, Qt.AlignHCenter)
        layout.addStretch(1)
        self._show_difficulty_choice('beginner')
        return page

    def _show_difficulty_choice(self, selected):
        self.difficulty_index = tuple(DUEL_DIFFICULTIES).index(selected)
        for key, button_widget in self.difficulty_buttons.items():
            profile = DUEL_DIFFICULTIES[key]
            button_widget.setText(
                f"{profile['label']}\n\nAI 속도 {int(profile['ai_speed_ratio'] * 100)}%\n"
                f"장애물 {sum(profile['obstacles'])}대\n\n{profile['description']}"
            )
            button_widget.setChecked(key == selected)
        mode = getattr(self, 'last_status', {}).get('input_mode')
        self.difficulty_hint.setText('A 버튼으로 선택' if mode == 'controller' else '원하는 난이도를 눌러 시작하세요')

    def move_difficulty_selection(self, delta):
        keys = tuple(DUEL_DIFFICULTIES)
        selected = self.difficulty_preview or self.last_status.get('session', {}).get('difficulty', 'beginner')
        index = (keys.index(selected) + delta) % len(keys)
        self.difficulty_preview = keys[index]
        self._show_difficulty_choice(self.difficulty_preview)

    def confirm_difficulty_selection(self):
        selected = self.difficulty_preview or self.last_status.get('session', {}).get('difficulty', 'beginner')
        self.send(f'difficulty:{selected}')

    def _results(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(45, 5, 45, 5)
        self.result_heading = label('', 30, TEAL, True)
        self.result_detail = label('', 21)
        layout.addWidget(self.result_heading)
        layout.addWidget(self.result_detail)
        layout.addStretch(1)
        quiz, box = surface()
        self.result_quiz = quiz
        box.addWidget(label('배운 내용 확인하기', 16, TEAL, True))
        box.addWidget(label('AI가 운전 중에 차선을 찾지 못하면?', 23, TEXT, True))
        choices = QHBoxLayout()
        self.answer_choices = {}
        for answer, text in (('answer_stop', '1  멈추고 다시 확인해요'), ('answer_go', '2  그대로 계속 달려요')):
            choice = button(text, lambda _checked=False, answer=answer: self.send(answer))
            choice.setCheckable(True)
            choice.setAutoExclusive(True)
            choice.setStyleSheet(choice.styleSheet() + 'QPushButton:checked { color: #25d8d0; border: 2px solid #25d8d0; background: #123b43; }')
            self.answer_choices[answer] = choice
            choices.addWidget(choice)
        box.addLayout(choices)
        self.answer_feedback = label('', 17, MUTED)
        box.addWidget(self.answer_feedback)
        layout.addWidget(quiz)
        self.duel_result = DuelPanel(self.send, results=True)
        layout.addWidget(self.duel_result, 4)
        self.duel_result.hide()
        layout.addStretch(1)
        actions = QHBoxLayout()
        self.result_practice = button('운전 다시 연습하기', lambda: self.send('practice'))
        self.result_again = button('한 번 더 도전', lambda: self.send('start'), True)
        actions.addWidget(self.result_practice, 1)
        actions.addWidget(self.result_again, 1)
        layout.addLayout(actions)
        return page

    def _paused(self):
        page, layout = surface()
        layout.addStretch(1)
        content = QWidget()
        content.setMinimumWidth(700)
        content.setMaximumWidth(850)
        column = QVBoxLayout(content)
        column.setSpacing(20)
        self.pause_title = label('', 34, CORAL, True)
        self.pause_detail = label('', 22)
        self.pause_title.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        self.pause_detail.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        column.addWidget(self.pause_title)
        column.addWidget(self.pause_detail)
        self.resume = button('이어서 체험하기', lambda: self.send('start'), True)
        self.pause_end = quiet_button('운전 마치고 결과 보기', lambda: self.send('secondary'))
        for control in (self.resume, self.pause_end):
            policy = control.sizePolicy()
            policy.setRetainSizeWhenHidden(True)
            control.setSizePolicy(policy)
        column.addWidget(self.resume)
        column.addWidget(self.pause_end)
        layout.addWidget(content, 0, Qt.AlignHCenter)
        layout.addStretch(1)
        return page

    def refresh(self):
        status = self.node.status_snapshot()
        self.last_status = status
        session = status['session'] or {}
        state = status['state']
        if state != 'DIFFICULTY':
            self.difficulty_preview = None
        phase = session.get('phase', 'IDLE')
        mode = status['input_mode'] if phase != 'IDLE' else ('controller' if status['controller_connected'] else 'keyboard')
        status['input_mode'] = mode
        primary_key = 'A' if mode == 'controller' else 'Space'
        secondary_key = 'B' if mode == 'controller' else 'Esc'
        self.device.setText('컨트롤러' if mode == 'controller' else '키보드 · 마우스')
        self.end.setText('BACK  시작 화면으로' if mode == 'controller' else '시작 화면으로')
        text, color = state_label(status)
        self.role.setText(text)
        self.role.setStyleSheet(f'color: {color};')
        self.role.hide()
        self.journey.set_phase(phase)
        self.end.setVisible(phase != 'IDLE' and state != 'PAUSED')
        self.hint.setText(shortcut_hint(status))
        self.hint.setVisible(mode != 'controller')
        frames = {'camera': self.node.image, **self.node.lesson_images}
        example_mode = ({1: 'segmentation', 2: 'bev'}.get(session.get('lesson', 0))
                        if state == 'INTRO' and self.dual_display else None)
        if self.lesson_example.mode != example_mode:
            self.lesson_example.set_mode(example_mode)
        if state == 'INTRO' and not self.lesson_example.variant:
            teaching = getattr(self.node, 'teaching_frames', None)
            if teaching is not None:
                self.lesson_example.set_live_frames(teaching.latest)
        self.lesson_example.setVisible(bool(example_mode))
        self.example_controls.setVisible(bool(example_mode))
        self.example_input_hint.setText('방향패드로 선택' if mode == 'controller' else '보고 싶은 예시를 눌러 보세요')
        self.example_credit.setVisible(bool(example_mode and self.lesson_example.variant))
        self.example_playback.setVisible(bool(example_mode))
        self.refresh_example_playback()
        analysis_lesson = state == 'INTRO' and self.dual_display and session.get('lesson', 0) >= 3
        analysis_index = session.get('lesson') if analysis_lesson else None
        if analysis_index == 3 and self.active_analysis_lesson != 3:
            self.lesson_analysis.reset_center()
        self.active_analysis_lesson = analysis_index
        self.lesson_analysis.setVisible(analysis_lesson)
        self.lesson_camera.setVisible(not example_mode and not analysis_lesson)
        if analysis_lesson:
            self.lesson_analysis.refresh(status, frames.get('fitting'), steering=session['lesson'] == 4)
        live_example = example_mode and not self.lesson_example.variant
        self.lesson_media_title.setText('실시간 도로 · AI가 달릴 공간을 구분해요' if live_example and example_mode == 'segmentation' else
                                       '실시간 도로 · 앞에서 본 도로를 위에서 봐요' if live_example else
                                       '설명용 예시 · 영역 구분' if example_mode == 'segmentation' else
                                       '설명용 예시 · 비스듬한 사진을 반듯하게 펼쳐요' if example_mode else
                                       '실시간 분석 · 차선 가운데 → 핸들 방향' if analysis_lesson and session['lesson'] == 4 else
                                       '설명 그림 · 경계에서 가운데 찾기' if analysis_lesson else '운전자 시점 · 실제 전방 카메라')
        demo = state == 'DEMO'
        self.demo_controls.setVisible(demo)
        self.driver_lesson_view.setVisible(self.dual_display or demo)
        self.learning_visual.setVisible(not self.dual_display and not demo)
        for item in (self.lesson_step, self.lesson_cue):
            item.setVisible(not demo)
        self.lesson_context.hide()
        self.lesson_task.setVisible(state == 'INTRO' and session.get('lesson') in (1, 2, 3))
        self.lesson_description.setVisible(not demo)
        self.demo_direction.hide()
        if state == 'DEMO':
            self.demo_controls.refresh(status)
            LearningVisual.frame(self.lesson_camera, self.node.image, status['camera_fresh'])
        for key in ('segmentation', 'bev'):
            status[f'{key}_fresh'] = time.monotonic() - getattr(self.node, 'lesson_received_at', {}).get(key, 0) < 1.5
        if state in ('PAUSED', 'RESETTING', 'COUNTDOWN') or (state == 'ERROR' and phase != 'IDLE'):
            self.pages.setCurrentIndex(4)
            titles = {'PAUSED': '운전을 잠시 멈췄어요', 'RESETTING': '차를 출발 위치로 옮기고 있어요',
                      'ERROR': '체험을 잠시 멈췄어요', 'COUNTDOWN': '출발까지 ' + status['countdown']}
            if status.get('reset_kind') == 'safety':
                titles['RESETTING'] = '안전한 도로 위로 돌아가고 있어요'
            self.pause_title.setText(titles[state])
            detail = status['message']
            if state == 'PAUSED' and detail in ('잠시 멈췄어요', '운전을 잠시 멈췄어요'):
                detail = '준비되면 이어서 운전할 수 있어요.'
            if state == 'PAUSED' and neutral_text(status) not in detail:
                detail += '\n' + neutral_text(status)
            self.pause_detail.setText(detail)
            if state == 'COUNTDOWN' and status.get('duel'):
                duel = status['duel']
                ready = duel.get('start_ready', False)
                self.pause_title.setText('함께 출발까지 ' + status['countdown'] if ready else '같이 달릴 준비를 하고 있어요')
                controls = '왼쪽 스틱 핸들 · RT 가속 · LT 브레이크' if status.get('input_mode') == 'controller' else '← → 핸들 · ↑ 가속 · ↓ 브레이크'
                throttle_hint = 'RT는 누를수록 빨라져요. ' if status.get('input_mode') == 'controller' else ''
                obstacle_count = duel.get('obstacle_count', sum(RACE_OBSTACLES))
                self.pause_detail.setText((f'장애물 {obstacle_count}대를 지나 한 바퀴 완주하세요.\n' + controls + '\n' + throttle_hint + '청록색 AI와는 부딪히지 않아요.') if ready else detail)
            self.resume.setVisible(state == 'PAUSED')
            self.resume.setText(f'{primary_key}  이어서 운전하기')
            self.pause_end.setVisible(state == 'PAUSED')
            self.pause_end.setText('운전 마치고 결과 보기' if phase == 'CHALLENGE' else '시작 화면으로')
        elif phase == 'IDLE':
            self.pages.setCurrentIndex(0)
            self.welcome_start.setText(f'{primary_key}  대회장 둘러보기')
            self.welcome_message.setText(
                f'버튼을 누르거나 {primary_key} 키로 시작하세요. 시간 제한 없이 살펴볼 수 있어요.'
                if state == 'READY' and status['message'].startswith('시작하기를 누르면') else status['message'])
        elif state == 'EXPLORE':
            self.pages.setCurrentIndex(5)
            venue = status.get('venue') or {}
            self.venue_map.refresh(status)
            self.venue_controls.setText('왼쪽 스틱  이동     ·     오른쪽 스틱  방향·기울기' if mode == 'controller' else '방향키 / W A S D  이동     ·     I J K L  둘러보기')
            self.venue_message.setText(venue.get('error') or ('높이는 고정 · 트랙 안에서만 이동해요' if venue.get('ready') else '대회장 화면을 준비하고 있어요.'))
            self.venue_next.setText(f'{primary_key}  차량 카메라로 보기')
            self.venue_next.setEnabled(bool(venue.get('ready')))
            self.venue_back.setText(f'{secondary_key}  처음 설명으로')
        elif state in ('INTRO', 'DEMO'):
            self.pages.setCurrentIndex(1)
            lesson = session.get('lesson', 0) if state == 'INTRO' else len(LESSONS) - 1
            self.lesson_title.setText(LESSONS[lesson][1] if state == 'INTRO' else 'AI가 차선을 따라 운전하고 있어요')
            self.lesson_description.setText(LESSONS[lesson][2] if state == 'INTRO' else '영상이 바뀌면 찾은 차선과 핸들 방향도 바뀌어요. 충분히 살펴본 뒤 직접 운전해 보세요.')
            if self.dual_display:
                title, description, task, cue = LESSON_GUIDES[lesson]
                self.lesson_title.setText(title if state == 'INTRO' else 'AI에게 장애물과 차선 변경을 맡겨 보세요')
                self.lesson_description.setText(description if state == 'INTRO' else
                    '지금은 AI가 핸들과 속도를 조절해요.\n카메라로 길을 보고, 가운데를 찾고, 방향을 정하는 과정이 계속 반복돼요.')
                self.lesson_step.setText(f'원리 {lesson + 1} / {len(LESSONS)}' if state == 'INTRO' else 'AI 운전 보기')
                self.lesson_term.setVisible(state == 'INTRO' and bool(LESSON_TERMS[lesson]))
                self.lesson_term.setText(LESSON_TERMS[lesson])
                self.lesson_task.setText(task if state == 'INTRO' else '커브에서 핸들 방향을 살펴보세요')
                self.lesson_cue.setText(cue if state == 'INTRO' else '길이 휘면 차도 그 방향으로 도나요? 충분히 봤다면 아래 버튼을 눌러 직접 운전해 보세요.')
                if example_mode:
                    if self.lesson_example.variant:
                        title, description = self.lesson_example.PUBLIC_GUIDES[example_mode]
                        self.lesson_title.setText(title)
                        self.lesson_description.setText(description)
                    stage, detail, credit = self.lesson_example.explanation()
                    self.lesson_task.setText(stage)
                    self.lesson_cue.setText(detail)
                    self.example_credit.setText(credit)
                    self.example_credit.setVisible(bool(credit))
                    for index, choice in enumerate(self.example_choices):
                        key = ('←  ' if index == 0 else '→  ') if mode == 'controller' else ''
                        choice.setText(key + self.lesson_example.CHOICES[example_mode][index] + ('  ✓' if index == self.lesson_example.variant else ''))
                        choice.setChecked(index == self.lesson_example.variant)
                elif state == 'INTRO' and lesson == 3:
                    heading, detail = self.lesson_analysis.center_guide()
                    self.lesson_task.setText(heading)
                    self.lesson_cue.setText(detail)
                LearningVisual.frame(self.lesson_camera, self.node.image, status['camera_fresh'])
            else:
                self.learning_visual.refresh(status, frames, lesson)
            self.lesson_back.setVisible(state == 'INTRO')
            self.lesson_back.setText(('B  ' if mode == 'controller' else '') + ('대회장 다시 보기' if lesson == 0 else '이전 설명'))
            self.lesson_next.setText(f'{primary_key}  {primary_text(status)}')
        elif state == 'DIFFICULTY':
            self.pages.setCurrentIndex(6)
            if self.difficulty_preview not in DUEL_DIFFICULTIES:
                self.difficulty_preview = session.get('difficulty', status.get('difficulty', 'beginner'))
            self._show_difficulty_choice(self.difficulty_preview if self.difficulty_preview in DUEL_DIFFICULTIES else 'beginner')
            self.difficulty_hint.setText(
                '방향패드로 고르고 A 버튼으로 선택하세요' if mode == 'controller'
                else '원하는 난이도를 눌러 시작하세요'
            )
            self.difficulty_back.setText(f'{secondary_key}  연습으로 돌아가기')
        elif state in ('PRACTICE', 'RUNNING'):
            self.pages.setCurrentIndex(2)
            practice = state == 'PRACTICE'
            duel = bool(status.get('duel')) and not practice
            self.duel_panel.setVisible(duel)
            self.driving_spacer.setVisible(not duel)
            self.controls.setVisible(not duel)
            self.practice_title.setVisible(not duel)
            self.practice_response.setVisible(not duel)
            if duel:
                self.duel_panel.refresh(status)
            step = session.get('practice_step', 0)
            practice_titles = ('핸들을 왼쪽·오른쪽으로 움직여 보세요', '가속해서 차를 앞으로 움직여 보세요', '브레이크를 눌러 차를 멈춰 보세요')
            self.drive_title.setText(practice_titles[step] if practice else '직접 운전해 한 바퀴를 돌아보세요')
            if duel:
                self.drive_title.setText(f"{duel.get('difficulty_label', '도전')} · 장애물 {duel.get('obstacle_count', sum(RACE_OBSTACLES))}대를 지나, AI와 함께 한 바퀴")
            LearningVisual.frame(self.track_canvas, self.node.image, status['camera_fresh'])
            self.controls.refresh(status, practice)
            prompts = ('핸들', '가속', '브레이크')
            self.practice_title.setText(f'운전 연습 {step + 1}/3 · {prompts[step]}' if practice else '내가 운전 중이에요')
            if practice:
                done = practice_complete(step, status['practice_tasks'])
                tasks = status['practice_tasks']
                steering_hint = ('왼쪽 완료 · 이제 오른쪽으로 움직여 보세요.' if '왼쪽 조향' in tasks else
                                 '오른쪽 완료 · 이제 왼쪽으로 움직여 보세요.' if '오른쪽 조향' in tasks else
                                 '왼쪽과 오른쪽을 한 번씩 움직여 보세요.')
                pending = (steering_hint, '가속을 누르면 차가 앞으로 움직여요.', '브레이크를 눌러 보세요.')
                feedback = ('✓ 좌우 조작 완료', '✓ 가속 조작 완료', '✓ 브레이크 조작 완료')
                self.practice_response.setText(feedback[step] + '\n아래 버튼으로 다음 단계에 가세요.' if done else pending[step])
            else:
                self.practice_response.setText('도로를 벗어나면 도로 위로 돌아와요.')
            self.drive_banner.setVisible(not status['input_armed'])
            self.drive_banner.setText(neutral_text(status))
            self.progress.setVisible(not practice)
            self.progress.setValue(int(status['player_progress_percent']))
            self.progress.setFormat('출발   ·   한 바퀴 중 %p%   ·   도착')
            self.drive_next.setVisible(practice)
            self.drive_next.setText(f'{primary_key}  {primary_text(status)}' if not practice or done else
                                   ('좌우 조작 후 다음', '가속 조작 후 다음', '브레이크 조작 후 다음')[step])
            self.drive_next.setEnabled(practice_complete(step, status['practice_tasks']) if practice else True)
            self.skip_practice.setVisible(practice)
            self.skip_practice.setText(('Y  ' if mode == 'controller' else '') + '연습 건너뛰고 AI와 함께 출발')
        else:
            self.pages.setCurrentIndex(3)
            duel = status.get('duel') or {}
            comparison = bool(duel.get('player'))
            self.duel_result.setVisible(comparison)
            self.result_quiz.setVisible(not comparison)
            records = session.get('results', [])
            latest = records[-1] if records else {}
            self.result_heading.setText('한 바퀴 완주했어요!' if latest.get('completed') and latest.get('valid', True) else '이번 운전 결과예요')
            self.result_detail.setText(session.get('error') or result_text(records))
            if comparison:
                self.duel_result.refresh(status)
                self.result_heading.setText('함께 달린 결과')
                self.result_detail.setText(session.get('error') or '안전장치 제동은 충돌 위험 때문에 자동으로 멈춘 횟수예요. 직접 누른 브레이크는 세지 않아요.')
            answers = {
                'answer_stop': '맞아요. AI는 따라갈 선을 확인할 수 없으면 차를 멈추고 다시 확인해요.',
                'answer_go': '이 차의 AI는 멈추도록 만들었어요. 따라갈 선을 모르는 상태에서 계속 달리지 않아요.',
            }
            self.answer_feedback.setText(answers.get(session.get('answer'), '버튼 또는 숫자 1·2를 눌러 확인해 보세요.'))
            for answer, choice in self.answer_choices.items():
                choice.setAutoExclusive(False)
                choice.setChecked(session.get('answer') == answer)
                choice.setAutoExclusive(True)
            self.result_again.setText(f'{primary_key}  한 바퀴 다시 운전')
            self.result_practice.setText(('X  ' if mode == 'controller' else '') + '운전 다시 연습하기')
            if comparison:
                self.result_again.setText(f'{primary_key}  같은 코스로 다시 도전')


class VisitorObserverWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.setMinimumSize(1280, 720)
        self.setObjectName('visitorWindow')
        self.setStyleSheet(f'QWidget#visitorWindow {{ background: {BG}; color: {TEXT}; }}')
        self.setWindowFlag(Qt.WindowDoesNotAcceptFocus, True)
        self.node = node
        self.setWindowTitle('자율주행 탐험 · AI 해설')
        root = QVBoxLayout(self)
        root.setContentsMargins(26, 16, 26, 20)
        root.setSpacing(12)
        heading = QHBoxLayout()
        heading.addWidget(label('02  실시간 분석 · 전체 트랙', 14, MUTED, True), 1)
        self.role = label('AI는 이렇게 운전해요', 17, TEAL, True)
        self.role.setAlignment(Qt.AlignRight)
        heading.addWidget(self.role)
        root.addLayout(heading)
        self.title = label('', 24, TEXT, True)
        self.description = label('', 15, MUTED)
        root.addWidget(self.title)
        root.addWidget(self.description)
        self.visual = LearningVisual(overview=True)
        root.addWidget(self.visual, 1)
        self.venue_context, venue_layout = surface()
        self.venue_overview = VenueMap()
        venue_layout.addWidget(self.venue_overview, 1)
        legend = label('노랑 · 자율주행차 위치       청록 · 지금 둘러보는 곳', 18, TEXT, True)
        legend.setAlignment(Qt.AlignCenter)
        venue_layout.addWidget(legend)
        venue_layout.addWidget(label('이 지도는 체험자를 위한 안내예요. AI는 차량 앞쪽 카메라 한 대의 영상에서 길을 찾아요.', 16, MUTED))
        root.addWidget(self.venue_context, 1)
        self.detail = label('', 13)
        root.addWidget(self.detail)
        self.detail.hide()  # Detailed prompts are on the participant's screen.
        bottom = QHBoxLayout()
        self.help = label('조작 안내는 운전 화면에서 크게 볼 수 있어요.', 12, MUTED)
        bottom.addWidget(self.help, 1)
        self.help.setText('자세한 설명과 모든 조작은 01 자율주행 체험 화면에서 볼 수 있어요.')
        root.addLayout(bottom)
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh)
        self.refresh_timer.start(100)
        self.refresh()
        if hasattr(node, 'frame_notifier'):
            node.frame_notifier.ready.connect(self.refresh_overview, Qt.QueuedConnection)
        self.video_timer = QTimer(self)
        self.video_timer.setTimerType(Qt.PreciseTimer)
        self.video_timer.timeout.connect(self.refresh_overview)
        self.video_timer.start(33)
        self.performance_pub = (node.create_publisher(String, '/exhibition/display/status', 1)
                                if hasattr(node, 'create_publisher') else None)
        self.performance_at = 0.

    def refresh_overview(self):
        if hasattr(self.node, 'collect_overview_frame'):
            self.node.collect_overview_frame()
        if self.visual.camera.isVisibleTo(self):
            fresh = time.monotonic()-self.node.image_received_at.get('overview', 0) < 1.5
            LearningVisual.frame(self.visual.camera, self.node.images.get('overview'), fresh)
        if self.performance_pub and time.monotonic()-self.performance_at > .2:
            import json
            self.performance_at = time.monotonic()
            self.performance_pub.publish(String(data=json.dumps({
                'overview_presented_frames': self.visual.camera.frames_presented})))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        compact = self.height() < 850
        for widget, small, large in ((self.title, 20, 24), (self.description, 12, 15),
                                     (self.detail, 11, 13), (self.help, 10, 12)):
            font = widget.font()
            font.setPointSize(small if compact else large)
            widget.setFont(font)
        self.layout().setSpacing(8 if compact else 12)

    def refresh(self):
        status = dict(self.node.race_status)
        if hasattr(self.node, 'collect_overview_frame'):
            self.node.collect_overview_frame()
        session = status.get('session') or {}
        fresh = time.monotonic() - self.node.last_race_at < 2
        context = not fresh or session.get('phase', 'IDLE') in ('IDLE', 'EXPLORE')
        self.visual.setVisible(not context)
        self.venue_context.setVisible(context)
        if context:
            self.role.setText('대회장 안내 지도')
            self.title.setText('트랙 전체와 자율주행차의 위치예요')
            self.description.setText('메인 화면에서 낮은 시점으로 둘러보고, 이 지도에서 전체 위치를 확인할 수 있어요.')
            self.venue_overview.refresh(status)
            return
        if not fresh:
            self.role.setText('체험 화면과 연결하고 있어요')
            self.title.setText('잠시만 기다려 주세요')
            self.description.setText('운전 화면이 준비되면 함께 시작해요.')
            self.visual.refresh(
                {'overview_fresh': time.monotonic() - self.node.image_received_at.get('overview', 0) < 1.5},
                self.node.images, analysis_visible=False)
            self.detail.setText('')
            return
        text, color = state_label(status)
        self.role.setText(text)
        self.role.setStyleSheet(f'color: {color};')
        state = status.get('state')
        phase = session.get('phase', 'IDLE')
        lesson = session.get('lesson', 0) if state == 'INTRO' else len(LESSONS) - 1
        stopped = state in ('PAUSED', 'RESETTING', 'COUNTDOWN', 'ERROR')
        if stopped:
            self.title.setText('차가 멈춰 있어요' if state == 'PAUSED' else '출발을 준비하고 있어요')
            self.description.setText(status.get('message', ''))
            self.detail.setText('차가 멈춰 있어도 인지 과정은 계속 볼 수 있어요. ' + neutral_text(status))
        elif state == 'INTRO':
            self.title.setText(f'{lesson + 1}/{len(LESSONS)}  {LESSONS[lesson][1]}')
            self.description.setText('현재 차량의 분석 결과예요. 테두리가 밝은 칸이 지금 배우는 단계예요.')
            prompts = (
                '찾아보세요: 트랙에서 내 차는 어디에 있고, 전방 카메라는 어느 쪽을 바라볼까요?',
                '②의 파랑·분홍 부분은 AI가 찾은 달릴 공간이에요. 두 색 사이의 경계를 살펴보세요.',
                '②에서 멀리 있던 길이 ③에서 어떻게 펼쳐지는지 비교할 수 있어요.',
                '④의 초록 선은 흰 점들 사이의 가운데예요. 점선은 부족한 정보를 보완한 구간이에요.',
                '살펴보세요: 차가 커브를 지나갈 때 초록 선과 핸들 방향이 함께 바뀌나요?',
            )
            self.detail.setText(prompts[lesson])
        elif state == 'DEMO':
            self.title.setText('보는 정보가 바뀌면 핸들 방향도 바뀌어요')
            self.description.setText('② 달릴 공간 → ③ 위에서 본 도로 → ④ 가운데 선 → ⑤ 핸들 방향이 계속 갱신돼요.')
            self.detail.setText('커브에서 비교해 보세요: 차선 가운데의 모양과 핸들 방향은 어떻게 함께 바뀌나요?')
        elif state in ('PRACTICE', 'RUNNING'):
            self.title.setText('지금은 체험자가 직접 운전해요')
            self.description.setText('AI는 계속 길과 가운데 선을 찾고 있어요. 직접 운전할 때는 메인 화면의 앞쪽 도로를 보세요.')
            self.detail.setText('여기 보이는 핸들은 AI의 계산 결과예요. 실제 차는 체험자의 조작으로 움직여요.')
        elif phase == 'RESULTS':
            self.title.setText('길을 찾고 핸들을 움직이는 과정을 배웠어요')
            self.description.setText('카메라 → 달릴 공간 → 위에서 본 도로 → 가운데 선 → 핸들 방향을 연결해 봤어요.')
            self.detail.setText('AI 운전 중에는 차선을 찾지 못하면 멈춰요. 운전 화면의 질문에서 다시 확인해 보세요.')
        else:
            self.title.setText('차가 길을 찾는 과정을 한눈에 봐요')
            self.description.setText('차량 전방 카메라 영상이 네 칸의 분석으로 이어져요. 시작하면 각 단계를 차례로 살펴볼 수 있어요.')
            self.detail.setText('전체 트랙은 체험자를 위한 안내 지도예요. AI의 입력은 차량 전방 카메라 영상이에요.')
        for key, field in (('overview', 'overview_fresh'), ('fitting', 'lesson_fresh'), ('segmentation', 'segmentation_fresh'), ('bev', 'bev_fresh')):
            status[field] = time.monotonic() - self.node.image_received_at.get(key, 0) < 1.5
        frames = dict(self.node.images)
        duel = status.get('duel') or {}
        if duel.get('started'):
            self.title.setText('함께 달리는 AI가 길을 찾는 과정이에요')
            self.description.setText('메인 화면은 내가 운전하는 시점, 이 화면은 AI 차량의 실시간 분석이에요.')
            for key in ('autonomy_steering', 'autonomy_fresh', 'lane_valid', 'lane_geometry'):
                status[key] = duel.get(key)
            for key, field in (('fitting','lesson_fresh'), ('segmentation','segmentation_fresh'), ('bev','bev_fresh')):
                frames[key] = self.node.images.get('ai_'+key)
                status[field] = time.monotonic() - self.node.image_received_at.get('ai_'+key, 0) < 1.5
        self.visual.refresh(status, frames, lesson)
