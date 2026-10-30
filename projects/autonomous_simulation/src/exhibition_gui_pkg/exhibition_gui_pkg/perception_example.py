"""Synchronized live frames and public examples for a visitor-paced lesson.

This is presentation only: it never publishes images or vehicle commands.
"""

import json
import time
from pathlib import Path

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QSizePolicy, QWidget


class PerceptionExample(QWidget):
    DURATION = {'segmentation': 11.0, 'bev': 13.0}
    CHOICES = {'segmentation': ('지금 달리는 도로', '책상 위 물건'), 'bev': ('지금 달리는 도로', '스도쿠 종이')}
    PUBLIC_GUIDES = {
        'segmentation': ('컵과 노트북을 색으로 구분해요',
                         '도로를 구분하는 방법을 친숙한 물건으로 살펴볼게요.\n'
                         '사진 속 물건을 한 색씩 칠하면 무엇이 어디에 있는지 쉽게 알아볼 수 있어요.'),
        'bev': ('비스듬한 종이를 반듯하게 펼쳐요',
                '도로를 위에서 본 모습으로 바꾸는 원리를 종이 사진으로 살펴볼게요.\n'
                '네 모서리를 맞춰 펴면 기울어졌던 칸들을 비교하기 쉬워져요.'),
    }

    def __init__(self):
        super().__init__()
        self.setMinimumSize(240, 160)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.mode = None
        self.variant = 0
        self.elapsed = 0.0
        self.paused = False
        self.last_tick = time.monotonic()
        self.images = {}
        self.source = None
        self.error = ''
        self.live_stamp = None
        self.live_received_at = 0.0
        try:
            folder = Path(get_package_share_directory('exhibition_gui_pkg')) / 'assets/examples'
            config = json.loads((folder / 'recording.json').read_text())
            for key in ('camera', 'segmentation', 'bev'):
                frame = cv2.imread(str(folder / f'source-{key}.png'))
                if frame is None:
                    raise ValueError('Missing example image')
                self.images[key] = self.qimage(frame)
                if key == 'segmentation':
                    self.source = frame
            height, width = self.source.shape[:2]
            self.source_quad = np.asarray(config['persp_src'], np.float32).reshape(4, 2) * [width, height]
            self.target_quad = np.asarray(config['persp_dst'], np.float32).reshape(4, 2) * [width, height]
            for key, filename in (('desk', 'desk-original.jpg'), ('desk_mask', 'desk-segmented.png'), ('paper', 'sudoku.png')):
                frame = cv2.imread(str(folder / filename))
                if frame is None:
                    raise ValueError(f'Missing example image: {filename}')
                self.images[key] = self.qimage(frame)
                if key == 'paper':
                    self.paper = frame
            # Corners of the grid in the unmodified 558 x 563 OpenCV sample.
            self.paper_source = np.float32([[34, 515], [520, 520], [491, 70], [72, 85]])
            self.paper_target = np.float32([[30, 531], [528, 531], [528, 33], [30, 33]])
        except (OSError, ValueError, KeyError) as exc:
            self.error = str(exc)
        self.timer = QTimer(self)
        self.timer.setInterval(12)
        self.timer.setTimerType(Qt.PreciseTimer)
        self.timer.timeout.connect(self.tick)

    @staticmethod
    def qimage(frame):
        height, width = frame.shape[:2]
        return QImage(frame.data, width, height, int(frame.strides[0]), QImage.Format_BGR888).copy()

    def set_mode(self, mode):
        if mode == self.mode:
            return
        self.mode = mode if mode in self.DURATION else None
        self.variant = 0
        self.restart()
        if self.mode and self.isVisible():
            self.timer.start()
        else:
            self.timer.stop()

    def set_variant(self, index):
        if self.mode and index in (0, 1) and self.variant != index:
            self.variant = index
            self.restart()

    def set_live_frames(self, bundle):
        if bundle is None or self.paused or bundle[0] == self.live_stamp:
            return
        self.live_stamp, images, self.source, self.live_received_at = bundle
        self.images.update(images)
        self.update()

    def restart(self):
        self.elapsed = 0.0
        self.paused = False
        self.last_tick = time.monotonic()
        self.update()

    def toggle_pause(self):
        self.paused = not self.paused
        self.last_tick = time.monotonic()
        self.update()
        return self.paused

    def showEvent(self, event):
        super().showEvent(event)
        self.last_tick = time.monotonic()
        if self.mode:
            self.timer.start()

    def hideEvent(self, event):
        self.timer.stop()
        super().hideEvent(event)

    def tick(self):
        now = time.monotonic()
        if self.mode and not self.paused:
            self.elapsed = (self.elapsed + min(now - self.last_tick, .2)) % self.DURATION[self.mode]
            self.update()
        self.last_tick = now

    def presentation(self):
        """Image and explanation for the current, pausable animation time."""
        t = self.elapsed
        if self.mode == 'segmentation':
            reveal = max(0.0, min(1.0, (t - 2.0) / 4.0))
            if t < 2:
                title = '색을 입히기 전, 카메라가 찍은 도로예요'
            elif t < 6:
                title = 'AI가 찾은 달릴 공간에 색이 나타나요'
            else:
                title = '달릴 공간이 두 색으로 구분됐어요'
            if self.variant:
                title = ('사진 속 물건을 먼저 살펴봐요' if t < 2 else
                         '물건마다 다른 색이 나타나요' if t < 6 else '물건의 모양이 색으로 구분됐어요')
            return self.images['desk' if self.variant else 'camera'], title, reveal, None
        progress = max(0.0, min(1.0, (t - 3.0) / 5.0))
        # Smooth the explanatory unfolding; the final projection uses the
        # same calibrated lane-detector parameters as the live BEV output.
        fraction = progress * progress * (3 - 2 * progress)
        source_quad = self.paper_source if self.variant else self.source_quad
        target_quad = self.paper_target if self.variant else self.target_quad
        source = self.paper if self.variant else self.source
        quad = source_quad * (1 - fraction) + target_quad * fraction
        if t < 3:
            title = '펼칠 종이의 네 모서리예요' if self.variant else '펼칠 도로의 네 모서리예요'
            result = self.images['paper' if self.variant else 'segmentation']
        elif t < 8 or self.variant:
            title = ('네 모서리를 맞춰 종이를 반듯하게 펴요' if t < 8 else '정면에서 본 듯한 사각형이 됐어요') if self.variant else '도로를 위에서 본 모양으로 펼쳐요'
            height, width = source.shape[:2]
            matrix = cv2.getPerspectiveTransform(source_quad.astype(np.float32), quad.astype(np.float32))
            result = self.qimage(cv2.warpPerspective(source, matrix, (width, height)))
        else:
            title = '위에서 본 도로가 완성됐어요'
            result = self.images['bev']
        return result, title, None, quad

    def explanation(self):
        """Keep reading text fixed while the visual transformation repeats."""
        if self.error:
            return '예시 연결 확인 중', '예시를 다시 선택해 주세요.', ''
        if self.mode == 'segmentation' and self.variant:
            stage = '물건마다 색을 나눠요'
            detail = ('검정은 노트북, 파랑은 컵, 노랑은 책상, 빨강은 전선이에요.\n'
                      '사람이 정답을 칠해 둔 학습 예시예요.')
            credit = 'Martin Thoma · CC0'
        elif self.mode == 'bev' and self.variant:
            stage = '비스듬한 사진을 반듯하게 펴요'
            detail = ('노란 네 모서리가 사각형으로 맞춰지는 모습을 보세요.\n'
                      '도로 사진도 같은 원리로 위에서 본 모습으로 바꿔요.')
            credit = 'OpenCV · Apache 2.0'
        elif self.mode == 'segmentation':
            stage = '색이 칠해진 부분이 달릴 공간이에요'
            detail = ('파랑과 분홍은 AI가 구분한 서로 다른 주행 공간이에요.\n'
                      '차가 움직여도 색이 흰 차선 사이를 따라가는지 보세요.')
            credit = ''
        else:
            stage = '같은 도로를 위에서 본 모습으로 바꿔요'
            detail = ('노란 네 점을 따라가 보세요. 멀리 있어 좁아 보이던 도로가 펼쳐져요.\n'
                      '다른 카메라가 아니라, 앞쪽 카메라 영상을 계산해서 바꾼 모습이에요.')
            credit = ''
        return stage, detail, credit

    @staticmethod
    def image_rect(image, bounds):
        ratio = min(bounds.width() / image.width(), bounds.height() / image.height())
        return QRectF(bounds.center().x() - image.width() * ratio / 2,
                      bounds.center().y() - image.height() * ratio / 2,
                      image.width() * ratio, image.height() * ratio)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)
        painter.fillRect(self.rect(), QColor('#071321'))
        if self.live_stamp is not None and not self.variant and not self.paused and time.monotonic() - self.live_received_at > 1.5:
            painter.setPen(QColor('#9eb0c8'))
            painter.drawText(self.rect(), Qt.AlignCenter, '새 카메라·AI 결과를 기다리고 있어요')
            return
        if self.error or not self.mode:
            painter.setPen(QColor('#9eb0c8'))
            painter.drawText(self.rect(), Qt.AlignCenter, '예시를 준비하지 못했어요')
            return
        frame, title, reveal, quad = self.presentation()
        available = QRectF(8, 8, self.width() - 16, self.height() - 24)
        if self.variant:
            half = (available.width() - 40) / 2
            left = QRectF(available.left(), available.top(), half, available.height())
            original = self.images['desk' if self.mode == 'segmentation' else 'paper']
            painter.drawImage(self.image_rect(original, left), original)
            available = QRectF(left.right() + 40, left.top(), half, left.height())
            x, y = left.right() + 20, left.center().y()
            painter.setPen(QPen(QColor('#25d8d0'), 3))
            painter.drawLine(QPointF(x - 12, y), QPointF(x + 12, y))
            painter.drawLine(QPointF(x + 12, y), QPointF(x + 4, y - 7))
            painter.drawLine(QPointF(x + 12, y), QPointF(x + 4, y + 7))
        target = self.image_rect(frame, available)
        ratio = target.width() / frame.width()
        painter.drawImage(target, frame)
        if reveal is not None:
            painter.save()
            painter.setClipRect(QRectF(target.left(), target.top(), target.width() * reveal, target.height()))
            painter.drawImage(target, self.images['desk_mask' if self.variant else 'segmentation'])
            painter.restore()
            if 0 < reveal < 1:
                x = target.left() + target.width() * reveal
                painter.setPen(QPen(QColor('#f0ca73'), 3))
                painter.drawLine(QPointF(x, target.top()), QPointF(x, target.bottom()))
        if quad is not None:
            points = QPolygonF([QPointF(target.left() + x * ratio, target.top() + y * ratio) for x, y in quad])
            painter.setPen(QPen(QColor('#f0ca73'), 3, Qt.DashLine))
            painter.setBrush(Qt.NoBrush)
            painter.drawPolygon(points)
            painter.setBrush(QColor('#f0ca73'))
            for point in points:
                painter.drawEllipse(point, 4, 4)
        painter.fillRect(QRectF(0, self.height() - 4, self.width(), 4), QColor('#203c58'))
        painter.fillRect(QRectF(0, self.height() - 4, self.width() * self.elapsed / self.DURATION[self.mode], 4), QColor('#25d8d0'))
