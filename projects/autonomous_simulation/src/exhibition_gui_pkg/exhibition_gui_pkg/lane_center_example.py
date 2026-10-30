"""Visitor-controlled illustration of the lane detector's row midpoint method.

This is an explicitly labeled teaching diagram, not live lane geometry or a
steering input. Its numbers describe positions in the illustration only.
"""

import math

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QSizePolicy, QWidget


CENTER_STEPS = (
    ('1  끝 찾기', '1. 달릴 공간의 양쪽 끝을 찾아요',
     '위에서 본 도로를 가로로 한 줄씩 살펴봐요.\n'
     '각 줄의 왼쪽 끝과 오른쪽 끝을 흰 점으로 표시해요. 노란 선은 지금 비교하는 한 줄이에요.'),
    ('2  반으로 나누기', '2. 양쪽 끝 사이를 반으로 나눠요',
     '양쪽 끝 사이를 똑같이 반으로 나누면 가운데예요. 그 자리에 초록 점을 찍어요.\n'
     '예: (왼쪽 100 + 오른쪽 300) ÷ 2 = 200\n'
     '숫자는 그림 속 가로 위치예요. 실제 도로의 미터 단위가 아니에요.'),
    ('3  선으로 잇기', '3. 가운데 점들을 선으로 이어요',
     '앞뒤 여러 위치에서 같은 계산을 반복해요. 중간점들의 작은 흔들림을 줄이고, 확인된 구간끼리 선으로 이어요.\n'
     '이 초록 선이 차가 따라갈 차선의 가운데예요. 길게 끊긴 구간은 억지로 잇지 않아요.'),
    ('4  가려진 곳', '4. 가려진 쪽은 차선 너비로 보완해요',
     '현재 영상이나 조금 전 확인한 차선 너비를 이용해, 보이는 경계에서 너비의 절반만큼 이동해요.\n'
     '초록 점선은 이렇게 보완한 구간이에요. 경계도 너비도 모르면 선을 만들지 않아요. 차 앞의 정보가 부족하면 AI는 멈춰요.'),
)


class LaneCenterExample(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(240, 160)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.step = 0

    def set_step(self, step):
        self.step = max(0, min(int(step), len(CENTER_STEPS) - 1))
        self.update()

    @staticmethod
    def left_at(y):
        return 100 + 62 * math.sin((300 - y) / 210)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor('#071321'))
        scale = min(self.width() / 460, self.height() / 520)
        painter.translate((self.width() - 460 * scale) / 2,
                          (self.height() - 520 * scale) / 2)
        painter.scale(scale, scale)
        ys = range(42, 479, 4)
        left = [QPointF(self.left_at(y), y) for y in ys]
        right = [QPointF(self.left_at(y) + 200, y) for y in ys]
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor('#193d59'))
        painter.drawPolygon(QPolygonF(left + list(reversed(right))))

        # In the final step only the left edge is visible in the upper region.
        # The clipped image boundary must never masquerade as a lane edge.
        if self.step == 3:
            painter.setBrush(QColor('#0d1c2d'))
            painter.drawRect(QRectF(295, 32, 140, 228))
            painter.setPen(QPen(QColor('#263f53'), 1.5))
            for y in range(40, 260, 22):
                painter.drawLine(QPointF(299, y), QPointF(430, min(y + 40, 260)))

        for y in range(60, 479, 38):
            x = self.left_at(y)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor('#f4f8ff'))
            painter.drawEllipse(QPointF(x, y), 4, 4)
            if self.step != 3 or y >= 270:
                painter.drawEllipse(QPointF(x + 200, y), 4, 4)

        # Highlight a measured row whose positions are exactly 100 and 300.
        painter.setPen(QPen(QColor('#f0ca73'), 2))
        painter.drawLine(QPointF(100, 300), QPointF(300, 300))
        for x in (100, 300):
            painter.drawLine(QPointF(x, 290), QPointF(x, 310))
            painter.setBrush(QColor('#f4f8ff'))
            painter.drawEllipse(QPointF(x, 300), 5, 5)

        font = QFont('Noto Sans CJK KR')
        font.setPixelSize(round(max(18, 13 / scale)))
        painter.setFont(font)
        painter.setPen(QColor('#9eb0c8'))
        painter.drawText(QRectF(70, 3, 110, 30), Qt.AlignCenter, '왼쪽 끝')
        painter.drawText(QRectF(280, 3, 150, 30), Qt.AlignCenter,
                         '안 보이는 부분' if self.step == 3 else '오른쪽 끝')
        painter.drawText(QRectF(125, 484, 170, 30), Qt.AlignCenter, '차에 가까운 쪽 ↓')

        if self.step >= 1:
            painter.setPen(QColor('#f0ca73'))
            if self.step == 1:
                for x, text in ((100, '100'), (200, '200'), (300, '300')):
                    painter.drawText(QRectF(x - 35, 312, 70, 34), Qt.AlignCenter, text)
            if self.step in (1, 3):
                painter.drawText(QRectF(100, 255, 100, 32), Qt.AlignCenter, '100칸')
                painter.drawText(QRectF(200, 255, 100, 32), Qt.AlignCenter, '100칸')
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor('#40e4a0'))
            painter.drawEllipse(QPointF(200, 300), 8, 8)

        if self.step >= 2:
            for begin, end, inferred in ((42, 260, self.step == 3), (260, 479, False)):
                points = QPolygonF([QPointF(self.left_at(y) + 100, y)
                                    for y in range(begin, end + 1, 2)])
                painter.setPen(QPen(QColor('#40e4a0'), 4,
                                    Qt.DashLine if inferred else Qt.SolidLine))
                painter.setBrush(Qt.NoBrush)
                painter.drawPolyline(points)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor('#40e4a0'))
            for y in range(60, 479, 38):
                if self.step != 3 or y >= 270:
                    painter.drawEllipse(QPointF(self.left_at(y) + 100, y), 6, 6)
        if self.step == 3:
            # Carry half the measured width to a row with only one visible edge.
            x, y = self.left_at(170), 170
            painter.setPen(QPen(QColor('#f0ca73'), 2))
            painter.drawLine(QPointF(x, y), QPointF(x + 100, y))
            painter.drawText(QRectF(x, y - 38, 100, 32), Qt.AlignCenter, '100칸')
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor('#f4f8ff'))
            painter.drawEllipse(QPointF(x, y), 5, 5)
            painter.setPen(QPen(QColor('#40e4a0'), 3))
            painter.setBrush(QColor('#071321'))
            painter.drawEllipse(QPointF(x + 100, y), 6, 6)
