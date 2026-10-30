"""Spatial context for the competition venue, separate from AI perception."""
import math
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import Qt, QRectF, QPointF
from PyQt5.QtGui import QPainter, QPixmap, QTransform, QColor, QPen, QFont
from PyQt5.QtWidgets import QWidget, QSizePolicy
from exhibition_gui_pkg.dashboard import ImageCanvas
from exhibition_gui_pkg.camera_canvas import FrontCameraCanvas
from exhibition_gui_pkg.venue_view import project, HEIGHT, PITCH


class VenueMap(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(200, 150)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        path = Path(get_package_share_directory('simulation_pkg'))/'models/race_track/materials/textures/track.png'
        self.track = QPixmap(str(path)).transformed(QTransform().rotate(180))
        self.status = {}

    def refresh(self, status):
        self.status = status
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.SmoothPixmapTransform)
        width = min(self.width()-16, (self.height()-16)*4/3)
        area = QRectF((self.width()-width)/2, (self.height()-width*.75)/2, width, width*.75)
        p.drawPixmap(area, self.track, QRectF(self.track.rect()))
        def point(x,y):
            return QPointF(area.left()+(.5-y/16)*width, area.top()+(.5-x/12)*area.height())
        venue = self.status.get('venue') or {}
        pose = venue.get('pose')
        if pose and venue.get('active'):
            yaw = pose[4] if len(pose)>4 else 0.
            focus = point(pose[0], pose[1])
            p.setPen(QPen(QColor('#25d8d0'), 2))
            p.setBrush(QColor(37,216,208,55))
            radius = width*.11
            p.drawEllipse(focus, radius, radius)
            p.setBrush(QColor('#25d8d0'))
            p.drawEllipse(focus, 5, 5)
            p.drawLine(focus, point(pose[0]+1.2*math.cos(yaw), pose[1]+1.2*math.sin(yaw)))
        car = self.status.get('player_pose') or (-1.15,-6.8,0)
        at = point(*car[:2])
        p.setPen(QPen(QColor('#071321'), 2))
        p.setBrush(QColor('#ffd166'))
        p.drawEllipse(at, 7, 7)
        p.setFont(QFont('Noto Sans CJK KR', 12 if width>600 else 9, QFont.Bold))
        box = QRectF(max(area.left()+5, min(area.right()-96, at.x()-48)), at.y()+12, 96, 28)
        p.fillRect(box, QColor('#071321'))
        p.setPen(QColor('#ffd166'))
        p.drawText(box, Qt.AlignCenter, '자율주행차')


class VenueCameraCanvas(FrontCameraCanvas if os.environ.get('E2E_GPU_CAMERA_DISPLAY') == '1' else ImageCanvas):
    def __init__(self):
        super().__init__('대회장 둘러보기 화면을 준비하고 있어요')
        self.status = {}

    def refresh(self, status):
        self.status = status
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        if not isinstance(self, FrontCameraCanvas):
            self.paint_annotation()

    def paintGL(self):
        super().paintGL()
        self.paint_annotation()

    def paint_annotation(self):
        if self._pixmap is None:
            return
        car = self.status.get('player_pose')
        venue = self.status.get('venue') or {}
        pose = venue.get('pose')
        if not car or not pose or not venue.get('ready'):
            return
        point = (car[0]+.145*math.cos(car[2]), car[1]+.145*math.sin(car[2]), .675)
        at = project(point, pose)
        if at is None:
            return
        size = self._pixmap.size().scaled(self.size(), Qt.KeepAspectRatio)
        px = (self.width()-size.width())/2+at[0]*size.width()
        py = (self.height()-size.height())/2+at[1]*size.height()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setPen(QPen(QColor('#ffd166'), 2))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(px,py), 10, 10)
        box = QRectF(max(8,min(self.width()-180,px-84)), max(8,py-66), 170, 38)
        p.drawLine(QPointF(px,py-11), QPointF(box.center().x(),box.bottom()))
        p.setBrush(QColor('#071321'))
        p.drawRoundedRect(box, 8, 8)
        p.setFont(QFont('Noto Sans CJK KR', 12, QFont.Bold))
        p.drawText(box, Qt.AlignCenter, '차량의 전방 카메라')
