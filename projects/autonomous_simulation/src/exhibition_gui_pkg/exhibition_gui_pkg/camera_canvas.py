"""Accelerated presentation shared by driving and venue cameras."""
import os
from PyQt5.QtCore import Qt, QRectF
from PyQt5.QtGui import QPainter, QPainterPath, QColor, QFont, QPen, QSurfaceFormat
from PyQt5.QtWidgets import QOpenGLWidget, QSizePolicy
from exhibition_gui_pkg.dashboard import ImageCanvas, MUTED
FONT = 'Noto Sans CJK KR'


class FrontCameraCanvas(QOpenGLWidget):
    """Upload the original display frame once; scale it on the GPU at paint time."""
    def __init__(self, placeholder):
        super().__init__()
        fmt = QSurfaceFormat()
        fmt.setSwapInterval(0)
        self.setFormat(fmt)
        self.setMinimumSize(120, 110)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.placeholder = placeholder
        # The shared LearningVisual.frame helper clears this slot on stale data.
        self._pixmap = None
        self._sequence = -1
        self._painted_sequence = None
        self.frames_presented = 0

    def set_frame(self, sequence, image):
        if image is None or sequence == self._sequence:
            return
        self._sequence = sequence
        self._pixmap = image
        self.update()

    def paintGL(self):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)
        bounds = QRectF(self.rect()).adjusted(1, 1, -1, -1)
        clip = QPainterPath()
        clip.addRoundedRect(bounds, 10, 10)
        painter.setClipPath(clip)
        painter.fillRect(bounds, QColor('#071321'))
        frame = self._pixmap
        if frame is None or frame.isNull():
            painter.setPen(QColor(MUTED))
            painter.setFont(QFont(FONT, 11))
            painter.drawText(bounds, Qt.AlignCenter, self.placeholder)
        else:
            scale = min(bounds.width() / frame.width(), bounds.height() / frame.height())
            width, height = frame.width() * scale, frame.height() * scale
            painter.drawImage(QRectF((self.width() - width) / 2,
                (self.height() - height) / 2, width, height), frame)
            if self._painted_sequence != self._sequence:
                self._painted_sequence = self._sequence
                self.frames_presented += 1
        painter.setClipping(False)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor('#21466d'), 1))
        painter.drawRoundedRect(bounds, 10, 10)
        painter.end()


def front_camera_canvas(placeholder):
    # Launch enables this only after a successful real NVIDIA context probe.
    # Offscreen QA and machines without accelerated GL retain the raster path.
    accelerated = os.environ.get('E2E_GPU_CAMERA_DISPLAY') == '1'
    return FrontCameraCanvas(placeholder) if accelerated else ImageCanvas(placeholder)

