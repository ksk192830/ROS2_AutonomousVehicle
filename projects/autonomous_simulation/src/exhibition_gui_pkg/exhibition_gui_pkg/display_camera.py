"""Convert display frames off the GUI thread, keeping at most one waiting frame."""
import threading
import time
from PyQt5.QtCore import QObject, pyqtSignal


class FrameNotifier(QObject):
    ready = pyqtSignal()
    quit = pyqtSignal()


def display_image(message):
    """Copy the native RGB/BGR pixels once, without a full-frame CPU filter."""
    from PyQt5.QtGui import QImage
    formats = {'rgb8': QImage.Format_RGB888, 'bgr8': QImage.Format_BGR888}
    if message.encoding in formats:
        import cv2
        import numpy as np
        source = np.ndarray((message.height, message.width, 3), dtype=np.uint8,
                            buffer=message.data, strides=(message.step, 3, 1))
        rgba = cv2.cvtColor(source, cv2.COLOR_RGB2RGBA if message.encoding == 'rgb8'
                           else cv2.COLOR_BGR2RGBA)
        # A native 32-bit texture avoids QPainter expanding every RGB pixel
        # on the UI thread each time it uploads a frame.
        return QImage(rgba.data, message.width, message.height, int(rgba.strides[0]),
                      QImage.Format_RGBX8888).copy()
    from cv_bridge import CvBridge
    frame = CvBridge().imgmsg_to_cv2(message, desired_encoding='bgr8')
    return QImage(frame.data, message.width, message.height, int(frame.strides[0]),
                  QImage.Format_BGR888).copy()


class LatestFrameConverter:
    def __init__(self, convert, on_ready=None):
        self.convert = convert
        self.on_ready = on_ready
        self._condition = threading.Condition()
        self._pending = None
        self._stopped = False
        self.latest = None
        self.error = None
        self._thread = threading.Thread(target=self._run, name='front-camera-display', daemon=True)
        self._thread.start()

    def submit(self, message):
        with self._condition:
            if not self._stopped:
                self._pending = message
                self._condition.notify()

    def _run(self):
        sequence = 0
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._stopped or self._pending is not None)
                if self._stopped:
                    return
                message, self._pending = self._pending, None
            try:
                image = self.convert(message)
            except Exception as exc:
                self.error = str(exc)
                continue
            sequence += 1
            self.error = None
            # Publish an immutable snapshot; the GUI only reads this tuple.
            self.latest = (sequence, image, time.monotonic())
            if self.on_ready is not None:
                self.on_ready()

    def close(self):
        with self._condition:
            self._stopped = True
            self._pending = None
            self._condition.notify()
        self._thread.join(timeout=2)
