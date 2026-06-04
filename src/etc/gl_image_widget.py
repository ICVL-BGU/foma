"""QOpenGLWidget that displays a numpy image frame. Resize/repaint scaling
runs on the GPU via QPainter on an OpenGL surface, so the CPU never
re-blits the frame on widget resize or expose events. The frame is uploaded
once per setFrame() call.

Use as a drop-in replacement for QLabel-based camera displays."""
from PyQt5.QtCore import Qt, QRect
from PyQt5.QtGui import QImage, QPainter
from PyQt5.QtWidgets import QOpenGLWidget, QSizePolicy


class GLImageWidget(QOpenGLWidget):
    def __init__(self, parent=None, fmt=QImage.Format_RGB888):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._image = None
        self._fmt = fmt

    def setFrame(self, arr):
        """arr: H x W x 3 uint8. Format selected at construction time
        determines whether bytes are interpreted as RGB or BGR."""
        if arr is None:
            return
        h, w, c = arr.shape
        # .copy() because arr may belong to a buffer the caller mutates.
        self._image = QImage(arr.data, w, h, c * w, self._fmt).copy()
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        if self._image is None:
            p.fillRect(self.rect(), Qt.black)
            return
        # KeepAspectRatio fit, centered.
        iw, ih = self._image.width(), self._image.height()
        ww, wh = self.width(), self.height()
        scale = min(ww / iw, wh / ih) if iw and ih else 1.0
        dw, dh = max(1, int(iw * scale)), max(1, int(ih * scale))
        x = (ww - dw) // 2
        y = (wh - dh) // 2
        p.setRenderHint(QPainter.SmoothPixmapTransform, True)
        p.drawImage(QRect(x, y, dw, dh), self._image)
