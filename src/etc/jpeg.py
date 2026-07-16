"""JPEG codec wrapper. Uses PyTurboJPEG (libjpeg-turbo) when available
(2-4x faster than cv2.imdecode/imencode on CPU); falls back to OpenCV
otherwise. Public API mirrors what call sites need.

Install for the speedup (lab PC):
    sudo apt install libturbojpeg
    pip install PyTurboJPEG
"""
import numpy as np
import cv2

try:
    from turbojpeg import TurboJPEG, TJPF_BGR, TJSAMP_420
    _tj = TurboJPEG()
    _HAVE_TJ = True
except Exception:
    _tj = None
    _HAVE_TJ = False


def have_turbojpeg() -> bool:
    return _HAVE_TJ


def decode(buf: bytes) -> np.ndarray:
    """Decode JPEG bytes -> BGR ndarray."""
    if _HAVE_TJ:
        return _tj.decode(buf, pixel_format=TJPF_BGR)
    arr = np.frombuffer(buf, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


def encode(bgr: np.ndarray, quality: int = 80) -> bytes:
    """Encode BGR ndarray -> JPEG bytes."""
    if _HAVE_TJ:
        return _tj.encode(bgr, quality=quality, pixel_format=TJPF_BGR,
                          jpeg_subsample=TJSAMP_420)
    ok, buf = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError('cv2.imencode failed')
    return buf.tobytes()
