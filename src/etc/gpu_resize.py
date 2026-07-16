"""GPU resize wrapper. Uses cv2.cuda.resize when OpenCV was built with CUDA;
falls back to cv2.resize on CPU otherwise.

For small (<= 1280) targets the CPU/GPU break-even is close because of PCIe
upload/download cost, but it still removes the work from the GUI thread on
machines with a free GPU.
"""
import cv2

try:
    _HAVE_CUDA = cv2.cuda.getCudaEnabledDeviceCount() > 0
except Exception:
    _HAVE_CUDA = False


def have_cuda() -> bool:
    return _HAVE_CUDA


def resize(arr, w: int, h: int, interp=cv2.INTER_AREA):
    if _HAVE_CUDA:
        gpu = cv2.cuda_GpuMat()
        gpu.upload(arr)
        gpu = cv2.cuda.resize(gpu, (w, h), interpolation=interp)
        return gpu.download()
    return cv2.resize(arr, (w, h), interpolation=interp)


def fit_inside(src_w: int, src_h: int, max_w: int, max_h: int):
    """Returns (w, h) that fits src into max_* preserving aspect ratio."""
    if src_w <= 0 or src_h <= 0:
        return max_w, max_h
    scale = min(max_w / src_w, max_h / src_h)
    return max(1, int(src_w * scale)), max(1, int(src_h * scale))
