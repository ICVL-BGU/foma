#!/usr/bin/env python3
import rospy
import numpy as np
import shutil
import subprocess
import threading
from sensor_msgs.msg import CompressedImage
from abstract_node import AbstractNode
from etc import jpeg


class FFmpegRTSPReader:
    """Reads an RTSP H.264 stream via ffmpeg, decoding on the GPU (h264_cuvid)
    when available and scaling to the publish size. A background thread drains
    the pipe continuously and keeps only the newest frame, mirroring the old
    GStreamer appsink (drop=true max-buffers=1) so latency never accumulates.
    """

    def __init__(self, url, width, height, use_gpu=True):
        self.__w = width
        self.__h = height
        self.__nbytes = width * height * 3
        self.__cond = threading.Condition()
        self.__latest = None
        self.__seq = 0  # increments on each new frame
        self.__running = True
        self.__p = None

        cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error",
               "-flags", "low_delay",  # cut decoder output reorder delay
               "-rtsp_transport", "tcp"]
        if use_gpu:
            # -resize downscales on the GPU during decode (no CPU -vf scale).
            cmd += ["-hwaccel", "cuda", "-c:v", "h264_cuvid",
                    "-resize", f"{width}x{height}"]
        cmd += ["-i", url]
        if not use_gpu:
            cmd += ["-vf", f"scale={width}:{height}"]
        cmd += ["-f", "rawvideo", "-pix_fmt", "bgr24", "pipe:1"]
        try:
            self.__p = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                        stderr=subprocess.DEVNULL,
                                        bufsize=self.__nbytes)
        except Exception:
            self.__p = None
            return

        self.__thread = threading.Thread(target=self.__drain, daemon=True)
        self.__thread.start()

    def __drain(self):
        while self.__running and self.__p is not None:
            buf = self.__p.stdout.read(self.__nbytes)
            if len(buf) < self.__nbytes:
                break  # stream ended / decoder died
            frame = np.frombuffer(buf, np.uint8).reshape((self.__h, self.__w, 3))
            with self.__cond:
                self.__latest = frame
                self.__seq += 1
                self.__cond.notify_all()

    def wait_until_ready(self, timeout_s=8.0):
        """Block until the first frame arrives (decoder confirmed working)."""
        with self.__cond:
            if self.__latest is not None:
                return True
            if self.__p is None or self.__p.poll() is not None:
                return False
            self.__cond.wait(timeout=timeout_s)
            return self.__latest is not None

    def read_new(self, last_seq, timeout=1.0):
        """Block until a frame newer than last_seq is available, then return the
        NEWEST one (ret, frame, seq). Skips intermediate frames so a slow
        consumer never builds backlog. (False, None, last_seq) on timeout."""
        with self.__cond:
            if self.__seq <= last_seq:
                self.__cond.wait(timeout=timeout)
            if self.__seq <= last_seq or self.__latest is None:
                return False, None, last_seq
            return True, self.__latest, self.__seq

    def release(self):
        with self.__cond:
            self.__running = False
            self.__cond.notify_all()
        if self.__p is not None:
            self.__p.kill()
            self.__p = None


class CeilingCameraNode(AbstractNode):
    def __init__(self):
        super().__init__('ceiling_camera_node', 'Ceiling Camera Node')
        # Prefer global ROOM_RTSP_URL (shared with room_recorder); allow
        # per-node ~rtsp_url override for tests.
        self.url = rospy.get_param('~rtsp_url', rospy.get_param('/ROOM_RTSP_URL'))

        _shape = rospy.get_param('/ROOM_CAMERA_FRAME_SHAPE')
        self._w, self._h = _shape[1], _shape[0]  # publish (w, h)
        self._fps = rospy.get_param('/ROOM_CAMERA_FPS')

        # GPU decode (h264_cuvid) first; fall back to CPU ffmpeg decode if the
        # NVIDIA path fails to produce frames (plugins missing or no GPU).
        gpu_ok = shutil.which("ffmpeg") is not None
        self.cap = FFmpegRTSPReader(self.url, self._w, self._h, use_gpu=gpu_ok)
        if gpu_ok and self.cap.wait_until_ready():
            rospy.loginfo("CeilingCamera: using ffmpeg h264_cuvid (GPU decode).")
        else:
            self.cap.release()
            rospy.logwarn("CeilingCamera: GPU decode unavailable, falling back to CPU.")
            self.cap = FFmpegRTSPReader(self.url, self._w, self._h, use_gpu=False)
            if not self.cap.wait_until_ready():
                rospy.logerr("Failed to open RTSP stream. Check ffmpeg & URL.")
                rospy.signal_shutdown("stream open failed")
                return

        if not jpeg.have_turbojpeg():
            rospy.logwarn("CeilingCamera: PyTurboJPEG unavailable, JPEG encode using cv2 (slower).")

        self.pub  = rospy.Publisher('ceiling_camera/image', CompressedImage, queue_size=5, tcp_nodelay=True)
        self._msg = CompressedImage()
        self._msg.format = "jpeg"
        rospy.on_shutdown(self._on_shutdown)

    def run(self):
        rospy.loginfo("CeilingCameraNode running…")
        # Publish on each new decoded frame (camera-paced, 25fps) instead of a
        # fixed Rate poll, removing up to one frame-period of phase lag. read_new
        # always returns the newest frame, so a slow encode drops frames rather
        # than building backlog.
        last_seq = -1
        while not rospy.is_shutdown():
            ret, frame, last_seq = self.cap.read_new(last_seq, timeout=1.0)
            if not ret:
                rospy.logwarn_throttle(5, "Frame drop")
                continue
            try:
                data = jpeg.encode(frame, quality=80)
            except Exception as e:
                rospy.logwarn_throttle(5, f"JPEG encode failed; skipping frame ({e}).")
                continue
            self._msg.header.stamp = rospy.Time.now()
            self._msg.data = data
            self.pub.publish(self._msg)

    def _on_shutdown(self):
        if hasattr(self, 'cap'):
            self.cap.release()


if __name__ == "__main__":
    rospy.init_node('ceiling_camera', anonymous=False)
    CeilingCameraNode().run()
