#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from abstract_node import AbstractNode
from etc import jpeg


def _build_pipeline(url, width, height, use_nvidia):
    """Construct GStreamer pipeline string. NVIDIA path uses nvh264dec +
    nvvideoconvert for GPU decode and GPU scale; CPU path mirrors prior
    behavior with avdec_h264 + videoconvert + post-cv2 resize."""
    if use_nvidia:
        return (
            f"rtspsrc location={url} protocols=tcp latency=50 ! "
            "rtph264depay ! h264parse ! "
            "nvh264dec ! nvvideoconvert ! "
            f"video/x-raw,format=BGRx,width={width},height={height} ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )
    return (
        f"rtspsrc location={url} protocols=tcp latency=50 ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
        "video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
    )


class CeilingCameraNode(AbstractNode):
    def __init__(self):
        super().__init__('ceiling_camera_node', 'Ceiling Camera Node')
        # Prefer global ROOM_RTSP_URL (shared with room_recorder); allow
        # per-node ~rtsp_url override for tests.
        self.url = rospy.get_param('~rtsp_url', rospy.get_param('/ROOM_RTSP_URL'))

        _shape = rospy.get_param('/ROOM_CAMERA_FRAME_SHAPE')
        self._publish_size = (_shape[1], _shape[0])  # (w, h) for cv2.resize fallback
        self._fps = rospy.get_param('/ROOM_CAMERA_FPS')

        # Try NVIDIA hwaccel pipeline first; if it fails to open (plugins
        # missing or no GPU), fall back to CPU decoder.
        self._gpu_scaled = False
        self.cap = cv2.VideoCapture(_build_pipeline(self.url, _shape[1], _shape[0], True),
                                    cv2.CAP_GSTREAMER)
        if self.cap.isOpened():
            self._gpu_scaled = True
            rospy.loginfo("CeilingCamera: using NVIDIA nvh264dec + nvvideoconvert.")
        else:
            rospy.logwarn("CeilingCamera: NVIDIA pipeline unavailable, falling back to CPU.")
            self.cap = cv2.VideoCapture(_build_pipeline(self.url, _shape[1], _shape[0], False),
                                        cv2.CAP_GSTREAMER)
            if not self.cap.isOpened():
                rospy.logerr("Failed to open GStreamer pipeline. Check plugins & URL.")
                rospy.signal_shutdown("pipeline open failed")
                return

        if not jpeg.have_turbojpeg():
            rospy.logwarn("CeilingCamera: PyTurboJPEG unavailable, JPEG encode using cv2 (slower).")

        self.pub  = rospy.Publisher('ceiling_camera/image', CompressedImage, queue_size=5, tcp_nodelay=True)
        self._msg = CompressedImage()
        self._msg.format = "jpeg"
        rospy.on_shutdown(self._on_shutdown)

    def run(self):
        rospy.loginfo("CeilingCameraNode running…")
        rate = rospy.Rate(self._fps)
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                if not self._gpu_scaled:
                    frame = cv2.resize(frame, self._publish_size, interpolation=cv2.INTER_AREA)
                try:
                    data = jpeg.encode(frame, quality=80)
                except Exception as e:
                    rospy.logwarn_throttle(5, f"JPEG encode failed; skipping frame ({e}).")
                    rate.sleep()
                    continue
                self._msg.header.stamp = rospy.Time.now()
                self._msg.data = data
                self.pub.publish(self._msg)
            else:
                rospy.logwarn_throttle(5, "Frame drop")
            rate.sleep()

    def _on_shutdown(self):
        if hasattr(self, 'cap'):
            self.cap.release()


if __name__ == "__main__":
    rospy.init_node('ceiling_camera', anonymous=False)
    CeilingCameraNode().run()
