#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from abstract_node import AbstractNode
from etc.settings import *

class CeilingCameraNode(AbstractNode):
    def __init__(self):
        super().__init__('ceiling_camera_node', 'Ceiling Camera Node')
        self.url = rospy.get_param('~rtsp_url')

        gst_str = (
            f"rtspsrc location={self.url} protocols=tcp latency=50 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
            "video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
        )

        self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            rospy.logerr("Failed to open GStreamer pipeline. Check plugins & URL.")
            rospy.signal_shutdown("pipeline open failed")
            return

        self.pub    = rospy.Publisher('ceiling_camera/image', CompressedImage, queue_size=5, tcp_nodelay=True)
        self._jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        self._publish_size = (ROOM_CAMERA_FRAME_SHAPE[1], ROOM_CAMERA_FRAME_SHAPE[0])  # (w, h) for cv2.resize
        self._msg = CompressedImage()
        self._msg.format = "jpeg"
        rospy.on_shutdown(self._on_shutdown)

    def run(self):
        rospy.loginfo("CeilingCameraNode running…")
        rate = rospy.Rate(ROOM_CAMERA_FPS)
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.resize(frame, self._publish_size, interpolation=cv2.INTER_AREA)
                ok, buf = cv2.imencode(".jpg", frame, self._jpeg_params)
                if ok:
                    self._msg.header.stamp = rospy.Time.now()
                    self._msg.data = buf.tobytes()
                    self.pub.publish(self._msg)
                else:
                    rospy.logwarn_throttle(5, "JPEG encode failed; skipping frame.")
            else:
                rospy.logwarn_throttle(5, "Frame drop")
            rate.sleep()

    def _on_shutdown(self):
        if hasattr(self, 'cap'):
            self.cap.release()

if __name__ == "__main__":
    rospy.init_node('ceiling_camera', anonymous=False)
    CeilingCameraNode().run()
