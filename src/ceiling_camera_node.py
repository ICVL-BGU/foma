#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from abstract_node import AbstractNode

class CeilingCameraNode(AbstractNode):
    def __init__(self):
        super().__init__('ceiling_camera_node', 'Ceiling Camera Node (GStreamer LL)')
        self.url = "rtsp://admin:icvl2023@1.1.2.103:554"

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

        self.pub    = rospy.Publisher('ceiling_camera/image', Image, queue_size=1)
        self.bridge = CvBridge()
        rospy.on_shutdown(self._on_shutdown)

    def run(self):
        rospy.loginfo("CeilingCameraNode running…")
        rate = rospy.Rate(25)
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            else:
                rospy.logwarn_throttle(5, "Frame drop")
            rate.sleep()

    def _on_shutdown(self):
        if hasattr(self, 'cap'):
            self.cap.release()

if __name__ == "__main__":
    rospy.init_node('ceiling_camera', anonymous=False)
    CeilingCameraNode().run()
