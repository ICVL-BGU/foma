#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from abstract_node import AbstractNode
import cv2
from cv_bridge import CvBridge, CvBridgeError

class VideoCameraNode(AbstractNode):
    def __init__(self):
        super().__init__('fish_camera', 'Fish camera')
        self.camera_pub = rospy.Publisher('fish_camera/image', Image, queue_size=10)
        self.bridge = CvBridge()

    def run(self):
        while not rospy.is_shutdown():
            img = cv2.imread(r'/home/alex/ros_ws/src/foma/etc/vlc-record-2025-03-30-13h47m16s-test1.h264_000622.png', cv2.IMREAD_COLOR)
            try:
                img_msg = self.bridge.cv2_to_imgmsg(img, "rgb8")
                self.camera_pub.publish(img_msg)
            except CvBridgeError as e:
                self.logerr(f"Error converting image: {e}")    
            rospy.sleep(5)      

if __name__ == "__main__":
    rospy.init_node('video_camera_node')
    cam_handler = VideoCameraNode()
    cam_handler.run()
    rospy.spin()