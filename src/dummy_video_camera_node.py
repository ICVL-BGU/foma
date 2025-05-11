#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from abstract_node import AbstractNode
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

class VideoCameraNode(AbstractNode):
    def __init__(self):
        super().__init__('fish_camera', 'Fish camera')
        self.camera_pub = rospy.Publisher('fish_camera/image', Image, queue_size=10)
        self.img = None
        #print(self.camera_capture.camera_configuration())
        self.bridge = CvBridge()
        rospy.on_shutdown(self.__on_shutdown)

    def run(self):
        while not rospy.is_shutdown():
            img = cv2.imread(r'/home/alex/ROS/src/foma/etc/vlc-record-2025-03-30-13h47m16s-test1.h264_000622.png', cv2.IMREAD_COLOR)
            
            img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.camera_pub.publish(img_msg)

            rospy.sleep(5)

    def __on_shutdown(self):
        #TODO add log
        # self.camera_capture.stop()
        pass
            


if __name__ == "__main__":
    rospy.init_node('video_camera_node')
    cam_handler = VideoCameraNode()
    cam_handler.run()
    rospy.spin()