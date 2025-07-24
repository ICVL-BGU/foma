#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from abstract_node import AbstractNode
import cv2
from cv_bridge import CvBridge, CvBridgeError

class VideoCameraNode(AbstractNode):
    def __init__(self):
        super().__init__('ceiling_camera', 'Ceiling camera')
        self.camera_pub = rospy.Publisher('ceiling_camera/image', Image, queue_size=10)
        self.bridge = CvBridge()
        img = cv2.resize(cv2.imread(r'/home/alex/ros_ws/src/foma/etc/1bd0fb3f-frame_000969.png', cv2.IMREAD_COLOR), (0, 0), fx = 0.2, fy = 0.2)
        self.img_msg = self.bridge.cv2_to_imgmsg(img, "rgb8")

    def run(self):
        while not rospy.is_shutdown():
            try:
                # self.loginfo("Sending")
                self.camera_pub.publish(self.img_msg)
            except CvBridgeError as e:
                self.logerr(f"Error converting image: {e}")    
            rospy.sleep(5)      

if __name__ == "__main__":
    rospy.init_node('dummy_ceiling_camera')
    cam_handler = VideoCameraNode()
    cam_handler.run()
    rospy.spin()