#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension
from abstract_node import AbstractNode
import cv2
from cv_bridge import CvBridge, CvBridgeError
from fov.srv import Coordinate, CoordinateRequest, CoordinateResponse
import numpy as np
import pickle
import os

class CeilingCameraNode(AbstractNode):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('ceiling_camera_node', 'Ceiling Camera Node')

        
        self.loginfo("Ceiling Camera Node: Node started.")
        # Initialize the camera
        self.camera = cv2.VideoCapture("rtsp://admin:icvl2023@1.1.2.103:554?network-caching=200", cv2.CAP_FFMPEG)

        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Set up the image publisher
        self.image_pub = rospy.Publisher('ceiling_camera/image', Image, queue_size=10)
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Load calibration parameters
        # param_dir = r'/home/icvl/FOMA/src/fov/src/etc'
        self.resize_factor = 1

        # Shutdown behavior
        rospy.on_shutdown(self.__on_shutdown)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                ret, img = self.camera.read()
                if ret:
                    img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
                    self.image_pub.publish(img_msg)
                else:
                    self.logwarn(f"Camera: No image captured.")
            except CvBridgeError as e:
                rospy.logerr(f"Camera: CV Bridge Error - {e}")

            rate.sleep()

    def __on_shutdown(self):
        self.loginfo(f"Ceiling Camera: Releasing camera.")
        self.camera.release()

if __name__ == "__main__":
    # Initialize the node with a unique name based on the camera ID
    rospy.init_node(f'ceiling_camera', anonymous=False)

    node = CeilingCameraNode()
    node.run()
    rospy.spin()
