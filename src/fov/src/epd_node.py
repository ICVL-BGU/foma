#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16
from std_srvs.srv import Trigger
from fov.srv import EPDImage, EPDImageRequest, EPDImageResponse
from funcs.epd_display import display_image
from abstract_node import AbstractNode
import cv2
from cv_bridge import CvBridge, CvBridgeError
import requests

class EPDNode(AbstractNode):
    def __init__(self):
        super().__init__('epd', 'EPD')
        self.display_service = rospy.Service('epd/display', EPDImage, self.display)
        self.base_addr = '192.168.0.{}'

    def scan(self):
        self.epds = []
        for i in range(1,256):
            ans = touch(self.base_addr.format(i))
            if ans:
                self.epds.append[i]

    def display(self, data:EPDImageRequest):
        self.scan()
        if EPDImageRequest.id in self.epds:
            display_image(EPDImageRequest.image, f"{self.base_addr.format(id)}")
        else:
            return EPDImageResponse(success = False, message = "Couldn't find id")
        return EPDImageResponse()
    

if __name__ == "__main__":
    rospy.init_node('epd_node')
    rospy.loginfo("EPD Node: node created.")
    gui = EPDNode()
    rospy.loginfo("EPD Node: starting run.")
    gui.run()

    rospy.spin()