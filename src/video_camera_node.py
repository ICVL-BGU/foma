#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from abstract_node import AbstractNode
import numpy as np
import cv2
#from cv_bridge import CvBridge, CvBridgeError
from picamera2 import Picamera2

class VideoCameraNode(AbstractNode):
    def __init__(self):
        super().__init__('fish_camera', 'Fish camera')
        self.camera_pub = rospy.Publisher('fish_camera/image', Image, queue_size=10)
        #self.camera_capture = cv2.VideoCapture(0)  # Open first USB camera
        self.camera_capture = Picamera2()
        # config = self.camera_capture.create_still_configuration()
        # config['main']['size'] = (1640,1232) #(2304,1296)

        self.camera_capture.start_preview()
        print(self.camera_capture.camera_controls)
        config = self.camera_capture.create_video_configuration(
            main={"size": (3280, 2464), "format": "BGR888"}
        )
        
        # config['controls'] = {"AfMode": 0,"FrameRate": 15}  # Disable Auto Focus Mode (PDAF)
        config["controls"] = {
            "AwbEnable": True,        # Auto white balance
            "AwbMode": 0
            # "LensShadingEnable": True # Corrects for pink edges / vignetting
        }

        self.camera_capture.configure(config)
        self.camera_capture.start()
        self.img = None
        #print(self.camera_capture.camera_configuration())
        #self.bridge = CvBridge()

        width, height = config['main']['size']
        self.__new_size = (height//3, width//3)
        # self.loginfo(f"Size: {config['main']['size']}")

        rospy.on_shutdown(self.__on_shutdown)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            
            if True:#self._system_on:
                #rospy.logwarn("running")
                self.img = self.camera_capture.capture_array("main")
                self.img = np.rot90(self.img, k=3)
                self.img = cv2.resize(self.img, self.__new_size, interpolation=cv2.INTER_AREA)
                # Create and populate the Image message
                img_msg = Image()
                img_msg.height, img_msg.width = self.img.shape[:2]
                img_msg.encoding = 'bgr8'  # Or 'rgb8' if your NumPy array is RGB
                img_msg.step = img_msg.width * 3  # Number of bytes in a row: width * num_channels
                img_msg.data = self.img.tobytes()  # In ROS Noetic, use tostring() instead of tobytes()
                self.camera_pub.publish(img_msg)

                rate.sleep()

    def __on_shutdown(self):
        self.loginfo("Shutting down camera")
        self.camera_capture.stop()

if __name__ == "__main__":
    rospy.init_node('video_camera_node')
    cam_handler = VideoCameraNode()
    cam_handler.run()
    # print("hi")
    rospy.spin()