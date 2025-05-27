#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from abstract_node import AbstractNode
import numpy as np
import cv2
from picamera2 import Picamera2

class VideoCameraNode(AbstractNode):
    def __init__(self):
        super().__init__('fish_camera', 'Fish camera')
        self.camera_pub = rospy.Publisher('fish_camera/image', Image, queue_size=10)
        self.camera_capture = Picamera2()
        config = self.camera_capture.create_video_configuration(
            main={"size": (2464, 2464), "format": "BGR888"}
        )
        
        config["controls"] = {
            "AwbEnable": True,        # Auto white balance
            "AwbMode": 0
        }

        self.camera_capture.configure(config)
        self.camera_capture.start()
        self.img = None

        width, height = config['main']['size']
        self.__new_size = (height//2, width//2)

        rospy.on_shutdown(self.__on_shutdown)

    def run(self):
        while not rospy.is_shutdown():
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

    def __on_shutdown(self):
        self.loginfo("Shutting down camera")
        self.camera_capture.stop()

if __name__ == "__main__":
    rospy.init_node('video_camera_node')
    cam_handler = VideoCameraNode()
    cam_handler.run()
    rospy.spin()