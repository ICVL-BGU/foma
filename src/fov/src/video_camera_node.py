#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from abstract_node import AbstractNode
import numpy as np
#import cv2
#from cv_bridge import CvBridge, CvBridgeError
from picamera2 import Picamera2

'''




picam2.start()

np_array = picam2.capture_array()
print(np_array)
picam2.capture_file("demo.jpg")
picam2.stop()
'''

class VideoCameraNode(AbstractNode):
    def __init__(self):
        super().__init__('fish_camera', 'Fish camera')
        self.camera_pub = rospy.Publisher('fish_camera/image', Image, queue_size=10)
        #self.camera_capture = cv2.VideoCapture(0)  # Open first USB camera
        self.camera_capture = Picamera2()
        config = self.camera_capture.create_still_configuration()
        config['main']['size'] = (1024,768)
        self.camera_capture.configure(config)
        self.camera_capture.start()
        self.img = None
        #print(self.camera_capture.camera_configuration())
        #self.bridge = CvBridge()
        rospy.on_shutdown(self.__on_shutdown)

    def run(self):
        while not rospy.is_shutdown():
            
            if True:#self._system_on:
                #rospy.logwarn("running")
                self.img = self.camera_capture.capture_array()

                # Create and populate the Image message
                img_msg = Image()
                img_msg.height, img_msg.width = self.img.shape[:2]
                img_msg.encoding = 'bgr8'  # Or 'rgb8' if your NumPy array is RGB
                img_msg.step = img_msg.width * 3  # Number of bytes in a row: width * num_channels
                img_msg.data = self.img.tobytes()  # In ROS Noetic, use tostring() instead of tobytes()
                img_msg.header.stamp = rospy.Time.now()  # Current time
                img_msg.header.frame_id = 'camera_frame'  # Change to your frame
                #rospy.logwarn(self.img)

                #rospy.logwarn(self.img.shape)
                #img_msg = self.bridge.cv2_to_imgmsg(self.img)
                self.camera_pub.publish(img_msg)

                rospy.sleep(0.05)

    def __on_shutdown(self):
        #TODO add log
        self.camera_capture.stop()
            


if __name__ == "__main__":
    rospy.init_node('video_camera_node')
    cam_handler = VideoCameraNode()
    cam_handler.run()
    print("hi")
    rospy.spin()