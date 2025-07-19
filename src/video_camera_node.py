#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from abstract_node import AbstractNode
import numpy as np
import cv2
from picamera2 import Picamera2

class VideoCameraNode(AbstractNode):
    def __init__(self):
        super().__init__('fish_camera', 'Fish camera')

        self.camera_pub = rospy.Publisher('fish_camera/image', CompressedImage, queue_size=1)
        self.camera_capture = Picamera2()
        config = self.camera_capture.create_preview_configuration(
            main={"size": (2464, 2464), "format": "BGR888"}
        )
        self.camera_capture.configure(config)
        self.camera_capture.start()

        self.__new_size = (640, 640)
        self.msg = CompressedImage()

        rospy.on_shutdown(self.__on_shutdown)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            frame = self.camera_capture.capture_array("main")
            frame = cv2.resize(frame, self.__new_size, interpolation=cv2.INTER_AREA)
            frame = np.rot90(frame, k=3)

            success, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            self.msg.data = buf.tobytes() if success else b''
            self.camera_pub.publish(self.msg)

            rate.sleep()

    def __on_shutdown(self):
        self.loginfo("Shutting down camera")
        try:
            self.camera_capture.stop()
        except Exception:
            pass

if __name__ == "__main__":
    rospy.init_node('video_camera_node')
    cam_handler = VideoCameraNode()
    cam_handler.run()
    rospy.spin()