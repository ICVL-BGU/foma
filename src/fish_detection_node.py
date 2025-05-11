#!/usr/bin/env python3

import os

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
os.environ['NO_ALBUMENTATIONS_UPDATE'] = '1'

import rospy
from sensor_msgs.msg import Image
from abstract_node import AbstractNode
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from foma.msg import FishState  # Import the custom message
from geometry_msgs.msg import Twist, Vector3
# import sleap
from ultralytics import YOLO

class FishDetectionNode(AbstractNode):
    def __init__(self):
        super().__init__('fish_detection', 'Fish detection')
        # sleap.disable_preallocation()
        model_path = r"/home/icvl/ros_ws/src/foma/yolo_pose.pt" #\\wsl$\Ubuntu-20.04\home\alex\ROS\src\foma\yolo_pose.pt
        self.model = YOLO(model_path)
        self.direction = None
        self.img = None
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber('fish_camera/image', Image, self.read_image)
        self.fish_state_pub = rospy.Publisher('fish_detection/state', Twist, queue_size=10)


    # def run(self):
    #     while not rospy.is_shutdown():
    #         if self._system_on:
    #             if self.img is not None:
    #                 self.process_image()

    def read_image(self, img_msg: Image):
        try:
            self.img = self.bridge.imgmsg_to_cv2(img_msg)
            self.process_image()
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")

    def process_image(self):
        prediction = self.model(self.img, max_det=1, verbose=False)
        kp = prediction[0].keypoints
        # print(prediction[0].keypoints.shape==[1,8,2])
        # self.loginfo(f"shape: {kp}")
        if kp.shape[1] == 0:
            # self.logwarn(f"No fish detected, shape: {kp.shape}")
            # self.logwarn(f"Confidence too low: {confidences}")
            self.fish_state_pub.publish(Twist(linear = Vector3(0, 0, -1))) # fish not detected
            return
        # self.loginfo(f"Fish detected, shape: {kp.data.squeeze()}")
        points = kp.data[0][0], kp.data[0][3]
        # self.loginfo(f"Points: {points}")#, Confidences: {confidences}")
        dx, dy = points[0] - points[1]
        x, y = points[1]
        self.direction = Twist(linear = Vector3(x,y,0), angular = Vector3(dx, -dy, 0))
        # rospy.loginfo(f"dy = {dy}, dx = {dx}, y = {y}, x = {x}")
        self.fish_state_pub.publish(self.direction)

if __name__ == "__main__":
    rospy.init_node('fish_detection_node')
    fish_det = FishDetectionNode()
    # fish_det.run()
    rospy.spin()