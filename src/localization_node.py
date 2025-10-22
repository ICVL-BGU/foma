#!/usr/bin/env python3

import os

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
os.environ['NO_ALBUMENTATIONS_UPDATE'] = '1'

import rospy
from sensor_msgs.msg import Image
from abstract_node import AbstractNode
from cv_bridge import CvBridge, CvBridgeError
from foma.msg import FomaLocation
from geometry_msgs.msg import Point
from ultralytics import YOLO
import joblib
import pickle

LIDAR_TAG = 1

class LocalizationNode(AbstractNode):
    def __init__(self):
        super().__init__('localization', 'FOMA Localization')

        detection_model_path = r"/home/icvl/ros_ws/src/foma/models/foma_detection.pt" # /home/icvl/ros_ws/src/foma/yolo_pose.pt" OR /home/alex/ROS/src/foma/yolo_pose.pt
        localization_model_path = r"/home/icvl/ros_ws/src/foma/models/foma_localization.pkl"
        self.detection_model = YOLO(detection_model_path)
        with open(localization_model_path, 'rb') as f:
            self.localization_model = pickle.load(f)
        self.img = None
        self.image_sub = rospy.Subscriber('ceiling_camera/image', Image, self.read_image)
        self.location_pub = rospy.Publisher('localization/location', FomaLocation, queue_size=10)
        self.location = FomaLocation()
        self.bridge = CvBridge()
        
    def read_image(self, img_msg: Image):
        try:
            self.img = self.bridge.imgmsg_to_cv2(img_msg)
            self.process_image()
        except CvBridgeError as e:
            self.logerr(f"Error converting image: {e}")

    def process_image(self):
        timestamp = rospy.Time.now()
        self.location.header.stamp = timestamp
        prediction = self.detection_model.track(self.img, verbose=False) # max_det=1,
        indices = prediction[0].boxes.data[:, -1] == LIDAR_TAG
        
        if indices.any():
            bounding_box = prediction[0].boxes.xywhn[indices][0]
            x_i, y_i, _, _ = bounding_box
            
            x_w, y_w = self.localization_model.predict(bounding_box.cpu().reshape(1, -1))[0]

            self.location.image = Point(x_i, y_i, 0)
            self.location.world = Point(x_w, y_w, 0)

        self.location_pub.publish(self.location)

if __name__ == "__main__":
    rospy.init_node('localization_node')
    LocalizationNode()
    rospy.spin()