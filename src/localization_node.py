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
import json
import numpy as np
import torch
from etc.settings import *

LIDAR_TAG = 1

class LocalizationNode(AbstractNode):
    def __init__(self):
        super().__init__('localization', 'FOMA Localization')

        detection_model_path = r"/home/icvl/ros_ws/src/foma/models/foma_detection.pt"
        mapper_path = r"/home/icvl/ros_ws/src/foma/models/image_world_mapper.json"
        self.detection_model = YOLO(detection_model_path)
        with open(mapper_path, 'r') as f:
            self.mapper_params = json.load(f)
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
        
        result = self.__detect_lidar()
        
        if result is not None:
            x_i, y_i = result
            
            x_w, y_w = self._map(x_i * ROOM_CAMERA_FRAME_SHAPE[1], y_i * ROOM_CAMERA_FRAME_SHAPE[0])

            self.location.image = Point(x_i, y_i, 0)
            self.location.world = Point(x_w / ROOM_FLOOR_MAP_SHAPE[1], y_w / ROOM_FLOOR_MAP_SHAPE[1], 0)
        
        self.location_pub.publish(self.location)

    def __detect_lidar(self):
        prediction = self.detection_model.track(self.img, verbose=False)
        indices = prediction[0].boxes.data[:, -1] == LIDAR_TAG
        
        if indices.any():
            # Filter boxes by LIDAR_TAG
            lidar_boxes = prediction[0].boxes.xywhn[indices]
            lidar_conf = prediction[0].boxes.conf[indices]
            
            # Find the detection with the highest confidence
            max_conf_idx = lidar_conf.argmax()
            x, y, _, _ = lidar_boxes[max_conf_idx]
            
            return x, y
        
        return None

    def _map(self, x, y):
        """Complete transformation: image coordinates -> undistort -> world coordinates."""
        # Use torch tensors so we correctly handle CUDA tensors from detection
        # and avoid implicit numpy conversions. Returns Python floats for ROS messages.
        # Determine device from inputs (if tensors) or default to CPU.
        if isinstance(x, torch.Tensor):
            device = x.device
        elif isinstance(y, torch.Tensor):
            device = y.device
        else:
            device = torch.device('cpu')

        dtype = torch.float32

        # Convert inputs and mapper params to tensors on the chosen device
        x_t = torch.as_tensor(x, device=device, dtype=dtype)
        y_t = torch.as_tensor(y, device=device, dtype=dtype)
        cx = torch.as_tensor(self.mapper_params['cx'], device=device, dtype=dtype)
        cy = torch.as_tensor(self.mapper_params['cy'], device=device, dtype=dtype)
        f = torch.as_tensor(self.mapper_params['f'], device=device, dtype=dtype)

        # Step 1: Undistort fisheye point
        x_d = (x_t - cx) / f
        y_d = (y_t - cy) / f
        r_d = torch.sqrt(x_d * x_d + y_d * y_d)

        # Apply distortion model (equisolid)
        arg = torch.clamp(r_d / 2.0, -1.0, 1.0)
        theta = 2.0 * torch.asin(arg)

        # Compute scale safely (avoid division by zero)
        eps = 1e-9
        scale = torch.where(r_d > eps, torch.tan(theta) / r_d, torch.tensor(1.0, device=device, dtype=dtype))
        x_u = x_d * scale
        y_u = y_d * scale

        # Convert back to pixel coordinates
        undistorted_x = x_u * f + cx
        undistorted_y = y_u * f + cy

        # Step 2: Apply homography
        H = torch.as_tensor(self.mapper_params['homography'], device=device, dtype=dtype)
        point_h = torch.stack([undistorted_x, undistorted_y, torch.tensor(1.0, device=device, dtype=dtype)])
        world_h = H @ point_h

        # Convert back from homogeneous coordinates
        x_w = world_h[0] / world_h[2]
        y_w = world_h[1] / world_h[2]

        # Return plain Python floats (safe for ROS message fields)
        return float(x_w.cpu().item()), float(y_w.cpu().item())

if __name__ == "__main__":
    rospy.init_node('localization_node')
    LocalizationNode()
    rospy.spin()