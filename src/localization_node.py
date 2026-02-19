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
import json
import numpy as np
from etc.settings import *

LIDAR_TAG = 1

class LocalizationNode(AbstractNode):
    def __init__(self):
        super().__init__('localization', 'FOMA Localization')

        detection_model_path = rospy.get_param('~detection_model_path')
        mapper_path = rospy.get_param('~mapper_path')
        self.detection_model = YOLO(detection_model_path)
        with open(mapper_path, 'r') as f:
            self.mapper_params = json.load(f)
        self.img = None
        self.image_sub = rospy.Subscriber('ceiling_camera/image', Image, self.read_image, queue_size=1, buff_size=2**25, tcp_nodelay=True)
        self.location_pub = rospy.Publisher('localization/location', FomaLocation, queue_size=10, tcp_nodelay=True)
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
            # Index may return tensor/np array on CUDA; normalize to host Python floats
            box = lidar_boxes[max_conf_idx]

            # If box is a torch tensor on device (has cpu()), bring to CPU then to numpy
            try:
                # torch.Tensor -> numpy
                if hasattr(box, 'cpu'):
                    box_vals = box.cpu().numpy()
                else:
                    box_vals = box
            except Exception:
                # Fallback: try to convert elements individually
                box_vals = [getattr(box[i], 'cpu', lambda: box[i])() if hasattr(box[i], 'cpu') else box[i] for i in range(len(box))]

            x = float(box_vals[0])
            y = float(box_vals[1])

            return x, y
        
        return None

    def _map(self, x, y):
        """Complete transformation: image coordinates -> undistort -> world coordinates."""
        # Extract parameters
        cx = self.mapper_params['cx']
        cy = self.mapper_params['cy']
        f = self.mapper_params['f']
        
        # Step 1: Undistort fisheye point
        # Normalize by focal length and center
        x_d = (x - cx) / f
        y_d = (y - cy) / f
        r_d = np.sqrt(x_d**2 + y_d**2)
        
        # Apply distortion model (equisolid)
        arg = np.clip(r_d / 2, -1.0, 1.0)
        theta = 2 * np.arcsin(arg)
        
        # Convert to undistorted coordinates
        if r_d > 1e-9:
            scale = np.tan(theta) / r_d
        else:
            scale = 1.0
        x_u = x_d * scale
        y_u = y_d * scale
        
        # Convert back to pixel coordinates
        undistorted_x = x_u * f + cx
        undistorted_y = y_u * f + cy
        
        # Step 2: Apply homography
        # Convert to homogeneous coordinates
        H = np.array(self.mapper_params['homography'])
        point_h = np.array([undistorted_x, undistorted_y, 1.0])
        
        # Apply homography
        world_h = H @ point_h
        
        # Convert back from homogeneous coordinates
        x_w = world_h[0] / world_h[2]
        y_w = world_h[1] / world_h[2]
        
        return x_w, y_w

if __name__ == "__main__":
    rospy.init_node('localization_node')
    LocalizationNode()
    rospy.spin()