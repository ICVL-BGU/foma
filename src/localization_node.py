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
from etc.settings import *
import threading

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
        # Large queue to process all frames - output is recorded by csv_writer
        self.image_sub = rospy.Subscriber('ceiling_camera/image', Image, self.read_image, queue_size=200, buff_size=2**28)
        self.location_pub = rospy.Publisher('localization/location', FomaLocation, queue_size=10)
        self.location = FomaLocation()
        self.bridge = CvBridge()
        # Queue depth tracking
        self.pending_frames = 0
        self.frame_lock = threading.Lock()
        
    def read_image(self, img_msg: Image):
        try:
            with self.frame_lock:
                self.pending_frames += 1
                pending = self.pending_frames
            
            # Buffer lag diagnostic
            if img_msg.header.stamp.to_sec() > 0:
                msg_time = img_msg.header.stamp.to_sec()
                current_time = rospy.Time.now().to_sec()
                lag = current_time - msg_time
                if lag > 0.1:  # Log if lag > 100ms
                    rospy.logwarn_throttle(2.0, f"[Localization] Buffer: {pending} frames queued, lag: {lag:.3f}s")
            
            self.img = self.bridge.imgmsg_to_cv2(img_msg)
            self.process_image()
            
            with self.frame_lock:
                self.pending_frames -= 1
        except CvBridgeError as e:
            self.logerr(f"Error converting image: {e}")
            with self.frame_lock:
                self.pending_frames -= 1

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