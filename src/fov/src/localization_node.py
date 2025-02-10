#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from abstract_node import AbstractNode
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from fov.msg import FishState  # Import the custom message
from geometry_msgs.msg import Point


class LocalizationNode(AbstractNode):
    def __init__(self):
        super().__init__('localization', 'Localization')
        self.image_sub = rospy.Subscriber('ceiling_camera/image', Image, self.read_image)
        self.location_pub = rospy.Publisher('localization/location', Point, queue_size=10)
        self.location = None
        self.img = None
        self.bridge = CvBridge()
        rospy.loginfo("LocalizationNode initialized")

    def run(self):
        rospy.loginfo("LocalizationNode running")
        while not rospy.is_shutdown():
            if True: #self._system_on:
                if self.img is not None:
                    # rospy.loginfo("Processing image")
                    self.process_image()
                if self.location is not None:
                    rospy.loginfo(f"Publishing location: {self.location}")
                    self.location_pub.publish(self.location)

    def read_image(self, img_msg: Image):
        try:
            self.img = self.bridge.imgmsg_to_cv2(img_msg)
            # rospy.loginfo("Image received and converted")
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")

    def process_image(self):
        
        rospy.loginfo("Processing image")
        
        # Convert to grayscale
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian Blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Use adaptive thresholding or Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)
        
        # Find contours in the edge-detected image
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        lidar_center = None
        lidar_ellipse = None
        
        # Copy image for visualization
        vis_image = self.img.copy()
        
        for contour in contours:
            if len(contour) >= 5:  # Minimum points required to fit an ellipse
                ellipse = cv2.fitEllipse(contour)
                (x, y), (major_axis, minor_axis), angle = ellipse
                
                # Approximate the expected size of the lidar
                expected_minor = 10 / 2  # radius in cm
                expected_major = expected_minor * (215 - 95) / (95)  # Adjust for perspective
                
                if 0.8 * expected_major < major_axis < 1.2 * expected_major and \
                0.8 * expected_minor < minor_axis < 1.2 * expected_minor:
                    lidar_center = (x, y)
                    lidar_ellipse = ellipse
                    rospy.loginfo(f"Lidar detected at pixel coordinates: {lidar_center}")
                    break  # Assume one lidar per image
        
        if lidar_center:
            rospy.loginfo("Lidar detected")
            
            # Draw detected lidar ellipse in red
            cv2.ellipse(vis_image, lidar_ellipse, (0, 0, 255), 2)
            cv2.circle(vis_image, (int(lidar_center[0]), int(lidar_center[1])), 5, (0, 0, 255), -1)
            
            # **Correct Perspective Transform with given parameters**
            focal_length = 1.16  # mm
            pixel_size = 2e-6  # mm (2 micrometers per pixel)
            
            lidar_z = 950  # mm (converted from 95 cm)
            camera_z = 2150  # mm (converted from 215 cm)
            
            pixel_x, pixel_y = lidar_center
            
            real_x = (focal_length / pixel_size) * (lidar_z - camera_z) * (pixel_x - 1280)
            real_y = (focal_length / pixel_size) * (lidar_z - camera_z) * (pixel_y - 1280)

            # Publish the estimated location
            location = Point()
            location.x = real_x / 10  # Convert mm to cm
            location.y = real_y / 10  # Convert mm to cm
            location.z = 0  # Assuming lidar is at z=95, so no need to modify
            
            self.location = location
            rospy.loginfo(f"Estimated real-world location (cm): {self.location}")

        # Save visualization image
        cv2.imwrite("/home/icvl/FOMA/output/lidar_detection.png", vis_image)


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.loginfo("LocalizationNode starting")
    localizer = LocalizationNode()
    localizer.run()
    rospy.spin()
