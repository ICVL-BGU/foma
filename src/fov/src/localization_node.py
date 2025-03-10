#!/usr/bin/env python3

from email.mime import image
import rospy
from sensor_msgs.msg import Image
from abstract_node import AbstractNode
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Point


class LocalizationNode(AbstractNode):
    def __init__(self):
        super().__init__('localization', 'Localization')
        self.image_sub = rospy.Subscriber('ceiling_camera/image', Image, self.process_image)
        self.img_location_pub = rospy.Publisher('localization/img_location', Point, queue_size=10)
        self.real_location_pub = rospy.Publisher('localization/real_location', Point, queue_size=10)
        self.img_location = None
        self.real_location = None
        # self.img = None
        self.bridge = CvBridge()
        
        # Updated HSV bounds for olive green
        self.lower_bound, self.upper_bound = np.array([30, 100, 150]), np.array([80, 255, 255])
        rospy.loginfo("LocalizationNode initialized")

    # def run(self):
    #     rospy.loginfo("LocalizationNode running")
    #     while not rospy.is_shutdown():
    #         if True: #self._system_on:
    #             if self.img is not None:
    #                 # rospy.loginfo("Processing image")
    #                 self.process_image()
    #             # if self.img_location is not None:
    #             #     # rospy.loginfo(f"Publishing location: {self.location}")
    #             #     self.img_location_pub.publish(self.img_location)

    def process_image(self, img_msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg)
            # rospy.loginfo("Image received and converted")
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Threshold the image to get only olive green colors
            mask = cv2.inRange(hsv, self.lower_bound, self.upper_bound)

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Process detected contours
            if contours:
                # Get the bounding box of the first contour
                x, y, w, h = cv2.boundingRect(contours[0])
                self.img_location = Point()
                self.img_location.x = x + w / 2
                self.img_location.y = y + h / 2
                # location.z = 0  # Assuming lidar is at z=95, so no need to modify
                
                # Camera parameters in meters
                focal_length = 1.16 / 1000 # focal length in meters
                sensor_width = 5.5 / 1000
                sensor_height = 4 / 1000
                cam_x, cam_y, cam_z = (2.5, 2.5, 2.15)
                image_width, image_height = 1024, 768
                ground_height = 0.95
                
                # Define image center as origin
                origin_x = image_width/2 # 1024 / 2
                origin_y = image_height/2 # 768 / 2
                
                # Compute pixel offsets from the image center
                px = (self.img_location.x - origin_x) * (sensor_width / image_width)  # Convert pixels to sensor units (mm)
                py = (self.img_location.y - origin_y) * (sensor_height / image_height)  # Convert pixels to sensor units (mm)
                
                # Scale factor to map image coordinates to world coordinates
                scale = (cam_z - ground_height) / focal_length  # Use focal length in meters
                
                # Convert sensor distances to real-world distances
                self.real_location = Point()
                self.real_location.x = cam_x + px * scale
                self.real_location.y = cam_y + py * scale

                self.img_location_pub.publish(self.img_location)
                self.real_location_pub.publish(self.real_location)

        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.loginfo("LocalizationNode starting")
    localizer = LocalizationNode()
    # localizer.run()
    rospy.spin()
