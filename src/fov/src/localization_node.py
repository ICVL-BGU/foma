#!/usr/bin/env python3

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
        self.image_sub = rospy.Subscriber('ceiling_camera/image', Image, self.read_image)
        self.img_location_pub = rospy.Publisher('localization/img_location', Point, queue_size=10)
        self.real_location_pub = rospy.Publisher('localization/real_location', Point, queue_size=10)
        self.img_location = None
        self.real_location = None
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
                # if self.img_location is not None:
                #     # rospy.loginfo(f"Publishing location: {self.location}")
                #     self.img_location_pub.publish(self.img_location)

    def read_image(self, img_msg: Image):
        try:
            self.img = self.bridge.imgmsg_to_cv2(img_msg)
            # rospy.loginfo("Image received and converted")
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")

    def process_image(self):
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

        # Updated HSV bounds for olive green
        lower_olive = np.array([30, 100, 150])  
        upper_olive = np.array([80, 255, 255])  

        # Threshold the image to get only olive green colors
        mask = cv2.inRange(hsv, lower_olive, upper_olive)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Process detected contours
        if contours:
            # rospy.loginfo(f"Detected {len(contours)} contours")

            # Get the bounding box of the first contour
            x, y, w, h = cv2.boundingRect(contours[0])
            center_x = x + w / 2
            center_y = y + h / 2
            location = Point()
            location.x = center_x  # Convert mm to cm
            location.y = center_y  # Convert mm to cm
            # location.z = 0  # Assuming lidar is at z=95, so no need to modify
            
            self.img_location = location

            self.img_location_pub.publish(self.img_location)
            # rospy.loginfo(f"Estimated real-world location (cm): {self.location}")

        # Save visualization image
            # vis_image = self.img.copy()
            # cv2.circle(vis_image, (int(center_x), int(center_y)), 5, (0, 255, 0), -1)
            # cv2.imwrite("/home/icvl/FOMA/output/lidar_detection.png", vis_image)


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.loginfo("LocalizationNode starting")
    localizer = LocalizationNode()
    localizer.run()
    rospy.spin()
