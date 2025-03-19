#!/usr/bin/env python3

from email.mime import image
import rospy
from sensor_msgs.msg import Image
from abstract_node import AbstractNode
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from fov.msg import FomaLocation
from geometry_msgs.msg import Point

class LocalizationNode(AbstractNode):
    def __init__(self):
        super().__init__('localization', 'Localization')
        self.image_sub = rospy.Subscriber('ceiling_camera/image', Image, self.process_image)
        self.location_pub = rospy.Publisher('localization/location', FomaLocation, queue_size=10)
        self.bridge = CvBridge()
        
        # Updated HSV bounds for olive green
        self.lower_bound, self.upper_bound = np.array([30, 100, 150]), np.array([80, 255, 255])
        rospy.loginfo("Localization Node initialized")

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
                self.location = FomaLocation()
                x, y, w, h = cv2.boundingRect(contours[0])
                self.location.image = Point()
                self.location.image.x = x + w / 2
                self.location.image.y = y + h / 2
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
                px = (self.location.image.x - origin_x) * (sensor_width / image_width)  # Convert pixels to sensor units (mm)
                py = (self.location.image.y - origin_y) * (sensor_height / image_height)  # Convert pixels to sensor units (mm)
                
                # Scale factor to map image coordinates to world coordinates
                scale = (cam_z - ground_height) / focal_length  # Use focal length in meters
                
                # Convert sensor distances to real-world distances
                self.location.world = Point()
                self.location.world.x = cam_x + px * scale
                self.location.world.y = cam_y + py * scale

                self.location_pub.publish(self.location)

        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.loginfo("LocalizationNode starting")
    localizer = LocalizationNode()
    # localizer.run()
    rospy.spin()
