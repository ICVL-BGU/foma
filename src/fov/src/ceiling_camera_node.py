#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from abstract_node import AbstractNode
import cv2
from cv_bridge import CvBridge, CvBridgeError


class CeilingCameraNode(AbstractNode):
    def __init__(self):
        # Initialize the node with a name
        super().__init__(f'ceiling_camera_node_{camera_id}', f'Ceiling Camera Node {camera_id}')

        # Retrieve camera-specific parameters
        self.camera_id = rospy.get_param("~camera_id", 0)
        self.camera_ip = rospy.get_param("~camera_ip", "rtsp://admin:icvl2023@1.1.2.101:554")

        # Initialize the camera
        self.camera = cv2.VideoCapture(self.camera_ip)

        # Set up the image publisher
        self.image_pub = rospy.Publisher(f'ceiling_camera_{self.camera_id}/image', Image, queue_size=10)

        # OpenCV bridge
        self.bridge = CvBridge()

        # Shutdown behavior
        rospy.on_shutdown(self.__on_shutdown)

    def run(self):
        while not rospy.is_shutdown():
            try:
                ret, img = self.camera.read()
                if ret:
                    # Publish the image
                    img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
                    self.image_pub.publish(img_msg)
                else:
                    rospy.logwarn(f"Camera {self.camera_id}: No image captured.")
            except CvBridgeError as e:
                rospy.logerr(f"Camera {self.camera_id}: CV Bridge Error - {e}")

            rospy.sleep(0.05)

    def __on_shutdown(self):
        rospy.loginfo(f"Ceiling Camera {self.camera_id}: Releasing camera.")
        self.camera.release()


if __name__ == "__main__":
    # Retrieve the camera ID from parameters
    camera_id = rospy.get_param("~camera_id", 0)

    # Initialize the node with a unique name based on the camera ID
    rospy.init_node(f'ceiling_camera_node_{camera_id}', anonymous=False)

    rospy.loginfo(f"Ceiling Camera Node {camera_id}: Node started.")
    node = CeilingCameraNode()
    node.run()
    rospy.spin()
