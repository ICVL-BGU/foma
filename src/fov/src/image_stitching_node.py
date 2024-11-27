#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class ImageStitchingNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('image_stitching_node', anonymous=False)

        # Define the camera topics based on a fixed naming convention
        self.camera_topics = [
            "/ceiling_camera_1/image",
            "/ceiling_camera_2/image",
            "/ceiling_camera_3/image",
            "/ceiling_camera_4/image",
            "/ceiling_camera_5/image"
        ]

        # Create a dictionary to store the latest images from each camera
        self.camera_images = {topic: None for topic in self.camera_topics}

        # Subscribe to each camera's image topic
        self.bridge = CvBridge()
        for topic in self.camera_topics:
            rospy.Subscriber(topic, Image, self.image_callback, callback_args=topic)

        # Publisher for the stitched image
        self.stitched_image_pub = rospy.Publisher("stitched_image", Image, queue_size=10)

        rospy.on_shutdown(self.__on_shutdown)

    def image_callback(self, msg, topic):
        try:
            # Convert the ROS Image message to OpenCV format
            self.camera_images[topic] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image from topic {topic}: {e}")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Check if all cameras have provided images
            if all(img is not None for img in self.camera_images.values()):
                try:
                    # Stitch images
                    stitched_image = self.stitch_images(list(self.camera_images.values()))
                    if stitched_image is not None:
                        # Publish the stitched image
                        stitched_msg = self.bridge.cv2_to_imgmsg(stitched_image, "bgr8")
                        self.stitched_image_pub.publish(stitched_msg)
                except Exception as e:
                    rospy.logerr(f"Error stitching images: {e}")

            rate.sleep()

    def stitch_images(self, images):
        """
        Example image stitching function. Uses OpenCV's Stitcher.
        """
        stitcher = cv2.Stitcher_create()
        status, stitched_image = stitcher.stitch(images)
        if status == cv2.Stitcher_OK:
            return stitched_image
        else:
            rospy.logwarn(f"Image stitching failed with status {status}.")
            return None

    def __on_shutdown(self):
        rospy.loginfo("Shutting down Image Stitching Node.")


if __name__ == "__main__":
    node = ImageStitchingNode()
    node.run()
