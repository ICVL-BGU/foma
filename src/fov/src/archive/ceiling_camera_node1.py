#!/usr/bin/env python3

import rospy
import subprocess
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension
from abstract_node import AbstractNode
from cv_bridge import CvBridge, CvBridgeError  # Needed for converting images to ROS format
import cv2

class CeilingCameraNode(AbstractNode):
    def __init__(self):
        super().__init__('ceiling_camera_node', 'Ceiling Camera Node')
        rospy.loginfo("Ceiling Camera Node: Node started.")

        # RTSP stream URL (Modify as needed)
        # self.rtsp_url = "rtsp://admin:icvl2023@1.1.2.103:554"
        self.camera = cv2.VideoCapture("http://1.1.2.103/mjpeg")

        # Use FFmpeg to grab frames
        self.ffmpeg_cmd = [
            "ffmpeg", "-i", self.rtsp_url,
            "-fflags", "nobuffer", "-flags", "low_delay",
            "-f", "image2pipe", "-pix_fmt", "bgr24",
            "-vcodec", "rawvideo", "-"
        ]

        # Start FFmpeg subprocess to read the video stream
        self.process = subprocess.Popen(self.ffmpeg_cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=10**8)

        # Set up the image publisher
        self.image_pub = rospy.Publisher('ceiling_camera/image', Image, queue_size=10)

        # OpenCV bridge (only for converting NumPy images to ROS format)
        self.bridge = CvBridge()

        # Image resolution (Modify based on your camera settings)
        self.width = 1024
        self.height = 768

        rospy.on_shutdown(self.__on_shutdown)

    def run(self):
        while not rospy.is_shutdown():
            try:
                # Read raw frame from FFmpeg
                raw_frame = self.process.stdout.read(self.width * self.height * 3)
                if len(raw_frame) != self.width * self.height * 3:
                    rospy.logwarn("Incomplete frame received, skipping.")
                    continue

                # Convert raw bytes to NumPy array
                frame = np.frombuffer(raw_frame, np.uint8).reshape((self.height, self.width, 3))

                # Convert to ROS Image message
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(img_msg)

            except CvBridgeError as e:
                rospy.logerr(f"Ceiling Camera: CV Bridge Error - {e}")
                

    def __on_shutdown(self):
        rospy.loginfo("Ceiling Camera: Stopping FFmpeg process.")
        self.process.terminate()
        self.process.wait()

if __name__ == "__main__":
    rospy.init_node('ceiling_camera', anonymous=False)
    node = CeilingCameraNode()
    node.run()
    rospy.spin()
