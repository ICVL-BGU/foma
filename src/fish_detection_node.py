#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from abstract_node import AbstractNode
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from foma.msg import FishState  # Import the custom message
import sleap

class FishDetectionNode(AbstractNode):
    def __init__(self):
        super().__init__('fish_detection', 'Fish detection')
        self.image_sub = rospy.Subscriber('fish_camera/image', Image, self.read_image)
        self.fish_state_pub = rospy.Publisher('fish_detection/state', FishState, queue_size=10)

        sleap.disable_preallocation()
        model_path = r"/home/alex/sleap/folwerhorn4/models/250402_192455.single_instance.n=93"
        self.model = sleap.load_model(model_path)
        self.direction = None
        self.img = None
        self.bridge = CvBridge()

    def run(self):
        while not rospy.is_shutdown():
            if self._system_on:
                if self.img is not None:
                    self.process_image()

    def read_image(self, img_msg: Image):
        try:
            self.img = self.bridge.imgmsg_to_cv2(img_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")

    def process_image(self):
        prediction = self.inference_model.model.predict(self.img)[0]
        points, confidences = prediction['instance_peaks'][[0,5]], prediction['instance_peak_vals'][[0,5]]
        if np.any(confidences < 0.2):
            return
        direction_vector = points[0] - points[1]
        x, y = points[1]
        self.direction = FishState(direction = direction_vector, x = x, y = y)
        self.fish_state_pub.publish(self.direction)

if __name__ == "__main__":
    rospy.init_node('fish_detection_node')
    fish_det = FishDetectionNode()
    fish_det.run()
    rospy.spin()
