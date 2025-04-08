#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from abstract_node import AbstractNode
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from foma.msg import FishState  # Import the custom message


class FishDetectionNode(AbstractNode):
    def __init__(self):
        super().__init__('fish_detection', 'Fish detection')
        self.image_sub = rospy.Subscriber('web_camera/image', Image, self.read_image)
        self.fish_state_pub = rospy.Publisher('fish_detection/state', FishState, queue_size=10)
        self.direction = None
        self.img = None
        self.bridge = CvBridge()
        self.lower_hsv = np.array([0, 0, 0])  # Adjust these thresholds as needed
        self.higher_hsv = np.array([255, 255, 255])  # Adjust these thresholds as needed
        self.fgbg = cv2.createBackgroundSubtractorMOG2()
        self.count = 0

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
        frame = self.img
        if frame is None:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.higher_hsv)
        result = cv2.bitwise_and(frame, frame, mask=mask)
        fgmask = self.fgbg.apply(result)

        _, th1 = cv2.threshold(result, 30, 255, cv2.THRESH_BINARY)
        gray = 255 - cv2.cvtColor(th1, cv2.COLOR_BGR2GRAY)

        _, cnts, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        areaSUM, CCx, CCy, maxA, CxMax, CyMax, maxC, c = 0, 0, 0, 0, 0, 0, 0, None

        for c in cnts:
            M = cv2.moments(c)
            if M['m00'] > 500:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                areaSUM += M['m00']
                CCx += cx * M['m00']
                CCy += cy * M['m00']

                if maxA < M['m00']:
                    maxA, CxMax, CyMax, self.cmax = M['m00'], cx, cy, c

        if areaSUM > 0:
            CCx, CCy = CCx / areaSUM, CCy / areaSUM
        Xc = tuple(np.array([CCx, CCy], dtype=int).reshape(1, -1)[0])

        # Example direction logic (replace with your logic)
        direction = 90 if Xc[0] > frame.shape[1] // 2 else 270

        # Publish state
        state_msg = FishState()
        state_msg.direction = direction
        state_msg.x = int(CCx)
        state_msg.y = int(CCy)
        self.fish_state_pub.publish(state_msg)

        cv2.drawContours(frame, cnts, -1, (255, 0, 0), 2)
        cv2.imshow('Processed Image', frame)

        key = cv2.waitKey(30)
        if key == 'q' or key == 27:
            return


if __name__ == "__main__":
    rospy.init_node('fish_detection_node')
    fish_det = FishDetectionNode()
    fish_det.run()
    rospy.spin()
