#!/usr/bin/env python3

import os

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
os.environ['NO_ALBUMENTATIONS_UPDATE'] = '1'

import rospy
from sensor_msgs.msg import CompressedImage
from abstract_node import AbstractNode
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from std_msgs.msg import Header
# import sleap
from ultralytics import YOLO

class FishDetectionNode(AbstractNode):
    def __init__(self):
        super().__init__('fish_detection', 'Fish detection')

        model_path = r"/home/icvl/ros_ws/src/foma/yolo_pose.pt" # /home/icvl/ros_ws/src/foma/yolo_pose.pt" OR /home/alex/ROS/src/foma/yolo_pose.pt
        self.model = YOLO(model_path)
        self.direction = None
        self.img = None
        self.bridge = CvBridge()
        self.no_fish_count = 0
        self.direction = Twist(linear=Vector3(0, 0, 0))
        self.prediction = None

        self.image_sub = rospy.Subscriber('fish_camera/image', CompressedImage, self.read_image)
        self.fish_state_pub = rospy.Publisher('fish_detection/state', TwistStamped, queue_size=10)

    def read_image(self, img_msg: CompressedImage):
        try:
            self.img = self.bridge.compressed_imgmsg_to_cv2(img_msg)
            self.process_image()
        except CvBridgeError as e:
            self.logerr(f"Error converting image: {e}")

    def process_image(self):
        timestamp = rospy.Time.now()
        prediction = self.model.track(self.img, max_det=1, verbose=False)
        kp = prediction[0].keypoints
        
        if kp.shape[1] == 0:
            self.no_fish_count += 1
            if self.no_fish_count >= 5:
                self.fish_state_pub.publish(TwistStamped(twist = Twist(linear=Vector3(0, 0, 0)), header = Header(stamp = timestamp)))  # fish not detected for 5 frames
            else:
                self.fish_state_pub.publish(TwistStamped(twist = self.direction, header = Header(stamp = timestamp)))  # send previous direction
            return
        else:
            self.no_fish_count = 0  # reset no fish count if fish is detected

        points = kp.data[0][0], kp.data[0][3]  # head and tail
        dx, dy = points[0] - points[1]
        x, y = points[1]

        # Check if the tail position is far off
        if hasattr(self, 'prev_tail') and (abs(self.prev_tail[0] - x) > 50 or abs(self.prev_tail[1] - y) > 50):
            self.fish_state_pub.publish(TwistStamped(twist = self.direction, header = Header(stamp = timestamp)))  # send previous direction
            return

        self.direction = Twist(linear=Vector3(x, y, 0), angular=Vector3(dx, -dy, 0))
        self.prev_tail = points[1]
        self.fish_state_pub.publish(TwistStamped(twist = self.direction, header = Header(stamp = timestamp)))

if __name__ == "__main__":
    rospy.init_node('fish_detection_node')
    fish_det = FishDetectionNode()
    rospy.spin()