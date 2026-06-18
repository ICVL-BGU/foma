#!/usr/bin/env python3

import os

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
os.environ['NO_ALBUMENTATIONS_UPDATE'] = '1'
# Per-node Ultralytics config dir: avoids settings.json write race between
# concurrent YOLO nodes (room/fish/localization) that triggers a spurious
# "settings reset to default values" warning. Ultralytics appends an
# 'Ultralytics' subdir, so the dir below must exist and be writable.
_yolo_cfg = os.path.expanduser('~/.config/yolo_fish')
os.makedirs(_yolo_cfg, exist_ok=True)
os.environ.setdefault('YOLO_CONFIG_DIR', _yolo_cfg)
# Silence Ultralytics import-time INFO (e.g. "Creating new Settings file").
os.environ.setdefault('YOLO_VERBOSE', 'False')

import rospy
from sensor_msgs.msg import CompressedImage
from abstract_node import AbstractNode
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from std_msgs.msg import Header
from ultralytics import YOLO

class FishDetectionNode(AbstractNode):
    def __init__(self):
        super().__init__('fish_detection', 'Fish detection')

        model_path = rospy.get_param('~model_path')
        self.tracker = rospy.get_param('~tracker_path', 'ocsort.yaml')
        self.model = YOLO(model_path)
        self.img = None
        self.bridge = CvBridge()
        self.no_fish_count = 0
        self.direction = Twist(linear=Vector3(0, 0, 0))
        self.prediction = None
        self.point_b, self.point_a = 0, 1 # default to head(0) and tail(3), neck is (1)

        self.image_sub = rospy.Subscriber('fish_camera/image', CompressedImage, self.read_image, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        self.fish_state_pub = rospy.Publisher('fish_detection/state', TwistStamped, queue_size=10, tcp_nodelay=True)

    def read_image(self, img_msg: CompressedImage):
        try:
            self.img = self.bridge.compressed_imgmsg_to_cv2(img_msg)
            # Stamp with the image CAPTURE time, not Time.now() after inference.
            # model.track adds tens-hundreds ms; using now() would tag the fish
            # state with capture_time + inference_latency, so downstream overlays
            # (sidebyside_worker) render the fish direction lagging the video.
            stamp = img_msg.header.stamp
            if stamp.to_sec() == 0.0:
                stamp = rospy.Time.now()
            self.process_image(stamp)
        except CvBridgeError as e:
            self.logerr(f"Error converting image: {e}")

    def process_image(self, timestamp):
        prediction = self.model.track(self.img, tracker=self.tracker, persist=True, verbose=False)[0]
        img_h, img_w = self.img.shape[:2]
        img_center = (img_w / 2, img_h / 2)

        best_kp = None
        min_dist = float('inf')

        kps = prediction.keypoints
        
        if kps.shape[1] == 0:
            self.no_fish_count += 1
            if self.no_fish_count >= 10:
                self.fish_state_pub.publish(TwistStamped(twist = Twist(linear=Vector3(0, 0, 0)), header = Header(stamp = timestamp)))  # fish not detected for 5 frames
            else:
                self.fish_state_pub.publish(TwistStamped(twist = self.direction, header = Header(stamp = timestamp)))  # send previous direction
            return
        else:
            self.no_fish_count = 0  # reset no fish count if fish is detected
            conf_mask = prediction.boxes.conf >= 0.3
            kps = kps[conf_mask]
            if len(kps) == 0:
                self.fish_state_pub.publish(TwistStamped(twist=Twist(linear=Vector3(0, 0, 0)), header=Header(stamp=timestamp)))
                return
            for kp in kps:
                keypoint_xy = kp.data[0][self.point_a] # tail
                dist = ((keypoint_xy[0] - img_center[0]) ** 2 + (keypoint_xy[1] - img_center[1]) ** 2) ** 0.5
                if dist < min_dist:
                    min_dist = dist
                    best_kp = kp

        points = best_kp.data[0][self.point_b], best_kp.data[0][self.point_a]  # head and tail
        dx, dy = points[0] - points[1]
        x, y = points[1]

        # Check if the tail position is far off
        # if hasattr(self, 'prev_tail') and (abs(self.prev_tail[0] - x) > 50 or abs(self.prev_tail[1] - y) > 50):
        #     self.fish_state_pub.publish(TwistStamped(twist = self.direction, header = Header(stamp = timestamp)))  # send previous direction
        #     return

        self.direction = Twist(linear=Vector3(x, y, 0), angular=Vector3(dx, -dy, 0))
        self.prev_tail = points[1]
        self.fish_state_pub.publish(TwistStamped(twist = self.direction, header = Header(stamp = timestamp)))

if __name__ == "__main__":
    rospy.init_node('fish_detection_node')
    fish_det = FishDetectionNode()
    rospy.spin()