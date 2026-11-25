#!/usr/bin/env python3

import os

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
os.environ['NO_ALBUMENTATIONS_UPDATE'] = '1'
# Per-node Ultralytics config dir: avoids settings.json write race between
# concurrent YOLO nodes (room/fish/localization) that triggers a spurious
# "settings reset to default values" warning. Ultralytics appends an
# 'Ultralytics' subdir, so the dir below must exist and be writable.
_yolo_cfg = os.path.expanduser('~/.config/yolo_localization')
os.makedirs(_yolo_cfg, exist_ok=True)
os.environ.setdefault('YOLO_CONFIG_DIR', _yolo_cfg)
# Silence Ultralytics import-time INFO (e.g. "Creating new Settings file").
os.environ.setdefault('YOLO_VERBOSE', 'False')

import rospy
from sensor_msgs.msg import CompressedImage
from abstract_node import AbstractNode
from etc import jpeg
from etc.room_mapping import load_mappers, map_point
from foma.msg import FomaLocation
from geometry_msgs.msg import Point
from ultralytics import YOLO

FOMA_TAG = 0
LIDAR_TAG = 1

# Per-axis static blend (LIDAR weight). Best held-out result: 42.60 mm test mean.
ALPHA_X = 0.65
ALPHA_Y = 0.85

CONF_MIN = 0.25
DETECTION_REPORT_S = 5.0    # summary of missed frames this often


class LocalizationNode(AbstractNode):
    def __init__(self):
        super().__init__('localization', 'FOMA Localization')

        detection_model_path = rospy.get_param('~detection_model_path')
        mapper_path = rospy.get_param('~mapper_path')
        self.tracker = rospy.get_param('~tracker_path', 'bytetrack.yaml')
        self.detection_model = YOLO(detection_model_path)
        # Single JSON, both mappers nested under 'lidar' and 'foma'.
        mappers = load_mappers(mapper_path)
        self.lidar_params = mappers['lidar']
        self.foma_params = mappers['foma']
        self.img = None
        self.image_sub = rospy.Subscriber('ceiling_camera/image', CompressedImage, self.read_image, queue_size=1, buff_size=2**21, tcp_nodelay=True)
        self.location_pub = rospy.Publisher('localization/location', FomaLocation, queue_size=10, tcp_nodelay=True)
        self.location = FomaLocation()
        self.__camera_native_shape = tuple(rospy.get_param('/ROOM_CAMERA_NATIVE_SHAPE'))
        self.__floor_map_shape = tuple(rospy.get_param('/ROOM_FLOOR_MAP_SHAPE'))
        self.__report_t = rospy.get_time()
        self.__counts = [0, 0, 0, 0]    # frames, FOMA seen, LIDAR seen, either

    def read_image(self, img_msg: CompressedImage):
        try:
            self.img = jpeg.decode(img_msg.data)
            # Use image CAPTURE time, not Time.now() after detection, so the
            # logged foma location aligns with the room video frame it came from.
            stamp = img_msg.header.stamp
            if stamp.to_sec() == 0.0:
                stamp = rospy.Time.now()
            self.process_image(stamp)
        except Exception as e:
            self.logerr(f"Error converting image: {e}")

    def process_image(self, timestamp):
        self.location.header.stamp = timestamp

        prediction = self.detection_model.track(self.img, tracker=self.tracker, persist=True, verbose=False)
        lidar_det = self.__select_detection(prediction, LIDAR_TAG)
        foma_det = self.__select_detection(prediction, FOMA_TAG)

        self.__count(foma_det, lidar_det)
        result = self.__localize(lidar_det, foma_det)
        if result is not None:
            x_i, y_i, x_w, y_w = result
            self.location.image = Point(x_i, y_i, 0)
            self.location.world = Point(x_w / self.__floor_map_shape[1], y_w / self.__floor_map_shape[0], 0)

        self.location_pub.publish(self.location)

    def __count(self, foma_det, lidar_det):
        """Log the per-tag detection rate every DETECTION_REPORT_S when frames
        are missed."""
        c = self.__counts
        c[0] += 1
        c[1] += foma_det is not None
        c[2] += lidar_det is not None
        c[3] += foma_det is not None or lidar_det is not None
        now = rospy.get_time()
        if now - self.__report_t < DETECTION_REPORT_S:
            return
        if c[3] < c[0]:
            rospy.logwarn(f"Detection last {now - self.__report_t:.0f}s: robot in "
                          f"{c[3]}/{c[0]} frames (FOMA {c[1]}, LIDAR {c[2]})")
        self.__report_t = now
        self.__counts = [0, 0, 0, 0]

    def __select_detection(self, prediction, tag):
        """Return (xc_n, yc_n, conf) of highest-conf box of `tag`, or None.

        Collapses multiple detections to the single top-conf box.
        """
        boxes = prediction[0].boxes
        indices = boxes.data[:, -1] == tag
        if not indices.any():
            return None

        cls_boxes = boxes.xywhn[indices]
        cls_conf = boxes.conf[indices]
        max_idx = cls_conf.argmax()
        conf_val = float(cls_conf[max_idx])
        if conf_val < CONF_MIN:
            return None

        box = cls_boxes[max_idx]
        box_vals = box.cpu().numpy() if hasattr(box, 'cpu') else box
        return float(box_vals[0]), float(box_vals[1]), conf_val

    def __localize(self, lidar_det, foma_det):
        """Fuse LIDAR/FOMA detections -> (x_i, y_i, x_w, y_w) or None."""
        lp = self.__map(self.lidar_params, lidar_det) if lidar_det else None
        fp = self.__map(self.foma_params, foma_det) if foma_det else None

        if lp is None and fp is None:
            return None

        if fp is None:
            return lidar_det[0], lidar_det[1], lp[0], lp[1]

        if lp is None:
            return foma_det[0], foma_det[1], fp[0], fp[1]

        # Both detected: per-axis static blend. Image point taken from LIDAR.
        x_w = ALPHA_X * lp[0] + (1 - ALPHA_X) * fp[0]
        y_w = ALPHA_Y * lp[1] + (1 - ALPHA_Y) * fp[1]
        return lidar_det[0], lidar_det[1], x_w, y_w

    def __map(self, params, det):
        """det = (xc_n, yc_n, conf) -> world (x_w, y_w). Scale to native px first."""
        x_px = det[0] * self.__camera_native_shape[1]
        y_px = det[1] * self.__camera_native_shape[0]
        return map_point(params, x_px, y_px)


if __name__ == "__main__":
    rospy.init_node('localization_node')
    LocalizationNode()
    rospy.spin()
