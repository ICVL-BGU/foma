#!/usr/bin/env python3
import math
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Vector3
from std_srvs.srv import SetBool, SetBoolResponse


class GoHomeNode:
    def __init__(self):
        rospy.init_node("go_home_node")

        self.enabled = False
        self._state = "IDLE"
        self._stable_center = 0

        self._ranges = None
        self._range_max = 15000.0

        self.publish_hz = rospy.get_param("~publish_hz", 20)

        self.center_tol_mm = float(rospy.get_param("~center_tol_mm", 60.0))
        self.align_tol_deg = float(rospy.get_param("~align_tol_deg", 4.0))
        self.realign_deg = float(rospy.get_param("~realign_deg", 10.0))
        self.window_deg = int(rospy.get_param("~window_deg", 5))

        self.rot_gain = float(rospy.get_param("~rot_gain", 1.0 / 45.0))
        self.rot_max = float(rospy.get_param("~rot_max", 0.6))

        self._target_axis = None  # chosen snap axis (0/90/180/270)

        self.pub_vec = rospy.Publisher("go_home/vector", Vector3, queue_size=10)
        self.pub_enabled = rospy.Publisher("go_home/enabled", Bool, queue_size=1, latch=True)
        self.pub_rotate = rospy.Publisher("motor_control/rotate", Float32, queue_size=10)

        rospy.Subscriber("lidar/scans", LaserScan, self.on_scan, queue_size=10)

        self.srv = rospy.Service("go_home/enable", SetBool, self.on_enable)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_hz), self.on_timer)

        self.pub_enabled.publish(Bool(self.enabled))
        rospy.loginfo("GoHomeNode ready. Use /go_home/enable (SetBool).")

    def on_enable(self, req):
        self.enabled = bool(req.data)
        self.pub_enabled.publish(Bool(self.enabled))

        if self.enabled:
            self._state = "ALIGN"
            self._stable_center = 0
            self._target_axis = None
        else:
            self._state = "IDLE"
            self._stable_center = 0
            self._target_axis = None
            self.pub_vec.publish(Vector3(0.0, 0.0, 0.0))
            self.pub_rotate.publish(Float32(0.0))

        return SetBoolResponse(success=True, message=f"go_home enabled={self.enabled}")

    def on_scan(self, msg: LaserScan):
        r = np.array(msg.ranges, dtype=np.float32)
        if r.size != 360:
            return

        self._range_max = float(msg.range_max) if msg.range_max > 0 else 15000.0

        r[~np.isfinite(r)] = self._range_max
        r[r <= 0] = self._range_max
        r[r > self._range_max] = self._range_max

        self._ranges = r

    @staticmethod
    def _wrap180(deg: float) -> float:
        return ((deg + 180.0) % 360.0) - 180.0

    @staticmethod
    def _nearest_axis(deg: float) -> float:
        return (round(deg / 90.0) * 90.0) % 360.0

    def _median_at(self, center_deg: int) -> float:
        if self._ranges is None:
            return self._range_max
        w = self.window_deg
        idx = [(center_deg + k) % 360 for k in range(-w, w + 1)]
        vals = self._ranges[idx]
        return float(np.median(vals))

    def _estimate_wall_normal_deg(self) -> float:
        if self._ranges is None:
            return 0.0

        w = 5
        kern = np.ones(2 * w + 1, dtype=np.float32) / float(2 * w + 1)
        sm = np.convolve(np.r_[self._ranges[-w:], self._ranges, self._ranges[:w]], kern, mode="valid")

        i = int(np.argmin(sm))
        return float(i % 360)

    def _align_step(self) -> bool:
        a0 = self._estimate_wall_normal_deg()
        if self._target_axis is None:
            self._target_axis = self._nearest_axis(a0)

        err = self._wrap180(a0 - self._target_axis)
        if abs(err) <= self.align_tol_deg:
            self.pub_rotate.publish(Float32(0.0))
            return True

        cmd = float(np.clip(err * self.rot_gain, -self.rot_max, self.rot_max))
        self.pub_vec.publish(Vector3(0.0, 0.0, 0.0))
        self.pub_rotate.publish(Float32(cmd))
        return False

    def _center_step(self) -> bool:
        d_front = self._median_at(0)
        d_back = self._median_at(180)
        d_left = self._median_at(90)
        d_right = self._median_at(270)

        e_lr = d_left - d_right
        e_fb = d_front - d_back

        if abs(e_lr) <= self.center_tol_mm and abs(e_fb) <= self.center_tol_mm:
            self._stable_center += 1
        else:
            self._stable_center = 0

        if self._stable_center >= int(0.5 * self.publish_hz):
            self.pub_vec.publish(Vector3(0.0, 0.0, 0.0))
            self.pub_rotate.publish(Float32(0.0))
            return True

        if self._target_axis is None:
            self._target_axis = 0.0

        a0 = self._estimate_wall_normal_deg()
        align_err = abs(self._wrap180(a0 - self._target_axis))
        if align_err >= self.realign_deg:
            self.pub_vec.publish(Vector3(0.0, 0.0, 0.0))
            self.pub_rotate.publish(Float32(0.0))
            self._state = "ALIGN"
            return False

        self.pub_rotate.publish(Float32(0.0))
        self.pub_vec.publish(Vector3(float(e_lr), float(e_fb), 0.0))
        return False

    def _finish(self):
        self.enabled = False
        self._state = "IDLE"
        self._stable_center = 0
        self._target_axis = None
        self.pub_vec.publish(Vector3(0.0, 0.0, 0.0))
        self.pub_rotate.publish(Float32(0.0))
        self.pub_enabled.publish(Bool(False))

    def on_timer(self, _):
        if not self.enabled:
            return
        if self._ranges is None:
            return

        if self._state == "ALIGN":
            if self._align_step():
                self._state = "CENTER"
                self._stable_center = 0
            return

        if self._state == "CENTER":
            if self._center_step():
                self._state = "FINAL_ALIGN"
            return

        if self._state == "FINAL_ALIGN":
            if self._align_step():
                self._finish()
            return


if __name__ == "__main__":
    GoHomeNode()
    rospy.spin()
