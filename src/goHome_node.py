#!/usr/bin/env python3
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool, SetBoolResponse

class GoHomeNode:
    IDLE = "IDLE"
    BACKOFF = "BACKOFF"
    ALIGN = "ALIGN"
    GO_CENTER = "GO_CENTER"
    ROTATE_90 = "ROTATE_90"

    def __init__(self):
        self.arrival_tol = rospy.get_param("~arrival_tol", 0.05)
        self.enabled = False

        self.max_speed = rospy.get_param("~max_speed", 0.30)
        self.kp = rospy.get_param("~kp", 0.40)

        self.max_rot_cmd = rospy.get_param("~max_rot_cmd", 0.60)
        self.kp_rot = rospy.get_param("~kp_rot", 0.05)

        self.align_stable_cycles = rospy.get_param("~align_stable_cycles", 10)
        self.align_window_deg = int(rospy.get_param("~align_window_deg", 12))
        self.align_probe_deg = int(rospy.get_param("~align_probe_deg", 20))
        self.align_cmd_tol = float(rospy.get_param("~align_cmd_tol", 0.02))

        self.min_valid = rospy.get_param("~min_valid_range", 0.10)
        self.max_valid = rospy.get_param("~max_valid_range", 10.0)

        self.backoff_min_dist = rospy.get_param("~backoff_min_dist", 0.22)
        self.backoff_speed = rospy.get_param("~backoff_speed", 0.20)
        self.backoff_stable_cycles = rospy.get_param("~backoff_stable_cycles", 10)

        self.total_timeout_s = rospy.get_param("~total_timeout_s", 30.0)
        self.progress_window_s = rospy.get_param("~progress_window_s", 1.5)
        self.progress_min_drop = rospy.get_param("~progress_min_drop", 0.02)

        self.rotate_sig_tol = rospy.get_param("~rotate_sig_tol", 0.08)
        self.rotate_timeout_s = rospy.get_param("~rotate_timeout_s", 12.0)
        self.rotate_stable_cycles = rospy.get_param("~rotate_stable_cycles", 10)
        self.rotate_cmd = float(rospy.get_param("~rotate_cmd", 0.35))

        self.cmd_pub = rospy.Publisher('go_home/twist', Twist, queue_size=10)
        self.rot_pub = rospy.Publisher('go_home/rotate', Float32, queue_size=10)
        self.pub_enabled = rospy.Publisher("go_home/enabled", Bool, queue_size=1, latch=True)

        rospy.Subscriber("lidar/scans", LaserScan, self.on_lidar, queue_size=1)
        self.srv = rospy.Service("/go_home/enable", SetBool, self.on_enable)

        self.state = self.IDLE
        self.align_ok_count = 0
        self.backoff_ok_count = 0
        self.rotate_ok_count = 0

        self.start_time = None
        self.last_progress_check_time = None
        self.last_progress_d = None

        self.arrival_signature = None
        self.rotate_start_time = None

        self.pub_enabled.publish(Bool(False))
        rospy.loginfo("GoHomeNode active and ready.")

    def on_enable(self, req):
        self.enabled = bool(req.data)
        self.pub_enabled.publish(Bool(self.enabled))

        if self.enabled:
            self._reset_run_state()
            self.state = self.ALIGN
        else:
            self._finish()

        return SetBoolResponse(success=True, message=f"GoHome active: {self.enabled}")

    def _reset_run_state(self):
        self.start_time = rospy.Time.now()
        self.last_progress_check_time = rospy.Time.now()
        self.last_progress_d = None

        self.align_ok_count = 0
        self.backoff_ok_count = 0
        self.rotate_ok_count = 0

        self.arrival_signature = None
        self.rotate_start_time = None

    def on_lidar(self, msg):
        if not self.enabled:
            return

        if (rospy.Time.now() - self.start_time).to_sec() > self.total_timeout_s:
            rospy.logwarn("GoHome timeout - stopping.")
            self._finish()
            return

        ranges = np.array(msg.ranges, dtype=float)
        ranges = self._sanitize(ranges)
        if ranges is None:
            return

        f = self._get_avg(ranges, 0)
        b = self._get_avg(ranges, 180)
        l = self._get_avg(ranges, 90)
        r = self._get_avg(ranges, 270)
        if None in [f, b, l, r]:
            return

        min_dist, min_ang = self._min_distance_and_angle(ranges)
        if min_dist is not None and min_dist < self.backoff_min_dist and self.state != self.ROTATE_90:
            self.state = self.BACKOFF

        if self.state == self.BACKOFF:
            self._do_backoff(min_dist, min_ang)
            return

        if self.state == self.ALIGN:
            self._do_align(ranges)
            return

        if self.state == self.GO_CENTER:
            self._do_go_center(f, b, l, r)
            return

        if self.state == self.ROTATE_90:
            self._do_rotate_90(f, b, l, r)
            return

    def _do_backoff(self, min_dist, min_ang):
        if min_dist is None or min_ang is None:
            self.stop_robot()
            return

        away_ang = (min_ang + 180) % 360
        cmd = self._twist_from_angle(away_ang, self.backoff_speed)
        self.cmd_pub.publish(cmd)
        self.rot_pub.publish(Float32(0.0))

        if min_dist >= self.backoff_min_dist + 0.05:
            self.backoff_ok_count += 1
        else:
            self.backoff_ok_count = 0

        if self.backoff_ok_count >= self.backoff_stable_cycles:
            self.stop_robot()
            self.state = self.ALIGN
            self.backoff_ok_count = 0

    def _do_align(self, ranges):
        f = self._get_avg_wide(ranges, 0, self.align_window_deg)
        l = self._get_avg_wide(ranges, 90, self.align_window_deg)
        b = self._get_avg_wide(ranges, 180, self.align_window_deg)
        r = self._get_avg_wide(ranges, 270, self.align_window_deg)
        if None in [f, b, l, r]:
            self.stop_robot()
            return

        e0 = self._probe_asym(ranges, 0, self.align_probe_deg)
        e90 = self._probe_asym(ranges, 90, self.align_probe_deg)
        e180 = self._probe_asym(ranges, 180, self.align_probe_deg)
        e270 = self._probe_asym(ranges, 270, self.align_probe_deg)

        errs = [e for e in [e0, e90, e180, e270] if e is not None]
        if len(errs) < 2:
            self.stop_robot()
            return

        err = float(np.mean(errs))
        rot_cmd = -self.kp_rot * err
        rot_cmd = max(min(rot_cmd, self.max_rot_cmd), -self.max_rot_cmd)

        if abs(rot_cmd) <= self.align_cmd_tol:
            self.align_ok_count += 1
        else:
            self.align_ok_count = 0

        if self.align_ok_count >= self.align_stable_cycles:
            self.stop_robot()
            self.state = self.GO_CENTER
            self.align_ok_count = 0
            return

        self.stop_robot()
        self.rot_pub.publish(Float32(rot_cmd))

    def _do_go_center(self, f, b, l, r):
        error_x = (f - b) / 2.0
        error_y = (l - r) / 2.0
        d = math.hypot(error_x, error_y)

        if d < self.arrival_tol:
            rospy.loginfo("Arrived at center - starting 90 rotation.")
            self.stop_robot()
            self.arrival_signature = (f, b, l, r)
            self.rotate_start_time = rospy.Time.now()
            self.state = self.ROTATE_90
            return

        now = rospy.Time.now()
        if self.last_progress_d is None:
            self.last_progress_d = d
            self.last_progress_check_time = now
        else:
            if (now - self.last_progress_check_time).to_sec() >= self.progress_window_s:
                drop = self.last_progress_d - d
                if drop < self.progress_min_drop:
                    rospy.logwarn("No progress towards center - stopping GoHome.")
                    self._finish()
                    return
                self.last_progress_d = d
                self.last_progress_check_time = now

        cmd = Twist()
        cmd.linear.y = max(min(error_x * self.kp, self.max_speed), -self.max_speed)
        cmd.linear.x = max(min(error_y * self.kp, self.max_speed), -self.max_speed)
        self.rot_pub.publish(Float32(0.0))
        self.cmd_pub.publish(cmd)

    def _do_rotate_90(self, f, b, l, r):
        if (rospy.Time.now() - self.rotate_start_time).to_sec() > self.rotate_timeout_s:
            rospy.logwarn("Rotate 90 timeout - stopping.")
            self._finish()
            return

        if self.arrival_signature is None:
            self._finish()
            return

        f0, b0, l0, r0 = self.arrival_signature
        score = (abs(f - r0) + abs(r - b0) + abs(b - l0) + abs(l - f0)) / 4.0

        if score <= self.rotate_sig_tol:
            self.rotate_ok_count += 1
        else:
            self.rotate_ok_count = 0

        if self.rotate_ok_count >= self.rotate_stable_cycles:
            rospy.loginfo("90 rotation complete.")
            self._finish()
            return

        self.stop_robot()
        self.rot_pub.publish(Float32(self.rotate_cmd))

    def _sanitize(self, ranges):
        if ranges.shape[0] < 360:
            return None
        ranges = ranges[:360].copy()
        valid = np.isfinite(ranges) & (ranges >= self.min_valid) & (ranges <= self.max_valid)
        ranges[~valid] = np.inf
        if np.all(np.isinf(ranges)):
            return None
        return ranges

    def _get_avg(self, ranges, angle):
        idx = [(angle + i) % 360 for i in range(-5, 5)]
        vals = ranges[idx]
        vals = vals[np.isfinite(vals) & (vals < np.inf)]
        return float(np.mean(vals)) if vals.size > 0 else None

    def _get_avg_wide(self, ranges, center_deg, half_window_deg):
        idx = [(center_deg + i) % 360 for i in range(-half_window_deg, half_window_deg + 1)]
        vals = ranges[idx]
        vals = vals[np.isfinite(vals) & (vals < np.inf)]
        return float(np.mean(vals)) if vals.size > 0 else None

    def _probe_asym(self, ranges, axis_deg, delta_deg):
        a_plus = self._get_avg_wide(ranges, (axis_deg + delta_deg) % 360, 4)
        a_minus = self._get_avg_wide(ranges, (axis_deg - delta_deg) % 360, 4)
        if a_plus is None or a_minus is None:
            return None
        return a_plus - a_minus

    def _min_distance_and_angle(self, ranges):
        if ranges is None:
            return None, None
        i = int(np.argmin(ranges))
        d = float(ranges[i])
        if not np.isfinite(d) or d == np.inf:
            return None, None
        return d, i

    def _twist_from_angle(self, angle_deg, speed):
        rad = math.radians(angle_deg)
        cmd = Twist()
        cmd.linear.x = -math.sin(rad) * speed
        cmd.linear.y = math.cos(rad) * speed
        return cmd

    def stop_robot(self):
        self.cmd_pub.publish(Twist())
        self.rot_pub.publish(Float32(0.0))

    def _finish(self):
        self.enabled = False
        self.pub_enabled.publish(Bool(False))
        self.state = self.IDLE
        self.stop_robot()

if __name__ == "__main__":
    rospy.init_node("go_home_node")
    node = GoHomeNode()
    rospy.spin()
