#!/usr/bin/env python3
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Bool

class GoHomeNode:
    def _init_(self):
        rospy.init_node("go_home_node", anonymous=False)

        # --- Params (tune if needed) ---
        self.target_m = rospy.get_param("~target_distance_m", 2.5)   # target distance to each wall
        self.sector_deg = rospy.get_param("~sector_deg", 12)         # half-width of angle sector
        self.k = rospy.get_param("~k", 0.45)                         # proportional gain
        self.max_cmd = rospy.get_param("~max_cmd", 0.8)              # clamp command magnitude
        self.diff_tol = rospy.get_param("~diff_tol_m", 0.20)         # tolerate L-R and F-B diff
        self.center_tol = rospy.get_param("~center_tol_m", 0.35)     # tolerate average distance vs target
        self.min_safe = rospy.get_param("~min_safe_m", 0.45)         # if any wall closer than this -> stop
        self.max_time = rospy.get_param("~max_time_s", 45.0)         # never run forever
        self.cmd_rate_hz = rospy.get_param("~rate_hz", 20.0)

        # --- ROS I/O ---
        self.twist_pub = rospy.Publisher("motor_control/twist", Twist, queue_size=10)
        self.enabled_pub = rospy.Publisher("go_home/enabled", Bool, queue_size=10, latch=True)

        self.scan = None
        rospy.Subscriber("lidar/scans", LaserScan, self._on_scan, queue_size=1)

        self.enabled = False
        self.start_time = None

        self.srv = rospy.Service("go_home/enable", SetBool, self._on_enable)

        rospy.on_shutdown(self._shutdown)

    # ----------------- ROS callbacks -----------------
    def _on_scan(self, msg: LaserScan):
        # Your lidar_node publishes 360 ranges; values look like mm (e.g., 15000 init)
        ranges = np.array(msg.ranges, dtype=np.float32)

        # Convert to meters if values look like mm
        # Heuristic: if median > 50, assume mm
        finite = ranges[np.isfinite(ranges)]
        if finite.size > 10 and np.nanmedian(finite) > 50.0:
            ranges = ranges / 1000.0

        self.scan = ranges

    def _on_enable(self, req):
        if req.data:
            self.enabled = True
            self.start_time = rospy.Time.now()
            self.enabled_pub.publish(Bool(True))
            rospy.loginfo("GoHome: enabled")
        else:
            self._stop_and_disable("disabled by request")
        return SetBoolResponse(success=True, message="ok")

    # ----------------- Core logic -----------------
    def _sector_dist(self, center_deg):
        """
        Robust distance estimate for a direction using a small angle sector.
        Uses percentile to reduce the effect of a single bad beam.
        """
        if self.scan is None or self.scan.size < 360:
            return None

        half = int(self.sector_deg)
        angles = (np.arange(center_deg - half, center_deg + half + 1) % 360).astype(int)
        vals = self.scan[angles]

        # Filter invalids
        vals = vals[np.isfinite(vals)]
        vals = vals[(vals > 0.05) & (vals < 15.0)]  # meters
        if vals.size < 3:
            return None

        # Use 30th percentile (closer obstacles dominate, but not one outlier)
        return float(np.percentile(vals, 30))

    def _stop_and_disable(self, reason):
        self.enabled = False
        self.start_time = None

        # publish zero once (and a few times to be safe)
        z = Twist()
        for _ in range(3):
            self.twist_pub.publish(z)

        self.enabled_pub.publish(Bool(False))
        rospy.logwarn(f"GoHome: stopped ({reason})")

    def run(self):
        rate = rospy.Rate(self.cmd_rate_hz)

        while not rospy.is_shutdown():
            if not self.enabled:
                rate.sleep()
                continue

            # Timeout safety
            if self.start_time is not None:
                dt = (rospy.Time.now() - self.start_time).to_sec()
                if dt > self.max_time:
                    self._stop_and_disable("timeout")
                    rate.sleep()
                    continue

            d_front = self._sector_dist(0)
            d_back  = self._sector_dist(180)
            d_left  = self._sector_dist(90)
            d_right = self._sector_dist(270)

            if None in (d_front, d_back, d_left, d_right):
                self._stop_and_disable("missing lidar data")
                rate.sleep()
                continue

            # Safety stop if too close to any wall
            if min(d_front, d_back, d_left, d_right) < self.min_safe:
                self._stop_and_disable("too close to wall")
                rate.sleep()
                continue

            # Center criteria:
            # 1) front/back difference small, left/right difference small
            # 2) average distance close to target (roughly centered in room size)
            fb_diff = d_front - d_back
            lr_diff = d_right - d_left
            fb_avg = 0.5 * (d_front + d_back)
            lr_avg = 0.5 * (d_left + d_right)

            if (abs(fb_diff) < self.diff_tol and
                abs(lr_diff) < self.diff_tol and
                abs(fb_avg - self.target_m) < self.center_tol and
                abs(lr_avg - self.target_m) < self.center_tol):
                self._stop_and_disable("reached center")
                rate.sleep()
                continue

            # Control:
            # MotorControl original mapping (from your code):
            #   desired_h = msg.linear.y   (forward/back)
            #   desired_v = msg.linear.x   (right/left)
            #
            # So:
            #   forward command -> linear.y
            #   right command   -> linear.x
            #
            # If front > back => move forward (away from back wall) => +linear.y
            # If right > left => move right (away from left wall)  => +linear.x
            cmd_forward = np.clip(self.k * fb_diff, -self.max_cmd, self.max_cmd)
            cmd_right   = np.clip(self.k * lr_diff, -self.max_cmd, self.max_cmd)

            # Normalize if both are large (keep direction, cap magnitude)
            mag = np.hypot(cmd_forward, cmd_right)
            if mag > self.max_cmd:
                scale = self.max_cmd / mag
                cmd_forward *= scale
                cmd_right *= scale

            t = Twist()
            t.linear.x = float(cmd_right)    # right/left
            t.linear.y = float(cmd_forward)  # forward/back
            t.angular.z = 0.0
            self.twist_pub.publish(t)

            rate.sleep()

    def _shutdown(self):
        self._stop_and_disable("shutdown")


if _name_ == "_main_":
    node = GoHomeNode()
    node.run()