#!/usr/bin/env python3
"""GoHomeNode -- pure distance-based alignment and centering.

Operates inside a symmetric square room (5 m × 5 m).  Uses raw LIDAR
distance measurements -- no SLAM, no RANSAC, no wall fitting.

Algorithm
---------
Phase 1 (ALIGN):  Rotate until the robot is parallel to the walls.
    For each cardinal direction c ∈ {0°, 90°, 180°, 270°} and probe
    offsets α ∈ {30°, 45°}, compare d(c+α) vs d(c-α).  In a perfect
    square room a perfectly-aligned robot sees d(c+α) == d(c-α).
    The normalised difference Σ[(d+ − d−) / (d+ + d−)] drives a
    P-controller on angular.z with a minimum-velocity deadband clamp.

Phase 2 (CENTER):  Translate (no rotation) until equidistant from
    opposite walls.
        error_x = (d_front − d_back) / 2
        error_y = (d_left  − d_right) / 2
    A P-controller on linear.x / linear.y with deadband clamp.

Deterministic termination
-------------------------
Each phase requires N consecutive in-tolerance frames before
transitioning.  A VERIFY state re-checks both criteria; failures
loop back to the relevant phase.  A global timeout (60 s) guarantees
the routine never runs forever.
"""

import rospy
import math
import numpy as np
from abstract_node import AbstractNode
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse


class GoHomeNode(AbstractNode):
    """ROS node: align to walls, drive to room centre, stop.

    Publishers   motor_control/twist, go_home/enabled
    Subscriber   lidar/scans
    Service      /go_home/enable  (SetBool)
    """

    # -- States ----------------------------------------------------------
    ST_IDLE   = 0
    ST_ALIGN  = 1
    ST_CENTER = 2
    ST_VERIFY = 3

    # -- Confirmation frame counts --------------------------------------
    ALIGN_CONFIRM   = 5
    CENTER_CONFIRM  = 5

    # -- Tolerances ------------------------------------------------------
    ALIGN_TOL         = 0.005   # normalised symmetry-error threshold
    CENTER_TOL        = 0.10    # metres from room centre
    VERIFY_ALIGN_TOL  = 0.008   # slightly relaxed for verify
    VERIFY_CENTER_TOL = 0.15    # slightly relaxed for verify

    # -- Velocity limits -------------------------------------------------
    MAX_ANG_V = 0.45
    MIN_ANG_V = 0.20
    KP_ANG    = 30.0            # P-gain: error → angular velocity

    MAX_LIN_V = 0.55
    MIN_LIN_V = 0.30
    KP_LIN    = 2.5             # P-gain: error → linear velocity

    # -- Beam sampling ---------------------------------------------------
    BEAM_HALF_WIN = 10          # ± rays around target angle
    RANGE_MIN_M   = 0.10       # reject closer than 10 cm
    RANGE_MAX_M   = 12.0       # reject farther than 12 m

    # -- Symmetry probes -------------------------------------------------
    CARDINALS    = [0, 90, 180, 270]
    PROBE_ALPHAS = [30, 45]

    # -- Timeout ---------------------------------------------------------
    GLOBAL_TIMEOUT_S = 60.0

    def __init__(self):
        super().__init__('go_home', 'Go Home')

        # -- Runtime state -----------------------------------------------
        self.enabled = False
        self.state   = self.ST_IDLE
        self._align_count  = 0
        self._center_count = 0
        self._start_time   = None

        # -- ROS I/O -----------------------------------------------------
        self.cmd_pub = rospy.Publisher(
            'motor_control/twist', Twist, queue_size=1, tcp_nodelay=True)
        self.pub_enabled = rospy.Publisher(
            'go_home/enabled', Bool, queue_size=1, latch=True)

        rospy.Subscriber(
            'lidar/scans', LaserScan, self._on_lidar,
            queue_size=1, tcp_nodelay=True)

        self.srv = rospy.Service(
            '/go_home/enable', SetBool, self._on_enable)

        self._publish_enabled()
        rospy.loginfo("[GoHome] Node started (distance-based).")

    # ================================================================
    #  Service
    # ================================================================

    def _publish_enabled(self):
        self.pub_enabled.publish(Bool(data=self.enabled))

    def _on_enable(self, req):
        self.enabled = bool(req.data)
        if self.enabled:
            self.state = self.ST_ALIGN
            self._align_count  = 0
            self._center_count = 0
            self._start_time   = rospy.Time.now()
            rospy.loginfo("[GoHome] Enabled → ALIGN")
        else:
            self._hard_stop()
            self.state = self.ST_IDLE
            rospy.loginfo("[GoHome] Disabled → IDLE")
        self._publish_enabled()
        return SetBoolResponse(
            success=True,
            message="GoHome enabled: %s" % self.enabled)

    # ================================================================
    #  Beam sampling
    # ================================================================

    def _get_dist(self, ranges_m, angle_deg):
        """Median of ±BEAM_HALF_WIN rays around *angle_deg*.

        Returns distance in metres, or None if insufficient valid rays.
        """
        n = len(ranges_m)
        indices = [(int(round(angle_deg)) + i) % n
                   for i in range(-self.BEAM_HALF_WIN,
                                   self.BEAM_HALF_WIN + 1)]
        vals = ranges_m[indices]
        good = vals[(vals > self.RANGE_MIN_M)
                     & (vals < self.RANGE_MAX_M)
                     & np.isfinite(vals)]
        if len(good) < 3:
            return None
        return float(np.median(good))

    # ================================================================
    #  Alignment error  (Phase 1)
    # ================================================================

    def _compute_alignment_error(self, ranges_m):
        """Signed alignment error from beam-symmetry probes.

        For each cardinal direction c and probe offset α, we compare
        d(c+α) and d(c-α).  A perfectly-aligned robot in a square room
        yields d(c+α) == d(c-α).  The normalised difference
        (d+ − d−) / (d+ + d−) is proportional to sin(θ) for small θ.

        Returns (error_float, valid_bool).
        """
        accum = 0.0
        count = 0

        for c in self.CARDINALS:
            for alpha in self.PROBE_ALPHAS:
                d_plus  = self._get_dist(ranges_m, (c + alpha) % 360)
                d_minus = self._get_dist(ranges_m, (c - alpha) % 360)
                if d_plus is None or d_minus is None:
                    continue
                s = d_plus + d_minus
                if s < 0.01:
                    continue
                accum += (d_plus - d_minus) / s
                count += 1

        if count == 0:
            return 0.0, False
        return accum / count, True

    # ================================================================
    #  Centering error  (Phase 2)
    # ================================================================

    def _compute_center_error(self, ranges_m):
        """Compute (error_x, error_y, dist) for centering.

        error_x = (d_front − d_back)  / 2   → linear.x direction
        error_y = (d_left  − d_right) / 2   → linear.y direction

        Returns (error_x, error_y, dist_to_center, valid_bool).
        """
        d_front = self._get_dist(ranges_m, 0)
        d_back  = self._get_dist(ranges_m, 180)
        d_right = self._get_dist(ranges_m, 90)
        d_left  = self._get_dist(ranges_m, 270)

        if None in (d_front, d_back, d_right, d_left):
            return 0.0, 0.0, 0.0, False

        error_x = (d_front - d_back) / 2.0
        error_y = (d_left - d_right) / 2.0
        dist    = math.hypot(error_x, error_y)
        return error_x, error_y, dist, True

    # ================================================================
    #  Motor helpers
    # ================================================================

    def _hard_stop(self):
        """Send 3 consecutive zero-velocity commands to guarantee stop."""
        zero = Twist()
        for _ in range(3):
            self.cmd_pub.publish(zero)
            rospy.sleep(0.05)

    def _finish(self):
        """Task complete: hard stop, disable, reset state."""
        self._hard_stop()
        self.enabled = False
        self.state   = self.ST_IDLE
        self._publish_enabled()
        rospy.loginfo("[GoHome] ✓ Task finished — robot stopped at centre.")

    # ================================================================
    #  Main LIDAR callback
    # ================================================================

    def _on_lidar(self, msg):
        if not self.enabled or self.state == self.ST_IDLE:
            return

        # -- Global timeout guard ----------------------------------------
        elapsed = (rospy.Time.now() - self._start_time).to_sec()
        if elapsed > self.GLOBAL_TIMEOUT_S:
            rospy.logwarn(
                "[GoHome] Global timeout (%.0f s). Aborting.", elapsed)
            self._hard_stop()
            self.enabled = False
            self.state   = self.ST_IDLE
            self._publish_enabled()
            return

        # -- Convert mm → m ---------------------------------------------
        ranges_m = np.array(msg.ranges, dtype=np.float64) / 1000.0

        cmd = Twist()

        # ============================================================
        #  ST_ALIGN — rotate only
        # ============================================================
        if self.state == self.ST_ALIGN:
            ang_err, valid = self._compute_alignment_error(ranges_m)
            if not valid:
                self.cmd_pub.publish(cmd)
                return

            if abs(ang_err) <= self.ALIGN_TOL:
                # Within tolerance — stop and count confirmation frames
                self._align_count += 1
                self.cmd_pub.publish(cmd)  # zero velocity

                if self._align_count >= self.ALIGN_CONFIRM:
                    rospy.loginfo(
                        "[GoHome] Aligned (err=%.4f, %d frames). "
                        "Hard stop → CENTER",
                        ang_err, self._align_count)
                    self._hard_stop()
                    rospy.sleep(0.3)
                    self._align_count  = 0
                    self._center_count = 0
                    self.state = self.ST_CENTER
                return

            # Outside tolerance — reset counter, apply P-control
            self._align_count = 0

            v_ang = self.KP_ANG * ang_err
            # Deadband clamp: ensure we overcome static friction
            if 0 < abs(v_ang) < self.MIN_ANG_V:
                v_ang = math.copysign(self.MIN_ANG_V, v_ang)
            v_ang = max(-self.MAX_ANG_V, min(self.MAX_ANG_V, v_ang))

            cmd.angular.z = v_ang
            self.cmd_pub.publish(cmd)

            rospy.loginfo_throttle(
                2.0,
                "[GoHome] ALIGN  err=%.4f  cmd_ang=%.3f  elapsed=%.0fs",
                ang_err, v_ang, elapsed)
            return

        # ============================================================
        #  ST_CENTER — translate only
        # ============================================================
        if self.state == self.ST_CENTER:
            err_x, err_y, dist, valid = self._compute_center_error(ranges_m)
            if not valid:
                self.cmd_pub.publish(cmd)
                return

            # Check alignment drift — if bad, go back to ALIGN
            ang_err, ang_valid = self._compute_alignment_error(ranges_m)
            if ang_valid and abs(ang_err) > self.VERIFY_ALIGN_TOL * 2:
                rospy.loginfo(
                    "[GoHome] Heading drift (%.4f) during CENTER → ALIGN",
                    ang_err)
                self._hard_stop()
                self._center_count = 0
                self._align_count  = 0
                self.state = self.ST_ALIGN
                return

            if dist <= self.CENTER_TOL:
                # Within tolerance — stop and count
                self._center_count += 1
                self.cmd_pub.publish(cmd)  # zero velocity

                if self._center_count >= self.CENTER_CONFIRM:
                    rospy.loginfo(
                        "[GoHome] Centred (dist=%.3f m, %d frames). "
                        "Hard stop → VERIFY",
                        dist, self._center_count)
                    self._hard_stop()
                    rospy.sleep(0.3)
                    self._center_count = 0
                    self.state = self.ST_VERIFY
                return

            # Outside tolerance — reset counter, apply P-control
            self._center_count = 0

            speed = self.KP_LIN * dist
            # Deadband clamp
            if 0 < speed < self.MIN_LIN_V:
                speed = self.MIN_LIN_V
            speed = min(speed, self.MAX_LIN_V)

            # Direction unit vector scaled by speed
            cmd.linear.x = (err_x / dist) * speed
            cmd.linear.y = (err_y / dist) * speed
            self.cmd_pub.publish(cmd)

            rospy.loginfo_throttle(
                2.0,
                "[GoHome] CENTER  dist=%.3f  err_x=%.3f  err_y=%.3f  "
                "speed=%.3f  elapsed=%.0fs",
                dist, err_x, err_y, speed, elapsed)
            return

        # ============================================================
        #  ST_VERIFY — one-shot check of both criteria
        # ============================================================
        if self.state == self.ST_VERIFY:
            ang_err, ang_valid = self._compute_alignment_error(ranges_m)
            err_x, err_y, dist, ctr_valid = \
                self._compute_center_error(ranges_m)

            if not ang_valid or not ctr_valid:
                # Bad scan — stay in VERIFY, try next frame
                self.cmd_pub.publish(cmd)
                return

            heading_ok = abs(ang_err) < self.VERIFY_ALIGN_TOL
            center_ok  = dist < self.VERIFY_CENTER_TOL

            if heading_ok and center_ok:
                rospy.loginfo(
                    "[GoHome] VERIFY PASS  ang=%.4f  dist=%.3f",
                    ang_err, dist)
                self._finish()
                return

            if not heading_ok:
                rospy.loginfo(
                    "[GoHome] VERIFY FAIL heading (%.4f) → ALIGN", ang_err)
                self._align_count = 0
                self.state = self.ST_ALIGN
                self.cmd_pub.publish(cmd)
                return

            if not center_ok:
                rospy.loginfo(
                    "[GoHome] VERIFY FAIL centre (%.3f m) → CENTER", dist)
                self._center_count = 0
                self.state = self.ST_CENTER
                self.cmd_pub.publish(cmd)
                return

    # ================================================================
    #  Compatibility stubs
    # ================================================================

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


# ====================================================================
if __name__ == "__main__":
    rospy.init_node("go_home_node")
    try:
        node = GoHomeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass