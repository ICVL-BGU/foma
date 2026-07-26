#!/usr/bin/env python3
import rospy
import math
import time
import cv2
import numpy as np
from abstract_node import AbstractNode
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse


class GoHomeNode(AbstractNode):
    #  States
    ST_IDLE   = 0
    ST_ALIGN  = 1
    ST_CENTER = 2
    ST_VERIFY = 3

    STATE_NAMES = {ST_IDLE: 'IDLE', ST_ALIGN: 'ALIGN',
                   ST_CENTER: 'CENTER', ST_VERIFY: 'VERIFY'}

    # Every tuning value below is a default; override any of them from the
    # go_home_node: section of config/foma_params.yaml.

    # Confirmation frame counts.  Scans arrive at 100 Hz, so the old count of
    # 5 confirmed a state after 50 ms — well before the drive train has
    # finished ramping down.  30 frames (~0.3 s) lets the robot actually
    # settle before we judge whether it is inside tolerance.
    ALIGN_CONFIRM   = 30
    CENTER_CONFIRM  = 30

    #  Tolerances
    ALIGN_TOL         = 0.005   # normalised symmetry-error threshold
    CENTER_TOL        = 0.03    # metres from room centre
    VERIFY_ALIGN_TOL  = 0.008   # slightly relaxed for verify
    VERIFY_CENTER_TOL = 0.05    # slightly relaxed for verify

    #  Velocity limits
    # linear.{x,y} and angular.z are normalised motor duty in [-1, 1]
    # (motor_control_node scales them by MOTOR_SPEED), not m/s.
    MAX_ANG_V = 0.45
    MIN_ANG_V = 0.20
    KP_ANG    = 30.0            # P-gain: error → angular velocity

    MAX_LIN_V = 0.95
    MIN_LIN_V = 0.25
    KP_LIN    = 5.0             # P-gain: error → linear velocity

    #  Beam sampling
    BEAM_HALF_WIN = 10          # ± rays around target angle
    RANGE_MIN_M   = 0.10       # reject closer than 10 cm
    RANGE_MAX_M   = 12.0       # reject farther than 12 m

    #  Symmetry probes
    CARDINALS    = [0, 90, 180, 270]
    PROBE_ALPHAS = [30, 45]

    #  Timeout
    GLOBAL_TIMEOUT_S = 90.0

    #  Visualisation
    VIZ_ENABLED  = True
    VIZ_FPS      = 20.0
    VIZ_SIZE     = 720
    VIZ_RANGE_M  = 0.0          # 0 → auto-fit the view to the scan
    ROBOT_SIZE_M = 0.576        # footprint, matches motor_control_node

    VIZ_WINDOW = "GoHome - room model"

    # Overlay colours (BGR)
    C_BG       = (24, 24, 24)
    C_GRID     = (58, 58, 58)
    C_POINTS   = (0, 200, 255)
    C_ROOM     = (0, 235, 130)
    C_ROBOT    = (245, 245, 245)
    C_HEADING  = (0, 150, 255)
    C_CENTRE   = (60, 60, 255)
    C_CARDINAL = (215, 130, 255)
    C_PROBE    = (110, 190, 110)
    C_TEXT     = (235, 235, 235)
    C_TEXT_DIM = (150, 150, 150)

    FONT = cv2.FONT_HERSHEY_SIMPLEX

    def __init__(self):
        super().__init__('go_home', 'Go Home')

        #  Tuning (private params, defaults above)
        g = rospy.get_param
        self.align_confirm     = int(g('~align_confirm', self.ALIGN_CONFIRM))
        self.center_confirm    = int(g('~center_confirm', self.CENTER_CONFIRM))
        self.align_tol         = float(g('~align_tol', self.ALIGN_TOL))
        self.center_tol        = float(g('~center_tol', self.CENTER_TOL))
        self.verify_align_tol  = float(g('~verify_align_tol',
                                         self.VERIFY_ALIGN_TOL))
        self.verify_center_tol = float(g('~verify_center_tol',
                                         self.VERIFY_CENTER_TOL))
        self.max_ang_v = float(g('~max_ang_v', self.MAX_ANG_V))
        self.min_ang_v = float(g('~min_ang_v', self.MIN_ANG_V))
        self.kp_ang    = float(g('~kp_ang', self.KP_ANG))
        self.max_lin_v = float(g('~max_lin_v', self.MAX_LIN_V))
        self.min_lin_v = float(g('~min_lin_v', self.MIN_LIN_V))
        self.kp_lin    = float(g('~kp_lin', self.KP_LIN))
        self.timeout_s = float(g('~timeout', self.GLOBAL_TIMEOUT_S))

        #  Runtime state
        self.enabled = False
        self.state   = self.ST_IDLE
        self._align_count  = 0
        self._center_count = 0
        self._start_time   = None

        #  Visualisation state
        self.viz_fps = float(g('~viz_fps', self.VIZ_FPS))
        self._viz_on = bool(g('~visualize', self.VIZ_ENABLED))
        self._viz_size = int(g('~viz_size', self.VIZ_SIZE))
        self._viz_fixed_range = float(g('~viz_range_m', self.VIZ_RANGE_M))
        self._viz_cx = self._viz_size // 2
        self._viz_cy = self._viz_size // 2
        self._viz_view = self._viz_fixed_range if self._viz_fixed_range > 0 else 3.0
        self._viz_ppm = 1.0
        self._viz_last = 0.0
        self._viz_frame = None

        #  ROS I/O
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
        rospy.loginfo(
            "[GoHome] Node started (distance-based).  max_lin_v=%.2f "
            "kp_lin=%.1f  center_tol=%.3f m  viz=%s",
            self.max_lin_v, self.kp_lin, self.center_tol, self._viz_on)

    #  Service

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

    #  Beam sampling

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

    #  Alignment error  (Phase 1)

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

    #  Centering error  (Phase 2)

    def _wall_distances(self, ranges_m):
        """Cardinal wall distances (forward, backward, left, right) in metres.

        Any entry may be None when that direction has too few valid rays.
        """
        return (self._get_dist(ranges_m, 0),     # forward wall
                self._get_dist(ranges_m, 180),   # backward wall
                self._get_dist(ranges_m, 90),    # left wall
                self._get_dist(ranges_m, 270))   # right wall

    def _compute_center_error(self, walls):
        """Compute (error_x, error_y, dist) for centering.

        Motor axis mapping (from motor_control_node.__handle_twist):
            linear.x → v_component:  positive = rightward  (toward LIDAR 270°)
            linear.y → h_component:  positive = forward    (toward LIDAR 0°)

        LIDAR direction convention (from motor_control_node.__check_blocking):
            0°   = Forward
            90°  = Left
            180° = Backward
            270° = Right

        error_x = (d_right − d_left)    / 2  → drives linear.x (right/left axis)
        error_y = (d_forward − d_back)  / 2  → drives linear.y (forward/back axis)

        When one wall is farther, the error is positive toward that wall,
        and the corresponding linear.{x,y} drives TOWARD it (negative feedback).

        Returns (error_x, error_y, dist_to_center, valid_bool).
        """
        d_fwd, d_bwd, d_left, d_right = walls

        if None in walls:
            return 0.0, 0.0, 0.0, False

        # linear.x > 0 → rightward (toward 270°): drive right when right wall is farther
        error_x = (d_right - d_left) / 2.0
        # linear.y > 0 → forward (toward 0°): drive forward when forward wall is farther
        error_y = (d_fwd - d_bwd) / 2.0
        dist    = math.hypot(error_x, error_y)
        return error_x, error_y, dist, True

    #  Motor helpers

    def _hard_stop(self):
        zero = Twist()
        for _ in range(3):
            self.cmd_pub.publish(zero)
            rospy.sleep(0.05)

    def _finish(self):
        self._hard_stop()
        self.enabled = False
        self.state   = self.ST_IDLE
        self._publish_enabled()
        rospy.loginfo("GoHome finished — robot stopped at centre.")

    #  Main LIDAR callback

    def _on_lidar(self, msg):
        active = self.enabled and self.state != self.ST_IDLE
        if not active and not self._viz_due():
            return

        #  Convert mm → m -
        ranges_m = np.array(msg.ranges, dtype=np.float64) / 1000.0

        # Sampled once per scan and shared by the controller and the overlay,
        # so the window always shows the values the robot is acting on.
        walls = self._wall_distances(ranges_m)
        err_x, err_y, dist, ctr_valid = self._compute_center_error(walls)
        ang_err, ang_valid = self._compute_alignment_error(ranges_m)

        elapsed = (0.0 if self._start_time is None
                   else (rospy.Time.now() - self._start_time).to_sec())

        cmd = self._step(err_x, err_y, dist, ctr_valid,
                         ang_err, ang_valid, elapsed)

        self._render_viz(ranges_m, walls, err_x, err_y, dist, ctr_valid,
                         ang_err, ang_valid, cmd, elapsed)

    def _step(self, err_x, err_y, dist, ctr_valid, ang_err, ang_valid,
              elapsed):
        """Run one control iteration.  Returns the Twist that was published."""
        cmd = Twist()

        if not self.enabled or self.state == self.ST_IDLE:
            return cmd

        #  Global timeout guard
        if elapsed > self.timeout_s:
            rospy.logwarn(
                "[GoHome] Global timeout (%.0f s). Aborting.", elapsed)
            self._hard_stop()
            self.enabled = False
            self.state   = self.ST_IDLE
            self._publish_enabled()
            return cmd

        #  ST_ALIGN — rotate only
        if self.state == self.ST_ALIGN:
            if not ang_valid:
                self.cmd_pub.publish(cmd)
                return cmd

            if abs(ang_err) <= self.align_tol:
                # Within tolerance — stop and count confirmation frames
                self._align_count += 1
                self.cmd_pub.publish(cmd)  # zero velocity

                if self._align_count >= self.align_confirm:
                    rospy.loginfo(
                        "[GoHome] Aligned (err=%.4f, %d frames). "
                        "Hard stop → CENTER",
                        ang_err, self._align_count)
                    self._hard_stop()
                    rospy.sleep(0.3)
                    self._align_count  = 0
                    self._center_count = 0
                    self.state = self.ST_CENTER
                return cmd

            # Outside tolerance — reset counter, apply P-control
            self._align_count = 0

            v_ang = self.kp_ang * ang_err
            # Deadband clamp: ensure to overcome static friction
            if 0 < abs(v_ang) < self.min_ang_v:
                v_ang = math.copysign(self.min_ang_v, v_ang)
            v_ang = max(-self.max_ang_v, min(self.max_ang_v, v_ang))

            cmd.angular.z = v_ang
            self.cmd_pub.publish(cmd)

            rospy.loginfo_throttle(
                2.0,
                "[GoHome] ALIGN  err=%.4f  cmd_ang=%.3f  elapsed=%.0fs",
                ang_err, v_ang, elapsed)
            return cmd

        #  ST_CENTER — translate only
        if self.state == self.ST_CENTER:
            if not ctr_valid:
                self.cmd_pub.publish(cmd)
                return cmd

            # Check alignment drift — if bad, go back to ALIGN
            if ang_valid and abs(ang_err) > self.verify_align_tol * 2:
                rospy.loginfo(
                    "[GoHome] Heading drift (%.4f) during CENTER → ALIGN",
                    ang_err)
                self._hard_stop()
                self._center_count = 0
                self._align_count  = 0
                self.state = self.ST_ALIGN
                return cmd

            if dist <= self.center_tol:
                # Within tolerance — stop and count
                self._center_count += 1
                self.cmd_pub.publish(cmd)  # zero velocity

                if self._center_count >= self.center_confirm:
                    rospy.loginfo(
                        "[GoHome] Centred (dist=%.3f m, %d frames). "
                        "Hard stop → VERIFY",
                        dist, self._center_count)
                    self._hard_stop()
                    rospy.sleep(0.3)
                    self._center_count = 0
                    self.state = self.ST_VERIFY
                return cmd

            self._center_count = 0

            speed = self.kp_lin * dist
            # Deadband clamp
            if 0 < speed < self.min_lin_v:
                speed = self.min_lin_v
            speed = min(speed, self.max_lin_v)

            # Direction unit vector scaled by speed
            cmd.linear.x = (err_x / dist) * speed
            cmd.linear.y = (err_y / dist) * speed
            self.cmd_pub.publish(cmd)

            rospy.loginfo_throttle(
                2.0,
                "[GoHome] CENTER  dist=%.3f  err_x=%.3f  err_y=%.3f  "
                "speed=%.3f  elapsed=%.0fs",
                dist, err_x, err_y, speed, elapsed)
            return cmd

        #  ST_VERIFY — one-shot check of both criteria
        if self.state == self.ST_VERIFY:
            if not ang_valid or not ctr_valid:
                # Bad scan — stay in VERIFY, try next frame
                self.cmd_pub.publish(cmd)
                return cmd

            heading_ok = abs(ang_err) < self.verify_align_tol
            center_ok  = dist < self.verify_center_tol

            if heading_ok and center_ok:
                rospy.loginfo(
                    "[GoHome] VERIFY PASS  ang=%.4f  dist=%.3f",
                    ang_err, dist)
                self._finish()
                return cmd

            if not heading_ok:
                rospy.loginfo(
                    "[GoHome] VERIFY FAIL heading (%.4f) → ALIGN", ang_err)
                self._align_count = 0
                self.state = self.ST_ALIGN
                self.cmd_pub.publish(cmd)
                return cmd

            if not center_ok:
                rospy.loginfo(
                    "[GoHome] VERIFY FAIL centre (%.3f m) → CENTER", dist)
                self._center_count = 0
                self.state = self.ST_CENTER
                self.cmd_pub.publish(cmd)
                return cmd

        return cmd

    #  Visualisation

    def _viz_px(self, forward_m, left_m):
        return (int(round(self._viz_cx - left_m * self._viz_ppm)),
                int(round(self._viz_cy - forward_m * self._viz_ppm)))

    def _viz_polar_px(self, angle_deg, dist_m):
        a = math.radians(angle_deg)
        return self._viz_px(dist_m * math.cos(a), dist_m * math.sin(a))

    def _viz_fit(self, valid_ranges):
        """Keep the whole room in frame without the scale twitching per frame."""
        if self._viz_fixed_range > 0:
            self._viz_view = self._viz_fixed_range
        elif valid_ranges.size:
            target = float(np.percentile(valid_ranges, 98)) * 1.15
            target = min(max(target, 1.0), self.RANGE_MAX_M)
            self._viz_view = 0.85 * self._viz_view + 0.15 * target
        # Margin so the HUD text never lands on top of the far wall.
        self._viz_ppm = (self._viz_size * 0.44) / max(self._viz_view, 0.1)

    def _viz_draw_grid(self, img):
        step = 0.5 if self._viz_view <= 3.0 else 1.0
        for i in range(1, int(self._viz_view / step) + 1):
            cv2.circle(img, (self._viz_cx, self._viz_cy),
                       int(round(i * step * self._viz_ppm)), self.C_GRID, 1)
        cv2.line(img, (0, self._viz_cy), (self._viz_size, self._viz_cy),
                 self.C_GRID, 1)
        cv2.line(img, (self._viz_cx, 0), (self._viz_cx, self._viz_size),
                 self.C_GRID, 1)
        cv2.putText(img, "ring = %.1f m" % step, (10, self._viz_size - 12),
                    self.FONT, 0.42, self.C_TEXT_DIM, 1, cv2.LINE_AA)

    def _viz_draw_points(self, img, ranges_m):
        """Every valid LIDAR return, placed relative to the robot."""
        idx = np.flatnonzero(np.isfinite(ranges_m)
                             & (ranges_m > self.RANGE_MIN_M)
                             & (ranges_m < self.RANGE_MAX_M))
        if idx.size == 0:
            return
        a = np.radians(idx.astype(np.float64))
        r = ranges_m[idx]
        xs = np.rint(self._viz_cx - r * np.sin(a) * self._viz_ppm).astype(int)
        ys = np.rint(self._viz_cy - r * np.cos(a) * self._viz_ppm).astype(int)
        inside = ((xs >= 0) & (xs < self._viz_size)
                  & (ys >= 0) & (ys < self._viz_size))
        for x, y in zip(xs[inside], ys[inside]):
            cv2.circle(img, (int(x), int(y)), 2, self.C_POINTS, -1, cv2.LINE_AA)

    def _viz_draw_room(self, img, walls):
        """Rectangle implied by the four cardinal wall distances."""
        d_fwd, d_bwd, d_left, d_right = walls
        if None in walls:
            cv2.putText(img, "room model: incomplete scan",
                        (10, self._viz_size - 32), self.FONT, 0.5,
                        self.C_CENTRE, 1, cv2.LINE_AA)
            return
        cv2.rectangle(img, self._viz_px(d_fwd, d_left),
                      self._viz_px(-d_bwd, -d_right), self.C_ROOM, 2)

        for angle, dist in ((0, d_fwd), (90, d_left),
                            (180, d_bwd), (270, d_right)):
            end = self._viz_polar_px(angle, dist)
            cv2.line(img, (self._viz_cx, self._viz_cy), end,
                     self.C_CARDINAL, 1, cv2.LINE_AA)
            cv2.circle(img, end, 4, self.C_CARDINAL, -1, cv2.LINE_AA)
            cv2.putText(img, "%.2f" % dist, (end[0] + 6, end[1] - 6),
                        self.FONT, 0.42, self.C_CARDINAL, 1, cv2.LINE_AA)

    def _viz_draw_probes(self, img, ranges_m):
        """The ±α beam pairs whose asymmetry drives the ALIGN state."""
        for c in self.CARDINALS:
            for alpha in self.PROBE_ALPHAS:
                for angle in ((c + alpha) % 360, (c - alpha) % 360):
                    d = self._get_dist(ranges_m, angle)
                    if d is None:
                        continue
                    cv2.line(img, (self._viz_cx, self._viz_cy),
                             self._viz_polar_px(angle, d),
                             self.C_PROBE, 1, cv2.LINE_AA)

    def _viz_draw_robot(self, img):
        half = int(round(self.ROBOT_SIZE_M / 2.0 * self._viz_ppm))
        cv2.rectangle(img, (self._viz_cx - half, self._viz_cy - half),
                      (self._viz_cx + half, self._viz_cy + half),
                      self.C_ROBOT, 2)
        cv2.arrowedLine(img, (self._viz_cx, self._viz_cy),
                        (self._viz_cx, self._viz_cy - int(half * 2.2)),
                        self.C_HEADING, 2, cv2.LINE_AA, tipLength=0.3)

    def _viz_draw_centre(self, img, err_x, err_y):
        # err_x is rightward, err_y forward — the room centre relative to us.
        target = self._viz_px(err_y, -err_x)
        cv2.drawMarker(img, target, self.C_CENTRE, cv2.MARKER_CROSS, 18, 2,
                       cv2.LINE_AA)
        cv2.circle(img, target, 7, self.C_CENTRE, 1, cv2.LINE_AA)
        cv2.arrowedLine(img, (self._viz_cx, self._viz_cy), target,
                        self.C_CENTRE, 1, cv2.LINE_AA, tipLength=0.25)

    def _viz_draw_legend(self, img):
        items = (("LIDAR returns", self.C_POINTS),
                 ("room model", self.C_ROOM),
                 ("wall probes", self.C_CARDINAL),
                 ("align probes", self.C_PROBE),
                 ("room centre", self.C_CENTRE))
        y0 = self._viz_size - 12 - 18 * len(items)
        for i, (label, colour) in enumerate(items):
            y = y0 + i * 18
            cv2.line(img, (self._viz_size - 150, y - 4),
                     (self._viz_size - 132, y - 4), colour, 2)
            cv2.putText(img, label, (self._viz_size - 126, y), self.FONT,
                        0.42, self.C_TEXT_DIM, 1, cv2.LINE_AA)

    def _viz_due(self):
        """True when the overlay is enabled and its next frame is owed."""
        return (self._viz_on and
                time.monotonic() - self._viz_last >= 1.0 / max(1.0, self.viz_fps))

    def _render_viz(self, ranges_m, walls, err_x, err_y, dist, ctr_valid,
                    ang_err, ang_valid, cmd, elapsed):
        """Build the next overlay frame.  No-op above the configured FPS."""
        if not self._viz_due():
            return
        self._viz_last = time.monotonic()

        finite = ranges_m[np.isfinite(ranges_m)
                          & (ranges_m > self.RANGE_MIN_M)
                          & (ranges_m < self.RANGE_MAX_M)]
        self._viz_fit(finite)

        img = np.full((self._viz_size, self._viz_size, 3), self.C_BG,
                      dtype=np.uint8)
        self._viz_draw_grid(img)
        self._viz_draw_probes(img, ranges_m)
        self._viz_draw_points(img, ranges_m)
        self._viz_draw_room(img, walls)
        if ctr_valid:
            self._viz_draw_centre(img, err_x, err_y)
        self._viz_draw_robot(img)

        walls_txt = "  ".join(
            "%s=%s" % (name, "--" if d is None else "%.2f" % d)
            for name, d in zip(("F", "B", "L", "R"), walls))
        lines = [
            ("%s%s" % (self.STATE_NAMES[self.state],
                       "" if self.enabled else "  (disabled)"),
             self.C_TEXT if self.enabled else self.C_TEXT_DIM),
            ("centre  dist=%s  err_x=%+.3f  err_y=%+.3f"
             % ("--" if not ctr_valid else "%.3f m" % dist, err_x, err_y),
             self.C_TEXT),
            ("heading err=%s  (tol %.4f)"
             % ("--" if not ang_valid else "%+.4f" % ang_err, self.align_tol),
             self.C_TEXT),
            ("cmd  x=%+.2f  y=%+.2f  w=%+.2f"
             % (cmd.linear.x, cmd.linear.y, cmd.angular.z), self.C_TEXT),
            ("walls  %s" % walls_txt, self.C_TEXT_DIM),
            ("stop tol %.3f m   elapsed %.1f s" % (self.center_tol, elapsed),
             self.C_TEXT_DIM),
        ]
        for i, (text, colour) in enumerate(lines):
            cv2.putText(img, text, (10, 24 + i * 20), self.FONT, 0.5, colour,
                        1, cv2.LINE_AA)
        self._viz_draw_legend(img)

        self._viz_frame = img

    def run(self):
        """Pump the visualisation window from the main thread."""
        if not self._viz_on:
            rospy.spin()
            return

        rate = rospy.Rate(self.viz_fps)
        while not rospy.is_shutdown():
            frame = self._viz_frame
            if frame is not None:
                try:
                    cv2.imshow(self.VIZ_WINDOW, frame)
                    cv2.waitKey(1)
                except cv2.error as e:
                    # No display available — keep driving the robot headless.
                    rospy.logwarn(
                        "[GoHome] Visualisation unavailable (%s); "
                        "continuing without it.", e)
                    self._viz_on = False
                    rospy.spin()
                    return
            rate.sleep()
        cv2.destroyWindow(self.VIZ_WINDOW)

    #  Compatibility stubs

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


# ====================================================================
if __name__ == "__main__":
    rospy.init_node("go_home_node")
    try:
        node = GoHomeNode()
        node.run()
    except rospy.ROSInterruptException:
        pass