#!/usr/bin/env python3
"""Room-centring behaviour for FOMA, built from two reusable primitives."""

import math
import threading
import time
from collections import namedtuple

import cv2
import numpy as np
import rospy
from abstract_node import AbstractNode
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse

#  derived once per scan.
Measurement = namedtuple(
    'Measurement',
    'ranges walls err_x err_y dist ctr_valid ang_err ang_valid')


class Status:
    SUCCESS   = 'success'
    TIMEOUT   = 'timeout'
    CANCELLED = 'cancelled'
    NO_SCAN   = 'no_scan'
    UNSETTLED = 'unsettled'
    BUSY      = 'busy'


class PhaseResult(namedtuple('PhaseResult', 'status error elapsed')):
    """Outcome of a phase.  `error` is the residual it finished on."""

    __slots__ = ()

    @property
    def ok(self):
        return self.status == Status.SUCCESS

    def __bool__(self):
        return self.ok

    def __str__(self):
        return "%s (err=%.4f, %.1fs)" % (self.status, self.error, self.elapsed)


class GoHomeNode(AbstractNode):
    #  Phase labels
    ST_IDLE   = 0
    ST_ALIGN  = 1
    ST_CENTER = 2
    ST_VERIFY = 3

    STATE_NAMES = {ST_IDLE: 'IDLE', ST_ALIGN: 'ALIGN',
                   ST_CENTER: 'CENTER', ST_VERIFY: 'VERIFY'}

    # Time, not frames: at 100 Hz a frame count confirms before the drive settles.
    ALIGN_SETTLE_S  = 0.3
    CENTER_SETTLE_S = 0.3

    #  Tolerances
    ALIGN_TOL         = 0.005   # normalised symmetry-error threshold
    CENTER_TOL        = 0.03    # metres from room centre
    VERIFY_ALIGN_TOL  = 0.008   # slightly relaxed for the final check
    VERIFY_CENTER_TOL = 0.05

    #  Velocity limits: motor duty in [-1, 1], not m/s.
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

    #  Routine
    GLOBAL_TIMEOUT_S = 90.0
    CONTROL_HZ       = 100.0    # matches the lidar publish rate
    GO_HOME_PASSES   = 2        # centre+align retries before giving up
    SCAN_WAIT_S      = 2.0      # grace period for the first scan

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
        self.align_settle  = float(g('~align_settle', self.ALIGN_SETTLE_S))
        self.center_settle = float(g('~center_settle', self.CENTER_SETTLE_S))
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
        self.timeout_s  = float(g('~timeout', self.GLOBAL_TIMEOUT_S))
        self.control_hz = float(g('~control_hz', self.CONTROL_HZ))
        self.passes     = int(g('~go_home_passes', self.GO_HOME_PASSES))

        #  Runtime state
        self.enabled = False            # a go_home() routine is running
        self.state   = self.ST_IDLE
        self._latest   = None           # most recent Measurement
        self._last_cmd = Twist()
        self._measuring = 0             # >0 while any phase needs measurements
        self._phase_start = None
        self._cancel = threading.Event()
        self._lock   = threading.Lock()
        self._thread = None

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

        # Both behaviours drive motor_control/twist, so they must not overlap.
        self._rotate_active = False
        rospy.Subscriber(
            'rotate/active', Bool, self._on_rotate_active, queue_size=1)

        self.srv = rospy.Service(
            '/go_home/enable', SetBool, self._on_enable)

        rospy.on_shutdown(self.cancel)

        self._publish_enabled()
        rospy.loginfo(
            "[GoHome] Node started.  max_lin_v=%.2f kp_lin=%.1f "
            "center_tol=%.3f m  align_tol=%.4f  viz=%s",
            self.max_lin_v, self.kp_lin, self.center_tol, self.align_tol,
            self._viz_on)

    #  Service / lifecycle

    def _publish_enabled(self):
        self.pub_enabled.publish(Bool(data=self.enabled))

    def _on_enable(self, req):
        """SetBool entry point.  Returns immediately; the run is threaded."""
        if bool(req.data):
            started = self.start()
            msg = ("GoHome started" if started
                   else "GoHome already running")
        else:
            self.cancel()
            msg = "GoHome stopped"
        rospy.loginfo("[GoHome] %s", msg)
        return SetBoolResponse(success=True, message=msg)

    def arm(self):
        """Clear a latched cancel so the phases may run again."""
        self._cancel.clear()

    def _on_rotate_active(self, msg):
        self._rotate_active = bool(msg.data)

    def start(self):
        """Kick off go_home() on a worker thread.  False if already running."""
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return False
            if self._rotate_active:
                # Symmetric with rotate_node's own check on go_home/enabled.
                rospy.logwarn("[GoHome] refused: a rotation is in progress")
                return False
            self._cancel.clear()
            self._thread = threading.Thread(
                target=self._routine, name='go_home', daemon=True)
            self._thread.start()
            return True

    def cancel(self, join_timeout=1.0):
        """Ask a running routine to stop and bring the robot to rest."""
        self._cancel.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=join_timeout)
        self._hard_stop()

    def _routine(self):
        result = self.go_home()
        level = rospy.loginfo if result.ok else rospy.logwarn
        level("[GoHome] routine finished: %s", result)

    #  Measurement

    def _get_dist(self, ranges_m, angle_deg):
        """Median of ±BEAM_HALF_WIN rays around *angle_deg*."""
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

    def _compute_alignment_error(self, ranges_m):
        """Signed alignment error from beam-symmetry probes.

        Only a true heading error once centred: off-centre reads non-zero even
        when square, which is why go_home() centres first.
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

    def _wall_distances(self, ranges_m):
        """Cardinal wall distances (forward, backward, left, right) in metres."""
        return (self._get_dist(ranges_m, 0),     # forward wall
                self._get_dist(ranges_m, 180),   # backward wall
                self._get_dist(ranges_m, 90),    # left wall
                self._get_dist(ranges_m, 270))   # right wall

    def _compute_center_error(self, walls):
        """(error_x, error_y, dist) from the four wall distances.

        error_x = (right - left)/2 -> linear.x  (+ = right)
        error_y = (fwd - back)/2   -> linear.y  (+ = forward)
        Measured along the robot's own axes, so it stays valid while crooked.
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

    def _measure(self, ranges_m):
        walls = self._wall_distances(ranges_m)
        err_x, err_y, dist, ctr_valid = self._compute_center_error(walls)
        ang_err, ang_valid = self._compute_alignment_error(ranges_m)
        return Measurement(ranges_m, walls, err_x, err_y, dist, ctr_valid,
                           ang_err, ang_valid)

    def _on_lidar(self, msg):
        """Sensor sink only — never blocks and never drives the motors."""
        # ~0.5 ms a scan at 100 Hz: only sample when a phase or the overlay needs it.
        if not self._measuring and not self._viz_due():
            return

        ranges_m = np.array(msg.ranges, dtype=np.float64) / 1000.0
        m = self._measure(ranges_m)
        self._latest = m
        self._render_viz(m, self._last_cmd, self._elapsed())

    def _elapsed(self):
        if self._phase_start is None:
            return 0.0
        return time.monotonic() - self._phase_start

    def _begin_measuring(self):
        """Ask the scan callback to keep deriving measurements."""
        with self._lock:
            self._measuring += 1

    def _end_measuring(self):
        """Release one claim; True while some other caller still needs them."""
        with self._lock:
            self._measuring = max(0, self._measuring - 1)
            return self._measuring > 0

    #  Motor helpers

    def _publish(self, cmd):
        self._last_cmd = cmd
        self.cmd_pub.publish(cmd)

    def _hard_stop(self):
        zero = Twist()
        for _ in range(3):
            self._publish(zero)
            rospy.sleep(0.05)

    def stop_robot(self):
        self._publish(Twist())

    #  Phase primitives. _run_phase holds what they share, sense/drive what they do not.

    def _sense_center(self, m):
        return m.ctr_valid, m.dist

    def _drive_center(self, m):
        """P-control toward the room centre along the robot's own axes."""
        cmd = Twist()
        speed = self.kp_lin * m.dist
        if 0 < speed < self.min_lin_v:      # deadband: beat static friction
            speed = self.min_lin_v
        speed = min(speed, self.max_lin_v)
        cmd.linear.x = (m.err_x / m.dist) * speed
        cmd.linear.y = (m.err_y / m.dist) * speed
        return cmd

    def _sense_align(self, m):
        return m.ang_valid, abs(m.ang_err)

    def _drive_align(self, m):
        cmd = Twist()
        v_ang = self.kp_ang * m.ang_err
        if 0 < abs(v_ang) < self.min_ang_v:  # deadband: beat static friction
            v_ang = math.copysign(self.min_ang_v, v_ang)
        cmd.angular.z = max(-self.max_ang_v, min(self.max_ang_v, v_ang))
        return cmd

    def _run_phase(self, label, state, sense, drive, tol, settle_s, timeout):
        """Closed loop shared by every phase."""
        prev_state = self.state
        self._begin_measuring()
        self.state = state
        started = time.monotonic()
        self._phase_start = started
        deadline = started + timeout
        rate = rospy.Rate(self.control_hz)
        inside_since = None
        residual = float('inf')

        try:
            while not rospy.is_shutdown():
                now = time.monotonic()

                if self._cancel.is_set():
                    self._hard_stop()
                    return PhaseResult(Status.CANCELLED, residual,
                                       now - started)
                if now > deadline:
                    self._hard_stop()
                    rospy.logwarn("[GoHome] %s timed out after %.1f s "
                                  "(residual %.4f)", label, now - started,
                                  residual)
                    return PhaseResult(Status.TIMEOUT, residual, now - started)

                m = self._latest
                if m is None:
                    # No scan yet — hold still rather than driving blind.
                    if now - started > self.SCAN_WAIT_S:
                        self._hard_stop()
                        return PhaseResult(Status.NO_SCAN, residual,
                                           now - started)
                    self._publish(Twist())
                    rate.sleep()
                    continue

                valid, residual = sense(m)
                if not valid:
                    self._publish(Twist())
                    inside_since = None
                    rate.sleep()
                    continue

                if residual <= tol:
                    self._publish(Twist())      # coast to a stop and settle
                    if inside_since is None:
                        inside_since = now
                    elif now - inside_since >= settle_s:
                        self._hard_stop()
                        rospy.loginfo(
                            "[GoHome] %s done: residual=%.4f after %.1f s",
                            label, residual, now - started)
                        return PhaseResult(Status.SUCCESS, residual,
                                           now - started)
                else:
                    inside_since = None
                    self._publish(drive(m))
                    rospy.loginfo_throttle(
                        2.0, "[GoHome] %s residual=%.4f (tol %.4f) %.0fs",
                        label, residual, tol, now - started)

                rate.sleep()

            self._hard_stop()
            return PhaseResult(Status.CANCELLED, residual,
                               time.monotonic() - started)
        finally:
            still_driving = self._end_measuring()
            self.state = prev_state if still_driving else self.ST_IDLE
            self._phase_start = None

    def drive_to_center(self, tol=None, settle_s=None, timeout=None):
        """Translate the robot to the centre of the room."""
        return self._run_phase(
            'CENTER', self.ST_CENTER, self._sense_center, self._drive_center,
            self.center_tol if tol is None else tol,
            self.center_settle if settle_s is None else settle_s,
            self.timeout_s if timeout is None else timeout)

    def align_to_axes(self, tol=None, settle_s=None, timeout=None):
        """Rotate in place until the robot is square with the room axes."""
        return self._run_phase(
            'ALIGN', self.ST_ALIGN, self._sense_align, self._drive_align,
            self.align_tol if tol is None else tol,
            self.align_settle if settle_s is None else settle_s,
            self.timeout_s if timeout is None else timeout)

    def go_home(self, timeout=None, passes=None):
        """GOHOME: drive to the room centre, then square up to its axes."""
        budget = self.timeout_s if timeout is None else timeout
        passes = self.passes if passes is None else passes
        deadline = time.monotonic() + budget

        # Does NOT clear _cancel: start() arms the run before the worker exists.
        self.enabled = True
        self._publish_enabled()
        self._begin_measuring()
        started = time.monotonic()
        result = PhaseResult(Status.UNSETTLED, float('inf'), 0.0)

        try:
            for attempt in range(1, passes + 1):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return PhaseResult(Status.TIMEOUT, result.error,
                                       time.monotonic() - started)

                result = self.drive_to_center(timeout=remaining)
                if not result.ok:
                    return result

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return PhaseResult(Status.TIMEOUT, result.error,
                                       time.monotonic() - started)

                result = self.align_to_axes(timeout=remaining)
                if not result.ok:
                    return result

                # Confirm rather than assume — this is what guarantees the end state.
                self.state = self.ST_VERIFY
                m = self._latest
                if m is None:
                    return PhaseResult(Status.NO_SCAN, result.error,
                                       time.monotonic() - started)

                centred = m.ctr_valid and m.dist < self.verify_center_tol
                square  = m.ang_valid and abs(m.ang_err) < self.verify_align_tol
                if centred and square:
                    rospy.loginfo(
                        "[GoHome] VERIFY PASS  dist=%.3f m  ang=%+.4f  "
                        "(pass %d/%d)", m.dist, m.ang_err, attempt, passes)
                    return PhaseResult(Status.SUCCESS, m.dist,
                                       time.monotonic() - started)

                rospy.loginfo(
                    "[GoHome] VERIFY FAIL  dist=%.3f m (%s)  ang=%+.4f (%s) "
                    "— repeating (pass %d/%d)",
                    m.dist, "ok" if centred else "off",
                    m.ang_err, "ok" if square else "off", attempt, passes)

            return PhaseResult(Status.UNSETTLED, result.error,
                               time.monotonic() - started)
        finally:
            self._end_measuring()
            self.state = self.ST_IDLE
            self.enabled = False
            self._publish_enabled()
            self._hard_stop()

    #  Top-down view, robot centred. Drawn on the scan thread; run() displays it.

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
        """The ±α beam pairs whose asymmetry drives the ALIGN phase."""
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

    def _render_viz(self, m, cmd, elapsed):
        """Build the next overlay frame.  No-op above the configured FPS."""
        if not self._viz_due():
            return
        self._viz_last = time.monotonic()

        finite = m.ranges[np.isfinite(m.ranges)
                          & (m.ranges > self.RANGE_MIN_M)
                          & (m.ranges < self.RANGE_MAX_M)]
        self._viz_fit(finite)

        img = np.full((self._viz_size, self._viz_size, 3), self.C_BG,
                      dtype=np.uint8)
        self._viz_draw_grid(img)
        self._viz_draw_probes(img, m.ranges)
        self._viz_draw_points(img, m.ranges)
        self._viz_draw_room(img, m.walls)
        if m.ctr_valid:
            self._viz_draw_centre(img, m.err_x, m.err_y)
        self._viz_draw_robot(img)

        walls_txt = "  ".join(
            "%s=%s" % (name, "--" if d is None else "%.2f" % d)
            for name, d in zip(("F", "B", "L", "R"), m.walls))
        driving = bool(self._measuring)
        lines = [
            ("%s%s" % (self.STATE_NAMES[self.state],
                       "" if driving else "  (idle)"),
             self.C_TEXT if driving else self.C_TEXT_DIM),
            ("centre  dist=%s  err_x=%+.3f  err_y=%+.3f"
             % ("--" if not m.ctr_valid else "%.3f m" % m.dist,
                m.err_x, m.err_y), self.C_TEXT),
            ("heading err=%s  (tol %.4f)"
             % ("--" if not m.ang_valid else "%+.4f" % m.ang_err,
                self.align_tol), self.C_TEXT),
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


# ====================================================================
if __name__ == "__main__":
    rospy.init_node("go_home_node")
    try:
        node = GoHomeNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
