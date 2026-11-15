#!/usr/bin/env python3
"""Room-centring behaviour for FOMA, built from two reusable primitives."""

import math
import threading
import time
from collections import deque, namedtuple

import cv2
import numpy as np
import rospy
from abstract_node import AbstractNode
from etc import scan_geometry as geom
from foma.srv import Float
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse

#  derived once per scan.  `fresh_t` = when the scan contents last changed
#  (lidar_node keeps republishing a dead sensor's buffer).
Measurement = namedtuple(
    'Measurement',
    'ranges walls err_x err_y dist ctr_valid ang_err ang_valid ang_conf '
    'fresh_t')


class Status:
    SUCCESS   = 'success'
    TIMEOUT   = 'timeout'
    CANCELLED = 'cancelled'
    NO_SCAN   = 'no_scan'
    UNSETTLED = 'unsettled'
    BUSY      = 'busy'
    DRIFT     = 'drift'


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
    # >= 2 lidar revolutions (~0.18 s each) plus the motor ramp-down.
    ALIGN_SETTLE_S  = 0.8
    CENTER_SETTLE_S = 0.8

    #  Tolerances
    ALIGN_TOL         = 2.5     # degrees; ~ shortest rotation the drive can do
    CENTER_TOL        = 0.03    # metres from room centre
    VERIFY_ALIGN_TOL  = 2.0     # between FINE_ALIGN_TOL and ALIGN_TOL
    VERIFY_CENTER_TOL = 0.05
    RELEASE_SCALE     = 1.5     # hysteresis: settling survives noise below this
    CENTER_DRIFT_DEG  = 6.0     # heading lost while centring -> re-align
    MIN_HEADING_CONF  = 0.5     # inlier fraction to trust the wall heading

    #  Velocity limits: motor duty in [-1, 1], not m/s.
    MAX_ANG_V = 0.45
    MIN_ANG_V = 0.20
    KP_ANG    = 0.015           # duty per degree of heading error
    # Wiring decides which angular.z sign reduces the error (/ALIGN_SIGN);
    # optionally checked during the first turn, see _check_sign.
    ALIGN_SIGN      = 1
    SIGN_AUTODETECT = True
    SIGN_PROBE_DEG  = 1.0       # x3 = error growth that proves the sign wrong
    # Smooth turn: min_ang_v only to break away, then a lower creep duty
    # (kinetic friction < static), and coast once the measured turn rate
    # would carry the robot to the target within STOP_LEAD_S.
    CREEP_ANG_V     = 0.17
    MOVING_DPS      = 3.0       # deg/s above which the robot counts as turning
    STOP_LEAD_S     = 0.25      # ~ lidar lag + motor ramp-down
    RATE_ALPHA      = 0.6
    RATE_WINDOW_S   = 0.15      # ~ one lidar revolution
    LAG_S           = 0.20      # scan age + half the rate window
    STALL_S         = 0.4       # not turning this long while commanded ->
    STALL_STEP      = 0.03      # raise the floor by this much
    # Final in-place squaring after centring (motor_control's heading hold
    # keeps the robot square while it drives, so this is usually a no-op).
    FINE_ALIGN_TOL  = 1.0       # degrees
    FINE_ALIGN_TIMEOUT_S = 12.0
    HEADING_OFFSET  = 0.0       # deg; lidar mounting yaw vs the chassis

    MAX_LIN_V = 0.95
    MIN_LIN_V = 0.25
    KP_LIN    = 5.0             # P-gain: error → linear velocity

    #  Beam sampling
    BEAM_HALF_WIN = 10          # ± rays around target angle
    RANGE_MIN_M   = 0.10       # reject closer than 10 cm
    RANGE_MAX_M   = 12.0       # reject farther than 12 m

    #  Routine
    GLOBAL_TIMEOUT_S = 90.0
    CONTROL_HZ       = 100.0    # matches the lidar publish rate
    GO_HOME_PASSES   = 3        # align+centre retries before giving up
    SCAN_WAIT_S      = 2.0      # grace period for the first scan
    STALE_S          = 0.5      # unchanged scan this long -> stop driving
    VERIFY_SCANS     = 5        # fresh scans the final check is a median of
    VERIFY_WINDOW_S  = 1.5

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
    C_DRIVE    = (0, 220, 255)      # commanded drive direction (yellow)
    C_MOTION   = (255, 80, 255)     # measured motion direction (magenta)
    C_TRAIL    = (255, 220, 0)      # path history (cyan)

    TRAIL_S      = 6.0              # path history drawn behind the robot
    MOTION_WIN_S = 0.5              # window the measured velocity spans

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
        self.release_scale = float(g('~release_scale', self.RELEASE_SCALE))
        self.center_drift  = float(g('~center_drift_deg',
                                     self.CENTER_DRIFT_DEG))
        self.min_conf      = float(g('~min_heading_conf',
                                     self.MIN_HEADING_CONF))
        # Shared with motor_control's heading hold, hence global.
        self.align_sign      = 1 if int(g('/ALIGN_SIGN', self.ALIGN_SIGN)) >= 0 else -1
        self.heading_offset  = float(g('/HEADING_OFFSET_DEG',
                                       self.HEADING_OFFSET))
        self.sign_autodetect = bool(g('~sign_autodetect', self.SIGN_AUTODETECT))
        self._sign_checked   = False    # sign check runs once per node lifetime
        self.creep_ang_v     = float(g('~creep_ang_v', self.CREEP_ANG_V))
        self.stop_lead_s     = float(g('~stop_lead_s', self.STOP_LEAD_S))
        self.fine_align_tol  = float(g('~fine_align_tol', self.FINE_ALIGN_TOL))
        self._reset_rate()

        #  Runtime state
        self.enabled = False            # a go_home() routine is running
        self.state   = self.ST_IDLE
        self._latest   = None           # most recent Measurement
        self._prev_ranges = None
        self._fresh_t  = 0.0
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

        # Drive-vs-motion overlay: what motor_control commands (any source:
        # joystick, fish, go_home) against how the robot really moves.
        self._drive_cmd = (0.0, 0.0)    # (forward, right) duty
        self._motion_hist = deque()     # (t, forward_m, right_m)
        rospy.Subscriber('motor_control/speed', TwistStamped,
                         self._on_drive_speed, queue_size=1)

        self.srv = rospy.Service(
            '/go_home/enable', SetBool, self._on_enable)
        # motor_control multiplies every command by its speed setting, which
        # the joystick changes; the tuned duty floors only hold at 1.0.
        self._set_speed = rospy.ServiceProxy('motor_control/set_speed', Float)
        # Go Home outranks a quarter turn: 0 quarter turns cancels one.
        self._rotate_srv = rospy.ServiceProxy('rotate/quarter_turns', Float)

        rospy.on_shutdown(self.cancel)

        self._publish_enabled()
        rospy.loginfo(
            "[GoHome] Node started.  max_lin_v=%.2f kp_lin=%.1f "
            "center_tol=%.3f m  align_tol=%.1f deg  align_sign=%+d  viz=%s",
            self.max_lin_v, self.kp_lin, self.center_tol, self.align_tol,
            self.align_sign, self._viz_on)

    #  Service / lifecycle

    def _publish_enabled(self):
        self.pub_enabled.publish(Bool(data=self.enabled))

    def _on_enable(self, req):
        """SetBool entry point.  Returns immediately; the run is threaded."""
        if bool(req.data):
            started = self.start()
            running = self._thread is not None and self._thread.is_alive()
            msg = ("GoHome started" if started
                   else "GoHome already running" if running
                   else "GoHome refused")
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

    def _cancel_rotation(self, wait_s=3.0):
        """Stop a running quarter turn so Go Home can take the motors."""
        rospy.loginfo("[GoHome] cancelling the rotation in progress")
        try:
            self._rotate_srv.wait_for_service(timeout=0.5)
            self._rotate_srv(0.0)
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn("[GoHome] could not cancel rotation: %s", e)
        end = time.monotonic() + wait_s
        while self._rotate_active and time.monotonic() < end:
            rospy.sleep(0.05)
        return not self._rotate_active

    def start(self):
        """Kick off go_home() on a worker thread.  False if already running."""
        if self._rotate_active and not self._cancel_rotation():
            rospy.logwarn("[GoHome] refused: the rotation did not stop")
            return False
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
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
        """Body yaw vs the walls -> (degrees, valid, confidence).

        Position independent, so the robot can square up before centring
        (the old symmetry probes needed it centred and read 0 at 45 deg).
        """
        heading, conf = geom.wall_heading_fit(ranges_m, r_min=self.RANGE_MIN_M,
                                          r_max=self.RANGE_MAX_M)
        if heading is None or conf < self.min_conf:
            return 0.0, False, conf
        return geom.fold_angle(heading - self.heading_offset), True, conf

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

    def _measure(self, ranges_m, fresh_t):
        walls = self._wall_distances(ranges_m)
        err_x, err_y, dist, ctr_valid = self._compute_center_error(walls)
        ang_err, ang_valid, ang_conf = self._compute_alignment_error(ranges_m)
        return Measurement(ranges_m, walls, err_x, err_y, dist, ctr_valid,
                           ang_err, ang_valid, ang_conf, fresh_t)

    def _on_lidar(self, msg):
        """Sensor sink only — never blocks and never drives the motors."""
        # ~0.5 ms a scan at 100 Hz: only sample when a phase or the overlay needs it.
        if not self._measuring and not self._viz_due():
            return

        ranges_m = np.array(msg.ranges, dtype=np.float64) / 1000.0
        if self._prev_ranges is None or not np.array_equal(ranges_m,
                                                           self._prev_ranges):
            self._fresh_t = time.monotonic()
        self._prev_ranges = ranges_m
        m = self._measure(ranges_m, self._fresh_t)
        self._latest = m
        self._render_viz(m, self._last_cmd, self._elapsed())

    def _collect(self, n, window_s):
        """Up to *n* distinct fresh measurements within *window_s* seconds."""
        out = []
        last_t = None
        end = time.monotonic() + window_s
        while time.monotonic() < end and len(out) < n:
            if self._cancel.is_set() or rospy.is_shutdown():
                break
            m = self._latest
            if m is not None and m.fresh_t != last_t:
                last_t = m.fresh_t
                out.append(m)
            rospy.sleep(0.03)
        return out

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

    def _guard_center(self, m):
        """Abort centring once the heading has walked off: cardinal distances
        taken along a crooked body point the robot at the wrong centre."""
        if m.ang_valid and abs(m.ang_err) > self.center_drift:
            rospy.logwarn("[GoHome] heading drifted to %+.1f deg while "
                          "centring -> re-align", m.ang_err)
            return Status.DRIFT
        return None

    def _sense_align(self, m):
        return m.ang_valid, abs(m.ang_err)

    def _align_duty(self, ang_err, magnitude):
        """angular.z that turns *against* a heading error of this sign."""
        return -self.align_sign * math.copysign(magnitude, ang_err)

    def _reset_rate(self):
        self._rate = 0.0                # filtered heading rate, deg/s
        self._rate_h = None
        self._rate_t = None
        self._best_abs = None           # smallest |error| this phase
        self._boost = 0.0               # floor raise after a stall
        self._prev_err = None
        self._stall_since = None

    def _check_sign(self, m):
        """Flip align_sign once if the turn goes the wrong way: the error
        grows past its best, or wraps through the +/-45 fold."""
        err = m.ang_err
        mag, side = abs(err), err > 0
        prev = self._prev_err
        self._prev_err = err
        if self._best_abs is None:
            self._best_abs = mag
            self._crossed = False
            return
        if prev is not None and (prev > 0) != side:
            if mag < 20.0 and abs(prev) < 20.0:
                self._crossed = True    # passed the target: the sign works
            elif mag > 30.0 and abs(prev) > 30.0:
                self._flip_sign("wrapped through 45 deg")
                return
        if mag < self._best_abs:
            self._best_abs = mag
        elif (not self._crossed
              and mag > self._best_abs + self.SIGN_PROBE_DEG * 3.0):
            self._flip_sign("error grew")

    def _flip_sign(self, why):
        if not self.sign_autodetect or self._sign_checked or self._crossed:
            return
        self.align_sign = -self.align_sign
        self._sign_checked = True
        self._best_abs = None
        self._rate = 0.0
        rospy.logwarn("[GoHome] heading moved the wrong way (%s): align_sign "
                      "flipped to %+d. Set ALIGN_SIGN in foma_params.yaml.",
                      why, self.align_sign)

    def _update_rate(self, m):
        """Heading rate over RATE_WINDOW_S: scan-to-scan (10 ms) noise
        reads as tens of deg/s and hides a stall."""
        if self._rate_h is None:
            self._rate_h, self._rate_t = m.ang_err, m.fresh_t
            return
        dt = m.fresh_t - self._rate_t
        if dt < self.RATE_WINDOW_S:
            return
        d = geom.unwrap_folded_delta(m.ang_err - self._rate_h)
        self._rate += self.RATE_ALPHA * (d / dt - self._rate)
        self._rate_h, self._rate_t = m.ang_err, m.fresh_t

    def _drive_align(self, m):
        """Smooth turn: break away at min_ang_v, creep once moving, and coast
        early enough that the ramp-down and lidar lag land on the target."""
        self._update_rate(m)
        self._check_sign(m)
        cmd = Twist()
        # The scan is a revolution old and the rate a window old: project
        # the error forward before deciding (as rotate_node does).
        err = m.ang_err + self._rate * self.LAG_S
        # deg/s the error is shrinking at (negative = growing).
        closing = -self._rate * math.copysign(1.0, err)
        if closing > 0 and abs(err) <= closing * self.stop_lead_s:
            return cmd                  # coast in
        if abs(err) <= self.align_tol and closing >= 0:
            return cmd                  # projected inside: let it settle
        moving = abs(self._rate) > self.MOVING_DPS
        now = time.monotonic()
        if moving:
            self._stall_since = None
        elif self._stall_since is None:
            self._stall_since = now
        elif now - self._stall_since > self.STALL_S:
            # Commanded but not turning: this floor needs more duty.
            self._boost = min(self._boost + self.STALL_STEP,
                              self.max_ang_v - self.min_ang_v)
            self._stall_since = now
        # The boost is for breaking away only: once turning, kinetic friction
        # is lower and a boosted floor just overshoots.
        floor = self.creep_ang_v if moving else self.min_ang_v + self._boost
        mag = min(max(self.kp_ang * abs(err), floor), self.max_ang_v)
        cmd.angular.z = self._align_duty(err, mag)
        return cmd

    def _run_phase(self, label, state, sense, drive, tol, settle_s, timeout,
                   guard=None):
        """Closed loop shared by every phase.  *guard(m)* may return a Status
        to end the phase early."""
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

                if guard is not None:
                    stop = guard(m)
                    if stop is not None:
                        self._hard_stop()
                        return PhaseResult(stop, residual, now - started)

                # Hysteresis: once settling, noise just above tol must not
                # restart the clock and send the robot hunting.
                inside = residual <= tol or (
                    inside_since is not None
                    and residual <= tol * self.release_scale)
                if inside:
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
                elif now - m.fresh_t > self.STALE_S:
                    # Never drive on a frozen buffer.
                    self._publish(Twist())
                    inside_since = None
                    if now - m.fresh_t > self.SCAN_WAIT_S:
                        self._hard_stop()
                        rospy.logwarn("[GoHome] %s: lidar scan stopped "
                                      "changing", label)
                        return PhaseResult(Status.NO_SCAN, residual,
                                           now - started)
                    rospy.logwarn_throttle(
                        1.0, "[GoHome] %s: scan stale, holding", label)
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
            self.timeout_s if timeout is None else timeout,
            guard=self._guard_center)

    def align_to_axes(self, tol=None, settle_s=None, timeout=None):
        """Rotate in place until the robot is square with the room axes."""
        self._reset_rate()
        return self._run_phase(
            'ALIGN', self.ST_ALIGN, self._sense_align, self._drive_align,
            self.align_tol if tol is None else tol,
            self.align_settle if settle_s is None else settle_s,
            self.timeout_s if timeout is None else timeout)

    def _reset_speed(self):
        try:
            self._set_speed.wait_for_service(timeout=0.5)
            self._set_speed(1.0)
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn("[GoHome] could not reset motor speed to 1.0 "
                          "(%s); duty floors may not hold", e)

    def _verify(self):
        """Median of several fresh scans -> (centred, square, dist, ang)."""
        ms = self._collect(self.VERIFY_SCANS, self.VERIFY_WINDOW_S)
        dists = [m.dist for m in ms if m.ctr_valid]
        angs = [m.ang_err for m in ms if m.ang_valid]
        dist = float(np.median(dists)) if dists else float('inf')
        ang = float(np.median(angs)) if angs else float('inf')
        return (dist < self.verify_center_tol, abs(ang) < self.verify_align_tol,
                dist, ang)

    def go_home(self, timeout=None, passes=None):
        """GOHOME: square up, drive to the centre, square up finely, verify.
        Aligning first works from any pose; cardinal distances read along a
        crooked body point at the wrong centre."""
        budget = self.timeout_s if timeout is None else timeout
        passes = self.passes if passes is None else passes
        deadline = time.monotonic() + budget

        # Does NOT clear _cancel: start() arms the run before the worker exists.
        self.enabled = True
        self._publish_enabled()
        self._latest = None             # never start on a previous run's scan
        self._begin_measuring()
        started = time.monotonic()
        result = PhaseResult(Status.UNSETTLED, float('inf'), 0.0)

        def out_of_time():
            return PhaseResult(Status.TIMEOUT, result.error,
                               time.monotonic() - started)

        try:
            self._reset_speed()

            for attempt in range(1, passes + 1):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return out_of_time()

                result = self.align_to_axes(timeout=remaining)
                if not result.ok:
                    return result

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return out_of_time()

                result = self.drive_to_center(timeout=remaining)
                if result.status == Status.DRIFT:
                    continue            # next pass re-aligns first
                if not result.ok:
                    return result

                # Square up in place once more. Usually already done: the
                # heading hold kept the robot square while it drove.
                fine = self.align_to_axes(
                    tol=self.fine_align_tol,
                    timeout=min(deadline - time.monotonic(),
                                self.FINE_ALIGN_TIMEOUT_S))
                if fine.status in (Status.CANCELLED, Status.NO_SCAN):
                    return fine

                # Confirm rather than assume — this is what guarantees the end state.
                self.state = self.ST_VERIFY
                centred, square, dist, ang = self._verify()
                if self._cancel.is_set():
                    return PhaseResult(Status.CANCELLED, dist,
                                       time.monotonic() - started)
                if centred and square:
                    rospy.loginfo(
                        "[GoHome] VERIFY PASS  dist=%.3f m  ang=%+.1f deg  "
                        "(pass %d/%d)", dist, ang, attempt, passes)
                    return PhaseResult(Status.SUCCESS, dist,
                                       time.monotonic() - started)

                rospy.loginfo(
                    "[GoHome] VERIFY FAIL  dist=%.3f m (%s)  ang=%+.1f deg (%s) "
                    "— repeating (pass %d/%d)",
                    dist, "ok" if centred else "off",
                    ang, "ok" if square else "off", attempt, passes)

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

    def _viz_polar_px(self, angle_deg, dist_m, lidar_frame=True):
        # The view is drawn in the robot-body frame: a lidar bearing is
        # turned by the mounting offset, so a square robot shows a square room.
        if lidar_frame:
            angle_deg -= self.heading_offset
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
        a = np.radians(idx.astype(np.float64) - self.heading_offset)
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
            # Along the body axes, like the rest of the (calibrated) view; the
            # raw beams are turned by the lidar mounting offset.
            end = self._viz_polar_px(angle, dist, lidar_frame=False)
            cv2.line(img, (self._viz_cx, self._viz_cy), end,
                     self.C_CARDINAL, 1, cv2.LINE_AA)
            cv2.circle(img, end, 4, self.C_CARDINAL, -1, cv2.LINE_AA)
            cv2.putText(img, "%.2f" % dist, (end[0] + 6, end[1] - 6),
                        self.FONT, 0.42, self.C_CARDINAL, 1, cv2.LINE_AA)

    def _viz_draw_axes(self, img, m):
        """The room axes as measured from the walls; square = on the grid."""
        if not m.ang_valid:
            return
        for base in (0.0, 90.0):
            a = base + m.ang_err
            # ang_err is already body-relative (offset removed).
            cv2.line(img, self._viz_polar_px(a, self._viz_view, False),
                     self._viz_polar_px(a + 180.0, self._viz_view, False),
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

    def _on_drive_speed(self, msg):
        # motor_control publishes linear.x = right (v), linear.y = forward (h).
        self._drive_cmd = (msg.twist.linear.y, msg.twist.linear.x)

    def _viz_track_motion(self, m):
        """Room position from the wall distances (robot axes = room axes
        while square), kept for TRAIL_S.  -> (velocity (fwd, right) m/s or
        None, history)."""
        now = time.monotonic()
        if None not in m.walls:
            d_fwd, d_bwd, d_left, d_right = m.walls
            self._motion_hist.append(((now), (d_bwd - d_fwd) / 2.0,
                                      (d_left - d_right) / 2.0))
        while self._motion_hist and now - self._motion_hist[0][0] > self.TRAIL_S:
            self._motion_hist.popleft()
        if len(self._motion_hist) < 2:
            return None
        t1, f1, r1 = self._motion_hist[-1]
        old = [p for p in self._motion_hist if t1 - p[0] >= self.MOTION_WIN_S]
        if not old:
            return None
        t0, f0, r0 = old[-1]
        return (f1 - f0) / (t1 - t0), (r1 - r0) / (t1 - t0)

    def _viz_arrow(self, img, fwd, right, colour, length_m):
        mag = math.hypot(fwd, right)
        if mag < 1e-9:
            return
        tip = self._viz_px(fwd / mag * length_m, -right / mag * length_m)
        cv2.arrowedLine(img, (self._viz_cx, self._viz_cy), tip, colour, 3,
                        cv2.LINE_AA, tipLength=0.2)

    def _viz_draw_motion(self, img, vel):
        """Trail of the last TRAIL_S, commanded vs measured direction.
        Returns (cmd_deg, motion_deg) with 0 = forward, + = right."""
        if self._motion_hist:
            _, fn, rn = self._motion_hist[-1]
            for _, f, r in self._motion_hist:
                cv2.circle(img, self._viz_px(f - fn, -(r - rn)), 2,
                           self.C_TRAIL, -1, cv2.LINE_AA)
        length = 0.35 * self._viz_view
        cf, cr = self._drive_cmd
        cmd_deg = mot_deg = None
        if math.hypot(cf, cr) > 0.02:
            cmd_deg = math.degrees(math.atan2(cr, cf))
            self._viz_arrow(img, cf, cr, self.C_DRIVE, length)
        if vel is not None and math.hypot(*vel) > 0.02:
            mot_deg = math.degrees(math.atan2(vel[1], vel[0]))
            self._viz_arrow(img, vel[0], vel[1], self.C_MOTION, length * 0.85)
        return cmd_deg, mot_deg

    def _viz_draw_legend(self, img):
        items = (("LIDAR returns", self.C_POINTS),
                 ("room model", self.C_ROOM),
                 ("wall probes", self.C_CARDINAL),
                 ("wall axes", self.C_PROBE),
                 ("room centre", self.C_CENTRE),
                 ("drive command", self.C_DRIVE),
                 ("actual motion", self.C_MOTION),
                 ("path (6 s)", self.C_TRAIL))
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
        self._viz_draw_axes(img, m)
        self._viz_draw_points(img, m.ranges)
        self._viz_draw_room(img, m.walls)
        if m.ctr_valid:
            self._viz_draw_centre(img, m.err_x, m.err_y)
        vel = self._viz_track_motion(m)
        cmd_deg, mot_deg = self._viz_draw_motion(img, vel)
        self._viz_draw_robot(img)

        if cmd_deg is not None and mot_deg is not None:
            diff = (mot_deg - cmd_deg + 180.0) % 360.0 - 180.0
            motion_txt = ("drive %+.1f deg  actual %+.1f deg  DIFF %+.1f deg  %.2f m/s"
                          % (cmd_deg, mot_deg, diff, math.hypot(*vel)))
        elif cmd_deg is not None:
            motion_txt = "drive %+.1f deg  actual --" % cmd_deg
        else:
            motion_txt = "drive --  (0 = forward, + = right)"

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
            ("heading err=%s  conf=%.2f  (tol %.1f deg)"
             % ("--" if not m.ang_valid else "%+.1f deg" % m.ang_err,
                m.ang_conf, self.align_tol), self.C_TEXT),
            ("cmd  x=%+.2f  y=%+.2f  w=%+.2f"
             % (cmd.linear.x, cmd.linear.y, cmd.angular.z), self.C_TEXT),
            (motion_txt, self.C_DRIVE),
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
