#!/usr/bin/env python3
import math
import threading
import time
import traceback
from collections import namedtuple

import numpy as np
import rospy
from abstract_node import AbstractNode
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String as StringMsg

from foma.srv import Float, FloatRequest, FloatResponse

# lidar_node's fill value; a bin still holding it was never measured.
SENTINEL_MM = 15000.0

MatchResult = namedtuple('MatchResult', 'shift_deg cost overlap valid reason')


class Status:
    SUCCESS   = 'success'
    BLOCKED   = 'blocked'      # obstacle inside the swept circle
    TIMEOUT   = 'timeout'
    CANCELLED = 'cancelled'
    NO_SCAN   = 'no_scan'      # no messages, or the buffer stopped changing
    LOST      = 'lost'         # tracking failed / drive train not responding
    UNSETTLED = 'unsettled'    # stopped, but outside tolerance
    REJECTED  = 'rejected'     # bad request, or another behaviour has the motors


TurnResult = namedtuple('TurnResult', 'status requested achieved trims detail')


# --- Scan matching: pure numpy, no ROS, so it can be tested off-robot. ------

def valid_mask(ranges_mm, min_mm=120.0, max_mm=12000.0):
    """Bins worth matching on."""
    r = np.asarray(ranges_mm, dtype=np.float64)
    return (np.isfinite(r) & (r >= min_mm) & (r <= max_mm)
            & (np.abs(r - SENTINEL_MM) > 1.0))


def shift_cost(live, live_m, ref, ref_m, shifts, clip_mm=300.0, min_overlap=100):
    """Truncated mean-absolute range difference for each integer shift."""
    s = np.asarray(shifts, dtype=np.int64)
    idx = (np.arange(360)[None, :] + s[:, None]) % 360      # (S, 360)
    both = ref_m[idx] & live_m[None, :]
    d = np.minimum(np.abs(ref[idx] - live[None, :]), clip_mm)
    n = both.sum(axis=1)
    cost = np.where(n >= min_overlap,
                    (d * both).sum(axis=1) / np.maximum(n, 1),
                    np.inf)
    return cost, n


def _frac_cost(live, live_m, ref, ref_m, s, clip_mm, min_overlap):
    """Cost at a fractional shift, interpolating the reference between bins."""
    base = np.arange(360) + s
    i0 = np.floor(base).astype(np.int64)
    frac = base - i0
    a, b = i0 % 360, (i0 + 1) % 360
    both = ref_m[a] & ref_m[b] & live_m
    n = int(both.sum())
    if n < min_overlap:
        return np.inf
    interp = ref[a] * (1.0 - frac) + ref[b] * frac
    d = np.minimum(np.abs(interp - live), clip_mm)
    return float((d * both).sum() / n)


def match_shift(live, ref, predicted_deg, window_deg=25.0, live_m=None,
                ref_m=None, clip_mm=300.0, min_overlap=100, cost_max_mm=400.0,
                reject_deg=15.0, refine=True, refine_step_deg=0.2,
                refine_span_deg=1.5):
    """Best circular shift from `ref` to `live`, in degrees, unwrapped."""
    live = np.asarray(live, dtype=np.float64)
    ref = np.asarray(ref, dtype=np.float64)
    if live_m is None:
        live_m = valid_mask(live)
    if ref_m is None:
        ref_m = valid_mask(ref)

    centre = int(round(predicted_deg))
    half = max(1, int(round(window_deg)))
    shifts = centre + np.arange(-half, half + 1)
    cost, n = shift_cost(live, live_m, ref, ref_m, shifts, clip_mm, min_overlap)

    k = int(np.argmin(cost))
    if not np.isfinite(cost[k]):
        return MatchResult(0.0, float('inf'), 0, False, 'no_overlap')
    # Edge minimum means the true one is outside the window: fail, don't clip.
    if k == 0 or k == len(shifts) - 1:
        return MatchResult(float(shifts[k]), float(cost[k]), int(n[k]), False,
                           'boundary')
    if cost[k] > cost_max_mm:
        return MatchResult(float(shifts[k]), float(cost[k]), int(n[k]), False,
                           'poor_fit')

    best = float(shifts[k])
    if refine:
        # Parabolic fit is biased on a piecewise-linear cost; use the fine grid.
        fine = best + np.arange(-refine_span_deg, refine_span_deg + 1e-9,
                                refine_step_deg)
        fc = np.array([_frac_cost(live, live_m, ref, ref_m, f, clip_mm,
                                  min_overlap) for f in fine])
        j = int(np.argmin(fc))
        best = float(fine[j])
        if 0 < j < len(fine) - 1:
            den = fc[j - 1] - 2.0 * fc[j] + fc[j + 1]
            if abs(den) > 1e-9:
                delta = 0.5 * (fc[j - 1] - fc[j + 1]) / den
                best += float(np.clip(delta, -0.5, 0.5)) * refine_step_deg

    if abs(best - predicted_deg) > reject_deg:
        return MatchResult(best, float(cost[k]), int(n[k]), False, 'outlier')
    return MatchResult(best, float(cost[k]), int(n[k]), True, 'ok')


def changed_bins(a_mm, b_mm, eps_mm=1.0):
    """How many bins differ. This, not message arrival, is the proof of life."""
    if a_mm is None or b_mm is None:
        return 0
    return int(np.count_nonzero(np.abs(np.asarray(a_mm, dtype=np.float64)
                                       - np.asarray(b_mm, dtype=np.float64))
                                > eps_mm))


def clearance_violations(ranges_mm, mask, radius_mm):
    """Bins with a return inside `radius_mm`. The full circle is swept, so no"""
    r = np.asarray(ranges_mm, dtype=np.float64)
    return np.flatnonzero(mask & (r < radius_mm))


# --- Motion profile: pure python, no ROS. ----------------------------------

class Runaway(Exception):
    """Error grew instead of shrinking: wrong sign, or the robot is stuck."""


class RotateController:
    """P-control on one signed unwrapped error, with a predictive brake."""

    def __init__(self, kp_duty_per_deg, min_ang_v, max_ang_v, stop_lead_s,
                 stop_lead_deg, runaway_deg):
        self.kp = kp_duty_per_deg
        self.min_v = min_ang_v
        self.max_v = max_ang_v
        self.stop_lead_s = stop_lead_s
        self.stop_lead_deg = stop_lead_deg
        self.runaway_deg = runaway_deg
        self.target = 0.0
        self._best = float('inf')

    def reset(self, target_deg):
        self.target = float(target_deg)
        self._best = float('inf')

    def step(self, theta_deg, rate_dps):
        """Duty to command, or None to cut power and coast into the target."""
        e = self.target - theta_deg
        self._best = min(self._best, abs(e))
        if abs(e) > self._best + self.runaway_deg:
            raise Runaway("error grew from %.1f to %.1f deg" % (self._best, abs(e)))

        # Brake early: 0.45 duty needs 0.30 s to ramp down. Lead scales with rate.
        lead = abs(rate_dps) * self.stop_lead_s + self.stop_lead_deg
        if abs(e) <= lead:
            return None
        v = min(self.max_v, self.kp * abs(e))
        v = max(v, self.min_v)              # beat static friction
        return math.copysign(v, e)


# --- ROS node --------------------------------------------------------------

class RotateNode(AbstractNode):

    def __init__(self):
        super().__init__('rotate', 'Rotate')
        g = rospy.get_param

        # request limits
        self.max_quarter_turns = float(g('~max_quarter_turns', 8))
        self.snap_to_quarter   = bool(g('~snap_to_quarter', True))

        # geometry / safety
        self.robot_radius_mm     = float(g('~robot_radius_mm', 407.0))
        self.clearance_margin_mm = float(g('~clearance_margin_mm', 80.0))
        self.run_margin_mm       = float(g('~run_margin_mm', 30.0))
        self.max_violating_bins  = int(g('~max_violating_bins', 2))
        self.min_valid_bins      = int(g('~min_valid_bins', 200))
        self.require_clearance   = bool(g('~require_clearance', True))
        self.run_violation_cycles = int(g('~run_violation_cycles', 3))

        # lidar model
        self.lidar_rev_s        = float(g('~lidar_rev_s', 0.18))
        self.measure_rev_period = bool(g('~measure_rev_period', True))
        self.scan_timeout_s     = float(g('~scan_timeout_s', 0.5))
        self.liveness_window_s  = float(g('~liveness_window_s', 0.7))
        self.min_changed_bins   = int(g('~min_changed_bins', 10))

        # matching
        self.window_deg      = float(g('~window_deg', 25.0))
        self.reject_deg      = float(g('~reject_deg', 15.0))
        self.clip_mm         = float(g('~clip_mm', 300.0))
        self.min_overlap     = int(g('~min_overlap', 100))
        self.cost_max_mm     = float(g('~cost_max_mm', 400.0))
        self.max_bad         = int(g('~max_bad_measurements', 15))
        self.refine_step_deg = float(g('~refine_step_deg', 0.2))
        self.rate_alpha      = float(g('~rate_alpha', 0.25))

        # motion
        self.control_hz    = float(g('~control_hz', 50.0))
        self.match_hz      = float(g('~match_hz', 25.0))
        self.max_ang_v     = float(g('~max_ang_v', 0.45))
        self.min_ang_v     = float(g('~min_ang_v', 0.20))
        self.kp_duty       = float(g('~kp_duty_per_deg', 0.015))
        self.stop_lead_s   = float(g('~stop_lead_s', 0.15))
        self.stop_lead_deg = float(g('~stop_lead_deg', 1.0))
        self.runaway_deg   = float(g('~runaway_deg', 20.0))

        # Sign conventions depend on wiring, hence the autodetect.
        self.angle_sign     = int(g('~angle_sign', 1))
        self.rotate_sign    = int(g('~rotate_sign', -1))
        self.sign_autodetect = bool(g('~sign_autodetect', True))
        self.sign_probe_deg = float(g('~sign_probe_deg', 4.0))
        self.sign_probe_s   = float(g('~sign_probe_s', 2.0))
        self._sign_checked  = False

        # finish
        self.settle_s      = float(g('~settle_s', 0.8))
        self.still_tol_deg = float(g('~still_tol_deg', 0.4))
        self.tol_deg       = float(g('~tol_deg', 2.5))
        self.trim_min_deg  = float(g('~trim_min_deg', 1.5))
        self.max_trims     = int(g('~max_trims', 2))
        self.max_trim_deg  = float(g('~max_trim_deg', 12.0))
        self.trim_timeout_s = float(g('~trim_timeout_s', 6.0))

        # budget
        self.base_timeout_s   = float(g('~base_timeout_s', 4.0))
        self.min_rate_dps     = float(g('~min_expected_rate_dps', 15.0))
        self.max_timeout_s    = float(g('~max_timeout_s', 120.0))
        self.stop_publish_s   = float(g('~stop_publish_s', 0.4))
        self.publish_progress = bool(g('~publish_progress', True))

        # runtime state
        self.active = False
        self._scan = None
        self._scan_time = 0.0
        self._ref = None
        self._ref_mask = None
        self._cancel = threading.Event()
        self._lock = threading.Lock()
        self._thread = None
        self._go_home_active = False
        self._progress = 0.0

        self._ctrl = RotateController(self.kp_duty, self.min_ang_v,
                                      self.max_ang_v, self.stop_lead_s,
                                      self.stop_lead_deg, self.runaway_deg)

        # ROS I/O
        self.cmd_pub = rospy.Publisher('motor_control/twist', Twist,
                                       queue_size=1, tcp_nodelay=True)
        self.pub_active = rospy.Publisher('rotate/active', Bool, queue_size=1,
                                          latch=True)
        self.pub_result = rospy.Publisher('rotate/result', StringMsg,
                                          queue_size=1, latch=True)
        self.pub_prog = rospy.Publisher('rotate/progress', Float32,
                                        queue_size=1)

        rospy.Subscriber('lidar/scans', LaserScan, self._on_lidar,
                         queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('go_home/enabled', Bool, self._on_go_home,
                         queue_size=1)
        # Channels we never use: traffic means the joystick took over.
        rospy.Subscriber('motor_control/rotate', Float32, self._on_foreign_rot,
                         queue_size=1)
        rospy.Subscriber('motor_control/vector', Vector3, self._on_foreign_vec,
                         queue_size=1)

        self.srv = rospy.Service('rotate/quarter_turns', Float, self._on_request)

        rospy.on_shutdown(self._on_shutdown)
        self._publish_active()
        self._publish_result("idle")
        rospy.loginfo("[Rotate] ready. tol=%.1f deg  max=%.1f quarter turns  "
                      "rotate_sign=%+d (autodetect=%s)",
                      self.tol_deg, self.max_quarter_turns, self.rotate_sign,
                      self.sign_autodetect)

    # -- subscriptions ------------------------------------------------------

    def _on_lidar(self, msg):
        self._scan = np.asarray(msg.ranges, dtype=np.float64)
        self._scan_time = time.monotonic()

    def _on_go_home(self, msg):
        self._go_home_active = bool(msg.data)

    def _on_foreign_rot(self, msg):
        if self.active and abs(float(msg.data)) > 0.05:
            rospy.logwarn("[Rotate] motor_control/rotate traffic - cancelling")
            self._cancel.set()

    def _on_foreign_vec(self, msg):
        if self.active and math.hypot(msg.x, msg.y) > 0.05:
            rospy.logwarn("[Rotate] motor_control/vector traffic - cancelling")
            self._cancel.set()

    # -- service ------------------------------------------------------------

    def _on_request(self, req):
        """Accept or reject.  Returns immediately -- the turn runs threaded."""
        data = float(req.data)

        if data == 0.0:
            self.cancel()
            return FloatResponse(result=True)

        if not math.isfinite(data) or abs(data) > self.max_quarter_turns:
            self._publish_result("rejected: %s quarter turns out of range "
                                 "(max %.0f)" % (data, self.max_quarter_turns))
            return FloatResponse(result=False)

        if self._go_home_active:
            self._publish_result("rejected: go_home is running")
            return FloatResponse(result=False)

        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                self._publish_result("rejected: already rotating")
                return FloatResponse(result=False)
            quarters = round(data * 4.0) / 4.0 if self.snap_to_quarter else data
            target = self.angle_sign * quarters * 90.0
            self._cancel.clear()
            self._thread = threading.Thread(target=self._routine, args=(target,),
                                            name='rotate', daemon=True)
            self._thread.start()
        rospy.loginfo("[Rotate] accepted %+.2f quarter turns (%+.1f deg)",
                      quarters, target)
        return FloatResponse(result=True)

    def cancel(self, join_timeout=1.0):
        self._cancel.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=join_timeout)
        self._hard_stop()

    def _on_shutdown(self):
        self._cancel.set()
        self.active = False
        try:
            self._publish_active()      # never leave the GUI toggle stuck on
        except Exception:
            pass
        self._hard_stop()

    # -- publishing ---------------------------------------------------------

    def _publish_active(self):
        self.pub_active.publish(Bool(data=self.active))

    def _publish_result(self, text):
        rospy.loginfo("[Rotate] %s", text)
        try:
            self.pub_result.publish(StringMsg(data=text))
        except Exception:
            pass

    def _publish_duty(self, duty):
        """Command a rotation duty. linear.{x,y} stay zero because"""
        cmd = Twist()
        cmd.angular.z = float(duty)
        self.cmd_pub.publish(cmd)

    def _hard_stop(self):
        """Zero the motors, then go quiet so motor_control's 1 s watchdog arms"""
        end = time.monotonic() + self.stop_publish_s
        period = 1.0 / self.control_hz
        while time.monotonic() < end:
            self._publish_duty(0.0)
            time.sleep(period)
        self._publish_duty(0.0)

    # -- helpers ------------------------------------------------------------

    def _sleep(self, dt):
        if dt > 0:
            time.sleep(dt)

    def _scan_ok(self):
        return (self._scan is not None
                and time.monotonic() - self._scan_time <= self.scan_timeout_s)

    def _wait_for_scan(self, timeout=2.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self._cancel.is_set():
                return False
            if self._scan_ok():
                return True
            self._sleep(0.02)
        return False

    def _clearance_ok(self, margin_mm):
        scan = self._scan
        if scan is None:
            return False, "no scan"
        mask = valid_mask(scan)
        if int(mask.sum()) < self.min_valid_bins:
            return False, "only %d valid bins" % int(mask.sum())
        radius = self.robot_radius_mm + margin_mm
        bad = clearance_violations(scan, mask, radius)
        if len(bad) > self.max_violating_bins:
            i = int(bad[np.argmin(scan[bad])])
            return False, "%.0f mm at %d deg (need %.0f)" % (scan[i], i, radius)
        return True, ""

    def _capture_reference(self):
        """Snapshot the stationary scan and time one buffer refresh."""
        self._ref = np.array(self._scan, dtype=np.float64)
        self._ref_mask = valid_mask(self._ref)

        if not self.measure_rev_period:
            return True, self.lidar_rev_s

        mark = np.array(self._scan, dtype=np.float64)
        start = time.monotonic()
        deadline = start + max(1.0, 6.0 * self.lidar_rev_s)
        target = int(0.9 * 360)
        while time.monotonic() < deadline:
            if self._cancel.is_set():
                return False, self.lidar_rev_s
            if changed_bins(self._scan, mark) >= target:
                return True, max(0.05, time.monotonic() - start)
            self._sleep(0.005)
        return False, self.lidar_rev_s

    # -- the maneuver -------------------------------------------------------

    def _routine(self, target_deg):
        self.active = True
        self._publish_active()
        result = TurnResult(Status.LOST, target_deg, 0.0, 0, "unknown")
        try:
            result = self._rotate(target_deg)
        except Exception:
            rospy.logerr("[Rotate] unhandled: %s", traceback.format_exc())
            result = TurnResult(Status.LOST, target_deg, self._progress, 0,
                                "internal error")
        finally:
            # Whatever happened, the motors stop and the GUI toggle resets.
            self._hard_stop()
            self.active = False
            self._publish_active()

        if result.status == Status.SUCCESS:
            self._publish_result(
                "success: %+.1f deg requested, %+.1f achieved, %d trim(s)"
                % (result.requested, result.achieved, result.trims))
        else:
            self._publish_result(
                "%s: %+.1f deg requested, %+.1f achieved -- %s"
                % (result.status, result.requested, result.achieved,
                   result.detail))

    def _rotate(self, target_deg):
        budget = min(self.max_timeout_s,
                     self.base_timeout_s + abs(target_deg) / self.min_rate_dps
                     + self.max_trims * self.trim_timeout_s + 2 * self.settle_s)
        deadline = time.monotonic() + budget
        self._progress = 0.0

        if self._cancel.is_set():
            return TurnResult(Status.CANCELLED, target_deg, 0.0, 0,
                              "cancelled before start")

        if not self._wait_for_scan():
            return TurnResult(Status.NO_SCAN, target_deg, 0.0, 0,
                              "no lidar/scans")

        if self.require_clearance:
            ok, why = self._clearance_ok(self.clearance_margin_mm)
            if not ok:
                # Refused before the motors ever move.
                return TurnResult(Status.BLOCKED, target_deg, 0.0, 0, why)

        alive, rev_s = self._capture_reference()
        if not alive:
            if self._cancel.is_set():
                return TurnResult(Status.CANCELLED, target_deg, 0.0, 0,
                                  "cancelled during reference capture")
            return TurnResult(Status.NO_SCAN, target_deg, 0.0, 0,
                              "lidar buffer never refreshed (sensor hung?)")
        lag_s = 0.5 * rev_s + 0.03
        rospy.loginfo("[Rotate] T_rev=%.3f s -> lag %.3f s", rev_s, lag_s)

        if self.sign_autodetect and not self._sign_checked:
            st, why = self._probe_sign(math.copysign(1.0, target_deg), deadline)
            if st is not None:
                return TurnResult(st, target_deg, self._progress, 0, why)

        st, theta, why = self._sweep(target_deg, deadline, lag_s)
        if st is not None:
            return TurnResult(st, target_deg, theta, 0, why)

        # Judge the result stationary-against-stationary: no shear either side.
        trims = 0
        while True:
            st, theta, why = self._settle(deadline, rev_s)
            if st is not None:
                return TurnResult(st, target_deg, theta, trims, why)

            residual = target_deg - theta
            if abs(residual) <= self.tol_deg:
                return TurnResult(Status.SUCCESS, target_deg, theta, trims, "")

            if (trims >= self.max_trims or abs(residual) < self.trim_min_deg
                    or abs(residual) > self.max_trim_deg):
                # Too small to resolve, or too large to correct blind.
                return TurnResult(Status.UNSETTLED, target_deg, theta, trims,
                                  "residual %+.1f deg" % residual)

            trims += 1
            rospy.loginfo("[Rotate] trim %d/%d for %+.1f deg", trims,
                          self.max_trims, residual)
            trim_deadline = min(deadline, time.monotonic() + self.trim_timeout_s)
            st, theta, why = self._sweep(target_deg, trim_deadline, lag_s,
                                         start_theta=theta)
            if st is not None and st != Status.TIMEOUT:
                return TurnResult(st, target_deg, theta, trims, why)

    def _probe_sign(self, direction, deadline):
        """Nudge briefly and check the shift moves the expected way."""
        period = 1.0 / self.control_hz
        end = min(deadline, time.monotonic() + self.sign_probe_s)
        duty = self.min_ang_v * direction
        moved = 0.0
        while time.monotonic() < end:
            if self._cancel.is_set():
                return Status.CANCELLED, "cancelled during sign probe"
            if not self._scan_ok():
                return Status.NO_SCAN, "lidar went quiet during sign probe"
            self._publish_duty(self.rotate_sign * duty)
            m = match_shift(self._scan, self._ref, moved, self.window_deg,
                            ref_m=self._ref_mask, clip_mm=self.clip_mm,
                            min_overlap=self.min_overlap,
                            cost_max_mm=self.cost_max_mm,
                            reject_deg=self.reject_deg,
                            refine_step_deg=self.refine_step_deg)
            if m.valid:
                moved = m.shift_deg
                if abs(moved) >= self.sign_probe_deg:
                    break
            self._sleep(period)

        self._publish_duty(0.0)
        self._sign_checked = True

        if abs(moved) < self.sign_probe_deg:
            # A hung LIDAR is indistinguishable from a stalled drive train here.
            return Status.LOST, ("no movement during sign probe (%.1f deg) -- "
                                 "drive train stalled or lidar frozen?" % moved)
        if math.copysign(1.0, moved) != math.copysign(1.0, direction):
            self.rotate_sign = -self.rotate_sign
            rospy.logerr("[Rotate] rotation sign was inverted -- flipped "
                         "rotate_sign to %+d. Set this in foma_params.yaml.",
                         self.rotate_sign)
        self._settle_quiet(0.4)         # let the probe motion bleed off
        return None, ""

    def _settle_quiet(self, seconds):
        end = time.monotonic() + seconds
        period = 1.0 / self.control_hz
        while time.monotonic() < end and not self._cancel.is_set():
            self._publish_duty(0.0)
            self._sleep(period)

    def _sweep(self, target_deg, deadline, lag_s, start_theta=0.0):
        """Drive to the target as one continuous motion."""
        period = 1.0 / self.control_hz
        match_period = 1.0 / self.match_hz
        self._ctrl.reset(target_deg)

        pred = float(start_theta)       # tracked in MEASUREMENT space (lagged)

        # The only loop that commands motion, so it zeroes on every exit.
        try:
            return self._drive(deadline, lag_s, pred, period, match_period)
        finally:
            self._publish_duty(0.0)

    def _drive(self, deadline, lag_s, pred, period, match_period):
        """The sweep's inner loop.  See _sweep for the contract."""
        rate = 0.0
        last_meas = last_meas_t = None
        next_match = 0.0
        bad = violations = 0
        live_mark = None
        live_mark_t = time.monotonic()
        moving_since = None

        while True:
            now = time.monotonic()

            if self._cancel.is_set():
                return Status.CANCELLED, pred, "cancelled"
            if now > deadline:
                return Status.TIMEOUT, pred, "budget exhausted"
            if not self._scan_ok():
                return Status.NO_SCAN, pred, "lidar/scans stopped"

            scan = self._scan

            # A hung sensor keeps publishing: check contents, and only while driving.
            if moving_since is not None and now - moving_since > 0.5:
                if changed_bins(scan, live_mark) >= self.min_changed_bins:
                    live_mark, live_mark_t = np.array(scan), now
                elif now - live_mark_t > self.liveness_window_s:
                    return (Status.NO_SCAN, pred,
                            "lidar buffer frozen while moving (sensor hung?)")
            elif live_mark is None:
                live_mark, live_mark_t = np.array(scan), now

            if now >= next_match:
                m = match_shift(scan, self._ref, pred, self.window_deg,
                                ref_m=self._ref_mask, clip_mm=self.clip_mm,
                                min_overlap=self.min_overlap,
                                cost_max_mm=self.cost_max_mm,
                                reject_deg=self.reject_deg,
                                refine_step_deg=self.refine_step_deg)
                if m.valid:
                    if last_meas is not None and now - last_meas_t > 1e-3:
                        raw = (m.shift_deg - last_meas) / (now - last_meas_t)
                        rate += self.rate_alpha * (raw - rate)
                    last_meas, last_meas_t = m.shift_deg, now
                    pred = m.shift_deg
                    bad = 0
                else:
                    bad += 1
                    if bad > self.max_bad:
                        return (Status.LOST, pred,
                                "tracking lost (%s)" % m.reason)
                next_match = now + match_period
            else:
                pred += rate * period       # stay aligned with the lagged
                                            # measurement stream

            if self.require_clearance:
                mask = valid_mask(scan)
                near = clearance_violations(
                    scan, mask, self.robot_radius_mm + self.run_margin_mm)
                # A moving buffer is noisy: require consecutive violations.
                violations = violations + 1 if len(near) > self.max_violating_bins else 0
                if violations >= self.run_violation_cycles:
                    return Status.BLOCKED, pred, "obstacle entered the swept circle"

            self._progress = pred
            if self.publish_progress:
                self.pub_prog.publish(Float32(data=float(pred)))

            # The reading is the mean heading over the last revolution; lead-compensate.
            theta = pred + rate * lag_s
            try:
                duty = self._ctrl.step(theta, rate)
            except Runaway as e:
                return Status.LOST, pred, str(e)

            if duty is None:
                self._publish_duty(0.0)
                return None, pred, ""

            self._publish_duty(self.rotate_sign * duty)
            if moving_since is None:
                moving_since = now
            self._sleep(period)

    def _settle(self, deadline, rev_s):
        """Stop, wait for the buffer to refresh, and prove the robot is still."""
        period = 1.0 / self.control_hz
        pred = self._progress
        end = time.monotonic() + self.settle_s
        # Keep matching and publishing zeros so motor_control ramps down.
        anchors = []
        next_anchor = end
        hard_end = time.monotonic() + self.settle_s + 2.0

        while True:
            now = time.monotonic()
            if self._cancel.is_set():
                return Status.CANCELLED, pred, "cancelled"
            if now > deadline or now > hard_end:
                return Status.UNSETTLED, pred, "never came to rest"
            if not self._scan_ok():
                return Status.NO_SCAN, pred, "lidar/scans stopped"

            self._publish_duty(0.0)

            m = match_shift(self._scan, self._ref, pred, self.window_deg,
                            ref_m=self._ref_mask, clip_mm=self.clip_mm,
                            min_overlap=self.min_overlap,
                            cost_max_mm=self.cost_max_mm,
                            reject_deg=self.reject_deg,
                            refine_step_deg=self.refine_step_deg)
            if m.valid:
                pred = m.shift_deg
                self._progress = pred
                if now >= next_anchor:
                    anchors.append(pred)
                    next_anchor = now + rev_s
                    # Two readings a revolution apart that agree mean a stationary buffer.
                    if len(anchors) >= 2:
                        if abs(anchors[-1] - anchors[-2]) <= self.still_tol_deg:
                            return None, pred, ""
                        anchors = anchors[-1:]
            self._sleep(period)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("rotate_node")
    try:
        node = RotateNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
