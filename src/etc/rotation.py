"""In-place quarter turns closed on the LIDAR.

Runs inside motor_control_node: the Rotator writes its duty straight into
the motor loop through `command`, so there is one owner of the motors and
no network round trip in the loop. Interface kept from the old rotate_node:
service rotate/quarter_turns, topics rotate/active, rotate/result,
rotate/progress.
"""
import math
import threading
import time
import traceback
from collections import namedtuple

import numpy as np
import rospy
from std_msgs.msg import Bool, Float32, String as StringMsg

from foma.srv import Float, FloatRequest, FloatResponse
from etc import scan_geometry as geom

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
        self.boost = 0.0                # added to min_v after a stall

    def reset(self, target_deg):
        self.target = float(target_deg)
        self._best = float('inf')

    def bump(self, step):
        """Commanded but not turning: static friction beat min_v on this
        floor.  Raise the floor for the rest of the request."""
        self.boost = min(self.boost + step, self.max_v - self.min_v)

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
        v = max(v, self.min_v + self.boost)  # beat static friction
        return math.copysign(v, e)


# --- The maneuver ----------------------------------------------------------

class Rotator:

    def __init__(self, command, param_ns='~rotate/'):
        """`command(duty)` sets the rotation duty of the motor loop. Called
        at control_hz while turning, so the loop's 1 s watchdog stays armed."""
        self._command = command
        g = lambda k, d: rospy.get_param(param_ns + k.lstrip('~'), d)

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
        self.trim_settle_s = float(g('~trim_settle_s', 0.4))
        self.still_tol_deg = float(g('~still_tol_deg', 0.4))
        self.tol_deg       = float(g('~tol_deg', 2.5))
        self.trim_min_deg  = float(g('~trim_min_deg', 1.5))
        self.max_trims     = int(g('~max_trims', 2))
        self.max_trim_deg  = float(g('~max_trim_deg', 12.0))
        self.trim_timeout_s = float(g('~trim_timeout_s', 6.0))
        # Stall recovery: no turn for stall_s while commanded -> floor + step.
        self.stall_s       = float(g('~stall_s', 0.4))
        self.stall_dps     = float(g('~stall_dps', 1.5))
        self.stall_step    = float(g('~stall_step', 0.04))
        # Finish square to the room: quarter turns end on the wall heading,
        # not on the relative angle, so a crooked start or a short turn
        # still ends aligned.
        self.snap_to_axes  = bool(g('~snap_to_axes', True))
        self.snap_tol_deg  = float(g('~snap_tol_deg', 1.0))
        self.max_snap_deg  = float(g('~max_snap_deg', 20.0))
        self.max_snap_trims = int(g('~max_snap_trims', 5))
        self.heading_offset = float(g('/HEADING_OFFSET_DEG', 0.0))

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
        self._still_scans = []

        self._ctrl = RotateController(self.kp_duty, self.min_ang_v,
                                      self.max_ang_v, self.stop_lead_s,
                                      self.stop_lead_deg, self.runaway_deg)

        # ROS I/O. Scans come from the owner through on_scan.
        self.pub_active = rospy.Publisher('rotate/active', Bool, queue_size=1,
                                          latch=True)
        self.pub_result = rospy.Publisher('rotate/result', StringMsg,
                                          queue_size=1, latch=True)
        self.pub_prog = rospy.Publisher('rotate/progress', Float32,
                                        queue_size=1)

        rospy.Subscriber('go_home/enabled', Bool, self._on_go_home,
                         queue_size=1)

        self.srv = rospy.Service('rotate/quarter_turns', Float, self._on_request)

        self._publish_active()
        self._publish_result("idle")
        rospy.loginfo("[Rotate] ready. tol=%.1f deg  max=%.1f quarter turns  "
                      "rotate_sign=%+d (autodetect=%s)",
                      self.tol_deg, self.max_quarter_turns, self.rotate_sign,
                      self.sign_autodetect)

    # -- inputs from the owner ----------------------------------------------

    def on_scan(self, ranges):
        self._scan = np.asarray(ranges, dtype=np.float64)
        self._scan_time = time.monotonic()

    def _on_go_home(self, msg):
        self._go_home_active = bool(msg.data)

    def request_cancel(self, why):
        """Another source took the motors. Returns at once, the turn thread
        stops on its next tick."""
        if self.active and not self._cancel.is_set():
            rospy.logwarn("[Rotate] %s - cancelling", why)
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

    def shutdown(self):
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
        """Command a rotation duty. Translation is zeroed by the owner."""
        self._command(float(duty))

    def _hard_stop(self):
        """Zero the motors for a while so the loop ramps down, then go quiet"""
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
                     + max(self.max_trims, self.max_snap_trims) * self.trim_timeout_s
                     + 2 * self.settle_s)
        deadline = time.monotonic() + budget
        self._progress = 0.0
        self._ctrl.boost = 0.0

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

        t0 = time.monotonic()
        st, theta, why = self._sweep(target_deg, deadline, lag_s)
        if st is not None:
            return TurnResult(st, target_deg, theta, 0, why)
        rospy.loginfo("[Rotate] sweep %.1f s, at %+.1f deg", time.monotonic() - t0, theta)

        # Judge the result stationary-against-stationary: no shear either side.
        trims = 0
        quarter = abs(target_deg / 90.0 - round(target_deg / 90.0)) < 1e-6
        while True:
            t_settle = time.monotonic()
            # A trim runs at low duty and ramps down fast: a shorter wait.
            # Stillness is still proven by the anchors either way.
            st, theta, why = self._settle(
                deadline, rev_s, self.trim_settle_s if trims else self.settle_s)
            if st is not None:
                return TurnResult(st, target_deg, theta, trims, why)
            t_walls = time.monotonic()

            goal, tol, limit, snapped = target_deg, self.tol_deg, self.max_trims, False
            if self.snap_to_axes and quarter:
                h = self._axis_heading(self._still_scans)
                if h is None:
                    rospy.logwarn("[Rotate] walls not trusted - finishing on "
                                  "the relative angle, may end off axis")
                else:
                    goal = self._square_goal(theta, h, target_deg)
                    tol, limit, snapped = self.snap_tol_deg, self.max_snap_trims, True
                    if abs(goal - target_deg) > self.max_snap_deg:
                        rospy.logwarn("[Rotate] squaring up %+.1f deg away from "
                                      "the request (crooked start or tracking drift)",
                                      goal - target_deg)

            residual = goal - theta
            rospy.loginfo("[Rotate] settle %.1f s, walls %.1f s, residual %+.1f deg",
                          t_walls - t_settle, time.monotonic() - t_walls, residual)
            if abs(residual) <= tol:
                return TurnResult(Status.SUCCESS, target_deg, theta, trims, "")

            # The wall heading is absolute, so a large snap residual is still
            # safe to correct. Only a relative finish is capped as blind.
            if (trims >= limit
                    or abs(residual) < min(self.trim_min_deg, tol)
                    or (not snapped and abs(residual) > self.max_trim_deg)):
                return TurnResult(Status.UNSETTLED, target_deg, theta, trims,
                                  "residual %+.1f deg" % residual)

            trims += 1
            rospy.loginfo("[Rotate] trim %d/%d for %+.1f deg", trims,
                          limit, residual)
            trim_deadline = min(deadline, time.monotonic() + self.trim_timeout_s)
            t_trim = time.monotonic()
            st, theta, why = self._sweep(goal, trim_deadline, lag_s,
                                         start_theta=theta)
            rospy.loginfo("[Rotate] trim sweep %.1f s, at %+.1f deg",
                          time.monotonic() - t_trim, theta)
            if st is not None and st != Status.TIMEOUT:
                return TurnResult(st, target_deg, theta, trims, why)

    @staticmethod
    def _square_goal(theta, h, target_deg):
        """Shift that leaves the robot square to the room, nearest the request.
        The wall heading moves by minus the scan shift, so at shift theta with
        heading h the squared-up shifts are theta + h + k * 90."""
        square = theta + h
        return square + 90.0 * round((target_deg - square) / 90.0)

    def _probe_sign(self, direction, deadline):
        """Nudge briefly and check the shift moves the expected way."""
        period = 1.0 / self.control_hz
        start = time.monotonic()
        end = min(deadline, start + self.sign_probe_s)
        moved = 0.0
        while time.monotonic() < end:
            # Ramp from min_ang_v: a fixed floor-level nudge failed to break
            # static friction and rejected the first request of every session.
            level = self.min_ang_v + self.stall_step * int(
                (time.monotonic() - start) / self.stall_s)
            duty = min(level, self.max_ang_v) * direction
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

        if abs(moved) < self.sign_probe_deg:
            # A hung LIDAR is indistinguishable from a stalled drive train here.
            return Status.LOST, ("no movement during sign probe (%.1f deg) -- "
                                 "drive train stalled or lidar frozen?" % moved)
        if math.copysign(1.0, moved) != math.copysign(1.0, direction):
            self.rotate_sign = -self.rotate_sign
            rospy.logerr("[Rotate] rotation sign was inverted -- flipped "
                         "rotate_sign to %+d. Set this in foma_params.yaml.",
                         self.rotate_sign)
        self._sign_checked = True       # only once it actually proved the sign
        self._settle_quiet(0.4)         # let the probe motion bleed off
        return None, ""

    def _wall_heading(self, scan):
        """Wall heading (deg, 0 = square to the room) of one scan, or None."""
        h, conf = geom.wall_heading_fit(np.asarray(scan) / 1000.0)
        if h is None or conf < 0.5:
            return None
        return geom.fold_angle(h - self.heading_offset)

    def _axis_heading(self, still_scans=None, samples=3, window_s=0.6):
        """Median wall heading (deg, 0 = square to the room) of a few
        distinct stationary scans, or None when the walls are not trusted.
        Scans the settle already proved still are used first, which saves
        waiting for fresh ones."""
        if still_scans:
            vals = [v for v in map(self._wall_heading, still_scans) if v is not None]
            if vals:
                return float(np.median(vals))
        vals, last = [], None
        end = time.monotonic() + window_s
        while time.monotonic() < end and len(vals) < samples:
            scan = self._scan
            if scan is not None and (last is None or changed_bins(scan, last) > 0):
                last = np.array(scan)
                h = self._wall_heading(last)
                if h is not None:
                    vals.append(h)
            self._sleep(0.05)
        return float(np.median(vals)) if vals else None

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
        stall_since = None

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
            # Commanded but not turning: this floor needs more than min_v.
            if abs(rate) < self.stall_dps:
                if stall_since is None:
                    stall_since = now
                elif now - stall_since > self.stall_s:
                    self._ctrl.bump(self.stall_step)
                    stall_since = now
            else:
                stall_since = None
            self._sleep(period)

    def _settle(self, deadline, rev_s, settle_s):
        """Stop, wait for the buffer to refresh, and prove the robot is still.
        The two scans that proved it are left in _still_scans."""
        period = 1.0 / self.control_hz
        pred = self._progress
        end = time.monotonic() + settle_s
        # Keep matching and publishing zeros so motor_control ramps down.
        anchors, scans = [], []
        next_anchor = end
        hard_end = time.monotonic() + settle_s + 2.0
        self._still_scans = []

        while True:
            now = time.monotonic()
            if self._cancel.is_set():
                return Status.CANCELLED, pred, "cancelled"
            if now > deadline or now > hard_end:
                return Status.UNSETTLED, pred, "never came to rest"
            if not self._scan_ok():
                return Status.NO_SCAN, pred, "lidar/scans stopped"

            self._publish_duty(0.0)

            scan = self._scan
            m = match_shift(scan, self._ref, pred, self.window_deg,
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
                    scans.append(np.array(scan))
                    next_anchor = now + rev_s
                    # Two readings a revolution apart that agree mean a stationary buffer.
                    if len(anchors) >= 2:
                        if abs(anchors[-1] - anchors[-2]) <= self.still_tol_deg:
                            self._still_scans = scans[-2:]
                            return None, pred, ""
                        anchors, scans = anchors[-1:], scans[-1:]
            self._sleep(period)
