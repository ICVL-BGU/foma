"""Precise rotation, the align+centre routine, and the public ``homing`` API.

Layout of the homing code:

    etc/scan_geometry.py   pure LiDAR maths (no deps)
    etc/alignment.py       yaw alignment    — standalone controller
    etc/centering.py       room centering   — standalone controller
    etc/homing.py          rotation + the routine that chains the above

Alignment and centering never reference each other; this module is the only
place they are combined, which is what makes align-only, centre-only and the
full sequence three configurations of one object. Everything is re-exported
under the historical ``homing.*`` names, so ``from etc import homing`` and the
existing test suite keep working.
"""

try:
    from . import scan_geometry as geom
    from .scan_geometry import (                       # noqa: F401
        sample_distance, cardinal_distances, center_error, apply_deadband,
        clamp_magnitude, slew_limit, scan_to_points, fold_angle,
        unwrap_folded_delta, wall_heading_error, scan_match_delta_deg,
        render_room_view,
    )
    from .alignment import (                           # noqa: F401
        AlignParams, alignment_command, AlignmentController,
    )
    from .centering import (                           # noqa: F401
        CenteringParams, CenteringState, CenteringController,
        centering_command, center_distance,
    )
except ImportError:                     # imported flat (unit tests, scripts)
    import scan_geometry as geom
    from scan_geometry import (                        # noqa: F401
        sample_distance, cardinal_distances, center_error, apply_deadband,
        clamp_magnitude, slew_limit, scan_to_points, fold_angle,
        unwrap_folded_delta, wall_heading_error, scan_match_delta_deg,
        render_room_view,
    )
    from alignment import (                            # noqa: F401
        AlignParams, alignment_command, AlignmentController,
    )
    from centering import (                            # noqa: F401
        CenteringParams, CenteringState, CenteringController,
        centering_command, center_distance,
    )

_UNSET = geom.UNSET


# =========================================================================== #
#  Precise relative rotation (LiDAR feedback)                                 #
# =========================================================================== #
# There is no IMU and no wheel odometry, so "turn 90 degrees" is closed on the
# LiDAR: the folded wall angle is precise but ambiguous modulo 90 degrees (so
# it only works as a frame-to-frame delta), and scan matching is coarser but
# unambiguous. Both are fused below.
#
# The previous version span forever because it only ever exited on
# |remaining| <= tol. It had no overshoot exit (passing the target made it turn
# back and hunt), a single feedback source that one bad median could corrupt
# permanently, no direction or progress supervision, and it kept publishing the
# spin command indefinitely when the heading estimate was unavailable.

class RotationParams:
    """Tuning and safety envelope for one relative rotation."""

    def __init__(self, tol=2.0, slow_zone=20.0, cmd=0.35, min_cmd=0.18,
                 timeout=25.0, stall_time=2.5, stall_progress=1.0,
                 wrong_way_grace=2.0, wrong_way_deg=10.0, blind_time=1.5,
                 max_step_deg=40.0):
        self.tol = tol                    # deg; within this we are done
        self.slow_zone = slow_zone        # deg; start decelerating here
        self.cmd = cmd                    # full-speed command magnitude
        self.min_cmd = min_cmd            # static-friction floor for the spin
        self.timeout = timeout            # s; hard cap on one rotation
        self.stall_time = stall_time      # s without progress = stalled
        self.stall_progress = stall_progress   # deg that counts as progress
        self.wrong_way_grace = wrong_way_grace  # s to settle before judging
        self.wrong_way_deg = wrong_way_deg      # deg *away* that aborts
        self.blind_time = blind_time      # s; max time with no heading
        self.max_step_deg = max_step_deg  # reject deltas larger than this


class RotationTracker:
    """Accumulate signed yaw travelled from folded wall-angle measurements.

    The folded angle lives in [-45, 45), so a raw reading can't tell 0 from 90.
    Unwrapping the *change* between successive readings (each step turns well
    under 45 degrees at any sane spin rate) recovers the true rotation.

    Two robustness additions over the original: deltas too large to be physical
    are dropped rather than integrated (one bad median used to corrupt the
    total permanently), and when the raw scan is supplied an independent
    scan-match estimate arbitrates a folded delta that disagrees with it.
    """

    def __init__(self, max_step_deg=40.0, disagree_deg=12.0):
        self.max_step_deg = max_step_deg
        self.disagree_deg = disagree_deg
        self.reset()

    def reset(self):
        self.total = 0.0
        self.rejected = 0
        self.samples = 0
        self._prev = None
        self._prev_ranges = None

    @property
    def travelled(self):
        return self.total

    def update(self, folded_deg, ranges=None):
        """Feed a folded wall angle; return cumulative rotation (+ = CCW)."""
        self.samples += 1
        if self._prev is None:
            self._prev = folded_deg
            self._prev_ranges = None if ranges is None else list(ranges)
            return self.total

        # Walls appear to rotate opposite to the robot, hence the negation.
        delta = -geom.unwrap_folded_delta(folded_deg - self._prev)

        match = None
        if ranges is not None and self._prev_ranges is not None:
            match = geom.scan_match_delta_deg(self._prev_ranges, ranges)

        if abs(delta) > self.max_step_deg:
            self.rejected += 1          # impossible between two frames
            delta = 0.0
        elif match is not None and abs(match - delta) > self.disagree_deg:
            self.rejected += 1          # trust the unambiguous estimate
            delta = match

        self.total += delta
        self._prev = folded_deg
        if ranges is not None:
            self._prev_ranges = list(ranges)
        return self.total


class RotationController:
    """Decide how hard to turn given the remaining angle.

    ``MotorControl.rotate`` scales with command magnitude below 1.0, so the
    spin really can be slowed near the target. The old pulse/duty-cycle hack
    was self-defeating: a pulse shorter than motor_control_node's acceleration
    ramp (0.03 per 20 ms) produced no motion at all.
    """

    STOP = "stop"
    FULL = "full"
    SLOW = "slow"

    def __init__(self, tol=2.0, slow_zone=15.0, cmd=0.35, min_cmd=0.18):
        self.tol = tol
        self.slow_zone = slow_zone
        self.cmd = cmd
        self.min_cmd = min_cmd

    def classify(self, remaining_deg):
        """-> ``(mode, sign)`` for the remaining angle."""
        if abs(remaining_deg) <= self.tol:
            return self.STOP, 0
        sign = 1 if remaining_deg > 0 else -1
        if abs(remaining_deg) <= self.slow_zone:
            return self.SLOW, sign
        return self.FULL, sign

    def command(self, remaining_deg):
        """-> signed angular command."""
        mode, sign = self.classify(remaining_deg)
        if mode == self.STOP:
            return 0.0
        if mode == self.FULL:
            return sign * self.cmd
        # Proportional deceleration, never below the friction floor (or the
        # last few degrees never happen).
        frac = abs(remaining_deg) / max(self.slow_zone, 1e-6)
        return sign * max(self.min_cmd, self.cmd * frac)


class RotationStatus:
    """Result of one :meth:`PreciseRotator.update`."""

    def __init__(self, angular_z=0.0, done=False, success=False, reason='',
                 travelled=0.0, remaining=0.0):
        self.angular_z = angular_z
        self.done = done
        self.success = success
        self.reason = reason
        self.travelled = travelled
        self.remaining = remaining

    def __repr__(self):
        return (f'RotationStatus(cmd={self.angular_z:+.3f}, done={self.done}, '
                f'success={self.success}, reason={self.reason!r}, '
                f'travelled={self.travelled:+.1f}, '
                f'remaining={self.remaining:+.1f})')


class PreciseRotator:
    """Closed-loop relative rotation with a complete termination guarantee.

    Every ``update`` either returns a command with ``done=False``, or returns
    ``angular_z == 0.0`` with ``done=True``. Six independent conditions reach
    that exit — target reached, overshoot, timeout, stall, wrong direction and
    blind — so no path through this class keeps commanding a turn forever.
    """

    def __init__(self, params=None):
        self.params = params or RotationParams()
        self.tracker = RotationTracker(max_step_deg=self.params.max_step_deg)
        self.controller = RotationController(
            tol=self.params.tol, slow_zone=self.params.slow_zone,
            cmd=self.params.cmd, min_cmd=self.params.min_cmd)
        self.active = False
        self.target = 0.0
        self._start_time = None
        self._best_remaining = None
        self._best_time = None
        self._last_seen = None
        self._last_cmd = 0.0

    def start(self, target_deg, now):
        """Begin a rotation of ``target_deg`` (signed; + = CCW)."""
        self.target = float(target_deg)
        self.tracker.reset()
        self.active = True
        self._start_time = now
        self._best_remaining = abs(self.target)
        self._best_time = now
        self._last_seen = now
        self._last_cmd = 0.0
        return self.active

    def cancel(self, reason='cancelled'):
        return self._finish(False, reason)

    def _finish(self, success, reason):
        self.active = False
        self._last_cmd = 0.0
        return RotationStatus(0.0, done=True, success=success, reason=reason,
                              travelled=self.tracker.total,
                              remaining=self.target - self.tracker.total)

    def update(self, heading_err, now, ranges=None):
        """Advance by one measurement.

        ``heading_err`` is the folded wall angle in degrees, or None when the
        scan has no usable wall structure. ``now`` is a monotonic float.
        """
        if not self.active:
            return RotationStatus(0.0, done=True, success=False, reason='idle')
        p = self.params

        if self._start_time is not None and (now - self._start_time) > p.timeout:
            return self._finish(False, 'timeout')

        # No usable heading: hold the last command, but never indefinitely.
        if heading_err is None:
            if self._last_seen is not None and (now - self._last_seen) > p.blind_time:
                return self._finish(False, 'no_lidar_heading')
            return RotationStatus(self._last_cmd, reason='blind',
                                  travelled=self.tracker.total,
                                  remaining=self.target - self.tracker.total)

        self._last_seen = now
        travelled = self.tracker.update(heading_err, ranges=ranges)
        remaining = self.target - travelled

        if abs(remaining) <= p.tol:
            return self._finish(True, 'reached')

        # Overshoot: we passed the target. Stop rather than turn back — that
        # hunt is what oscillated around the setpoint indefinitely.
        if (remaining > 0) != (self.target > 0):
            return self._finish(True, 'overshoot')

        elapsed = now - self._start_time if self._start_time is not None else 0.0
        if elapsed > p.wrong_way_grace and \
                abs(remaining) > abs(self.target) + p.wrong_way_deg:
            return self._finish(False, 'wrong_direction')

        if self._best_remaining is None or \
                abs(remaining) < self._best_remaining - p.stall_progress:
            self._best_remaining = abs(remaining)
            self._best_time = now
        elif self._best_time is not None and (now - self._best_time) > p.stall_time:
            return self._finish(False, 'stalled')

        self._last_cmd = self.controller.command(remaining)
        return RotationStatus(self._last_cmd, reason='turning',
                              travelled=travelled, remaining=remaining)


# =========================================================================== #
#  The align + centre routine (the only place the two are chained)            #
# =========================================================================== #
class HomingResult:
    """Outcome of one :meth:`HomingRoutine.step`."""

    def __init__(self, vx=0.0, vy=0.0, wz=0.0, state='idle', done=False,
                 success=False, reason='', heading_err=None, dist=None):
        self.vx = vx
        self.vy = vy
        self.wz = wz
        self.state = state
        self.done = done
        self.success = success
        self.reason = reason
        self.heading_err = heading_err
        self.dist = dist

    def __repr__(self):
        return (f'HomingResult(state={self.state!r}, done={self.done}, '
                f'v=({self.vx:+.2f},{self.vy:+.2f}), wz={self.wz:+.2f})')


class HomingRoutine:
    """Align and/or centre the robot, as a restartable state machine.

        HomingRoutine(mode=HomingRoutine.ALIGN_ONLY)    # square up only
        HomingRoutine(mode=HomingRoutine.CENTER_ONLY)   # drive to centre only
        HomingRoutine(mode=HomingRoutine.FULL)          # both, then verify

    ALIGN -> SETTLE -> CENTER -> VERIFY. SETTLE is a frame counter, not a
    sleep, letting the chassis come to rest before the cardinal distances are
    trusted. VERIFY re-checks both criteria and sends the routine back to
    whichever stage failed. Hardware independent: ``step`` returns a command
    and a state, and the ROS node only has to publish it.

    The two controllers are exposed as ``self.align`` / ``self.center`` so a
    caller can reuse or re-tune either directly.
    """

    ALIGN_ONLY = 'align'
    CENTER_ONLY = 'center'
    FULL = 'full'

    ST_ALIGN = 'ALIGN'
    ST_SETTLE = 'SETTLE'
    ST_CENTER = 'CENTER'
    ST_VERIFY = 'VERIFY'
    ST_DONE = 'DONE'

    def __init__(self, mode=FULL, align_ctrl=None, center_ctrl=None,
                 settle_frames=6, verify_align_scale=1.2,
                 verify_center_scale=1.5, max_retries=4):
        self.mode = mode
        self.align = align_ctrl or AlignmentController(confirm_frames=5)
        self.center = center_ctrl or CenteringController(confirm_frames=5,
                                                         drift_tol=8.0)
        self.settle_frames = settle_frames
        self.verify_align_scale = verify_align_scale
        self.verify_center_scale = verify_center_scale
        self.max_retries = max_retries
        self.reset()

    def reset(self, mode=None):
        if mode is not None:
            self.mode = mode
        self.align.reset()
        self.center.reset()
        self._settle_count = 0
        self._retries = 0
        self.state = (self.ST_CENTER if self.mode == self.CENTER_ONLY
                      else self.ST_ALIGN)

    def _enter_align(self):
        self.align.reset()
        self.state = self.ST_ALIGN

    def _enter_center(self):
        self.center.reset()
        self.state = self.ST_CENTER

    def _finish(self, reason='complete'):
        self.state = self.ST_DONE
        return HomingResult(state=self.ST_DONE, done=True, success=True,
                            reason=reason)

    def step(self, ranges, now=None, dt=None, heading_err=_UNSET,
             cardinals=_UNSET):
        """One step from a scan. ``heading_err`` / ``cardinals`` may be
        supplied by a caller that already derived them from this scan."""
        if self.state == self.ST_DONE:
            return HomingResult(state=self.ST_DONE, done=True, success=True,
                                reason='already_done')

        if self.state == self.ST_ALIGN:
            wz, aligned, err = self.align.step(ranges, dt=dt,
                                               heading_err=heading_err)
            if err is None:
                return HomingResult(state=self.ST_ALIGN, reason='no_heading')
            if aligned:
                if self.mode == self.ALIGN_ONLY:
                    return self._finish('aligned')
                self._settle_count = 0
                self.state = self.ST_SETTLE
                return HomingResult(state=self.ST_SETTLE, heading_err=err,
                                    reason='aligned')
            return HomingResult(wz=wz, state=self.ST_ALIGN, heading_err=err,
                                reason='aligning')

        if self.state == self.ST_SETTLE:
            self._settle_count += 1
            if self._settle_count >= self.settle_frames:
                self._enter_center()
                return HomingResult(state=self.ST_CENTER, reason='settled')
            return HomingResult(state=self.ST_SETTLE, reason='settling')

        if self.state == self.ST_CENTER:
            vx, vy, dist, arrived, drift = self.center.step(
                ranges, now=now, cardinals=cardinals, heading_err=heading_err)
            # Skewed cardinal readings: re-square first. Bounded by
            # max_retries so align/centre cannot ping-pong forever.
            if drift and self.mode == self.FULL and self._retries < self.max_retries:
                self._retries += 1
                self._enter_align()
                return HomingResult(state=self.ST_ALIGN, dist=dist,
                                    reason='drift')
            if arrived:
                if self.mode == self.CENTER_ONLY:
                    return self._finish('centred')
                self.state = self.ST_VERIFY
                return HomingResult(state=self.ST_VERIFY, dist=dist,
                                    reason='centred')
            return HomingResult(vx=vx, vy=vy, state=self.ST_CENTER, dist=dist,
                                reason='centring')

        if self.state == self.ST_VERIFY:
            if heading_err is _UNSET:
                heading_err = geom.wall_heading_error(ranges)
            if cardinals is _UNSET:
                cardinals = geom.cardinal_distances(ranges)
            if heading_err is None or cardinals is None:
                return HomingResult(state=self.ST_VERIFY, reason='bad_scan')

            dist = geom.center_error(cardinals)[2]
            heading_ok = abs(heading_err) < (self.align.params.angle_tol *
                                             self.verify_align_scale)
            center_ok = dist < (self.center.params.arrival_tol *
                                self.verify_center_scale)
            if heading_ok and center_ok:
                return self._finish('verified')
            if self._retries >= self.max_retries:
                # Good enough; retrying further would just burn the timeout.
                return self._finish('verify_retries_exhausted')
            self._retries += 1

            if not heading_ok:
                self._enter_align()
                return HomingResult(state=self.ST_ALIGN, heading_err=heading_err,
                                    dist=dist, reason='verify_failed_heading')
            self._enter_center()
            return HomingResult(state=self.ST_CENTER, heading_err=heading_err,
                                dist=dist, reason='verify_failed_centre')

        return HomingResult(state=self.state, reason='unknown_state')
