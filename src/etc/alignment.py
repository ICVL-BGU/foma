"""Wall-alignment (yaw) control — standalone, independent of centering.

Give it a scan (or any heading error from any source) and it produces a yaw
command. Usable on its own, or chained with ``etc.centering`` by
``etc.homing.HomingRoutine``.

The legacy controller (``v = 30.0 * err``, later ``kp=0.06``, plus a hard
dead-band) oscillated the moment it was switched on: the gain saturated the
command for any error past a couple of degrees, and the dead-band stepped it
discontinuously from +min through 0 to -min across the setpoint — bang-bang
limit cycling, amplified by the ~180 ms LiDAR buffer lag. The fix is a lower
gain, a derivative term, a low-pass on the heading estimate, slew limiting, a
direction-flip guard and hysteresis on the tolerance band.
"""

import math

try:
    from . import scan_geometry as geom
except ImportError:                     # imported flat (unit tests, scripts)
    import scan_geometry as geom

_UNSET = geom.UNSET


class AlignParams:
    """Tuning for the yaw controller. ``kp`` maps degrees of error to a motor
    command in [-1, 1] (motor_control_node scales by magnitude, so this is a
    real proportional command, not just a sign)."""

    def __init__(self, kp=0.018, min_ang_v=0.13, max_ang_v=0.40, angle_tol=2.5,
                 kd=0.004, filter_alpha=0.35, slow_zone=12.0,
                 release_scale=1.8, max_slew=0.05, flip_frames=3,
                 nominal_dt=0.05, approach_floor_scale=1.0):
        self.kp = kp                        # command per degree
        self.kd = kd                        # command per degree/second
        self.min_ang_v = min_ang_v          # static-friction floor
        self.max_ang_v = max_ang_v
        self.angle_tol = angle_tol          # degrees; inside this = aligned
        self.filter_alpha = filter_alpha    # EMA on the heading estimate
        self.slow_zone = slow_zone          # deg; floor ramp region
        self.release_scale = release_scale  # hysteresis leaving tolerance
        self.max_slew = max_slew            # max command change per step
        self.flip_frames = flip_frames      # frames of agreement before a flip
        self.nominal_dt = nominal_dt        # assumed step when dt is unknown
        # Must stay >= 1.0: ramping the floor *under* static friction is what
        # makes a controller creep to a halt just outside tolerance forever.
        self.approach_floor_scale = approach_floor_scale


def alignment_command(heading_err, params=None):
    """Stateless P + dead-band yaw command -> ``(angular_z, aligned)``."""
    params = params or AlignParams()
    if abs(heading_err) < params.angle_tol:
        return 0.0, True
    return geom.apply_deadband(heading_err * params.kp, params.min_ang_v,
                               params.max_ang_v), False


class AlignmentController:
    """Smooth yaw alignment with confirmation counting.

        ctrl = AlignmentController()
        wz, aligned, err = ctrl.step(scan_ranges_in_metres)

    ``aligned`` only goes True after ``confirm_frames`` consecutive in-tolerance
    frames, and the command is 0.0 on those frames, so the caller can publish
    the result verbatim and the robot stops itself.
    """

    def __init__(self, params=None, confirm_frames=5):
        self.params = params or AlignParams()
        self.confirm_frames = confirm_frames
        self.reset()

    def reset(self):
        self._count = 0
        self._filtered = None
        self._prev_filtered = None
        self._prev_cmd = 0.0
        self._flip_count = 0
        self._cmd_sign = 0

    def _filter(self, heading_err):
        # Initialised to the first sample so the first command isn't damped to 0.
        a = self.params.filter_alpha
        if self._filtered is None:
            self._filtered = heading_err
        else:
            self._filtered = a * heading_err + (1.0 - a) * self._filtered
        return self._filtered

    def _effective_floor(self, err_mag):
        """Approach floor, bounded below by ``min_ang_v`` (see AlignParams)."""
        p = self.params
        floor_hi = p.min_ang_v * max(p.approach_floor_scale, 1.0)
        if err_mag >= p.slow_zone:
            return floor_hi
        frac = (err_mag - p.angle_tol) / max(p.slow_zone - p.angle_tol, 1e-6)
        return p.min_ang_v + (floor_hi - p.min_ang_v) * min(max(frac, 0.0), 1.0)

    def _guard_flip(self, cmd):
        """Require several consecutive frames of agreement before reversing —
        one noisy scan must never slam the drive into reverse."""
        sign = 0 if cmd == 0.0 else (1 if cmd > 0 else -1)
        if sign == 0 or self._cmd_sign == 0 or sign == self._cmd_sign:
            self._flip_count = 0
            if sign != 0:
                self._cmd_sign = sign
            return cmd
        self._flip_count += 1
        if self._flip_count >= self.params.flip_frames:
            self._flip_count = 0
            self._cmd_sign = sign
            return cmd
        return 0.0

    def step(self, ranges, dt=None, heading_err=_UNSET):
        """One step from a scan -> ``(angular_z, aligned, heading_err)``.

        ``heading_err`` may be passed in when the caller already estimated it;
        the estimator is a 360-beam Python loop and is the node's largest
        avoidable per-frame cost.
        """
        if heading_err is _UNSET:
            heading_err = geom.wall_heading_error(ranges)
        if heading_err is None:
            self._prev_cmd = 0.0
            return 0.0, False, None
        return self.step_from_error(heading_err, dt=dt)

    def step_from_error(self, heading_err, dt=None):
        """Same as :meth:`step`, driven by any external heading source."""
        p = self.params
        err = self._filter(heading_err)
        err_mag = abs(err)

        if err_mag < p.angle_tol:
            self._count += 1
        elif err_mag > p.angle_tol * p.release_scale:
            # Only a clear departure resets the counter; noise inside the
            # hysteresis band must not restart the whole alignment phase.
            self._count = 0

        if err_mag < p.angle_tol:
            self._prev_filtered = err
            self._prev_cmd = 0.0
            return 0.0, self._count >= self.confirm_frames, heading_err

        step_dt = dt if (dt is not None and dt > 1e-6) else p.nominal_dt
        rate = 0.0 if self._prev_filtered is None else \
            (err - self._prev_filtered) / step_dt
        self._prev_filtered = err

        # Derivative opposes the current motion: back off when the error is
        # already closing fast. Clamped so a noise spike can't dominate.
        damping = max(-0.5 * p.max_ang_v,
                      min(0.5 * p.max_ang_v, p.kd * rate))
        cmd = p.kp * err + damping

        # The derivative may shrink the command but must not invert it.
        floor = self._effective_floor(err_mag)
        if cmd == 0.0 or (cmd > 0) != (err > 0):
            cmd = math.copysign(floor, err)
        else:
            cmd = geom.apply_deadband(cmd, floor, p.max_ang_v)

        cmd = geom.slew_limit(self._guard_flip(cmd), self._prev_cmd, p.max_slew)
        self._prev_cmd = cmd
        return cmd, False, heading_err
