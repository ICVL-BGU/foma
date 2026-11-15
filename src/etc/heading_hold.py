"""Keep the robot square to the room while it translates.

Translation does not change the wall heading, so any change is an unwanted
turn (wheel slip); a small rotation mixed into the drive cancels it. No ROS:
motor_control_node feeds a heading per tick and mixes in the result.
"""

import math

try:
    from . import scan_geometry as geom
except ImportError:                     # imported flat (unit tests, scripts)
    import scan_geometry as geom


class HeadingHoldParams:
    def __init__(self, kp=0.010, ki=0.004, max_corr=0.15, deadband=0.5,
                 filter_alpha=0.3, i_limit=0.08, decay=0.9,
                 runaway_deg=6.0, runaway_s=1.5, sign=1):
        self.kp = kp                    # duty per degree
        self.ki = ki                    # duty per degree-second
        self.max_corr = max_corr        # cap on the mixed-in rotation
        self.deadband = deadband        # deg; P term ignores smaller errors
        self.filter_alpha = filter_alpha
        self.i_limit = i_limit          # cap on the integral's contribution
        self.decay = decay              # per tick, while blind
        self.runaway_deg = runaway_deg  # error growth that trips the guard
        self.runaway_s = runaway_s
        self.sign = sign                # angular sign that reduces the error


class HeadingHold:
    """PI on the folded wall heading (target 0 = square to the room).

    The integral is what cancels a wheel that slips *consistently*: P alone
    would leave a steady angle. If the error keeps growing while we push
    against it the sign is wrong, and the hold disables itself (``tripped``)
    rather than spin the robot.
    """

    def __init__(self, params=None):
        self.p = params or HeadingHoldParams()
        self.tripped = False
        self.reset()

    def reset(self):
        """Call when the robot stops or turns on purpose."""
        self._filt = None
        self._integral = 0.0
        self._corr = 0.0
        self._best = None
        self._best_t = None

    def update(self, heading_deg, dt, now):
        """One tick.  ``heading_deg`` is None when the walls are not trusted.
        Returns the rotation to mix in (same units as angular.z)."""
        p = self.p
        if self.tripped:
            return 0.0
        if heading_deg is None:
            self._corr *= p.decay       # hold briefly, never act on a guess
            return self._corr

        if self._filt is None:
            self._filt = heading_deg
        else:
            # Filter the *change* so readings either side of the fold agree.
            step = geom.unwrap_folded_delta(heading_deg - self._filt)
            self._filt = geom.fold_angle(self._filt + p.filter_alpha * step)
        err = self._filt

        self._integral += err * dt
        lim = p.i_limit / max(p.ki, 1e-9)
        self._integral = max(-lim, min(lim, self._integral))

        pe = 0.0 if abs(err) < p.deadband else err
        u = p.kp * pe + p.ki * self._integral
        u = max(-p.max_corr, min(p.max_corr, u))
        self._corr = -p.sign * u

        self._check_runaway(abs(err), now)
        return 0.0 if self.tripped else self._corr

    def _check_runaway(self, mag, now):
        if self._best is None or mag < self._best:
            self._best, self._best_t = mag, now
        elif (mag > self._best + self.p.runaway_deg
              and now - self._best_t > self.p.runaway_s):
            self.tripped = True
            self._corr = 0.0
