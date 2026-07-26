"""Room-centering (translation) control — standalone, independent of alignment.

Drives the robot to the geometric centre of a rectangular room from the four
cardinal LiDAR distances. Usable on its own, or chained after alignment by
``etc.homing.HomingRoutine``.

The robot used to crawl to a halt short of the centre and hang there. Three
compounding causes, all fixed here: pure P-control decays below static friction
on approach; the old "settle zone" scaled the anti-stall floor *down* exactly
where stalling happens; and the EMA was applied *after* the floor, so the value
reaching the motors could be far below it.
"""

try:
    from . import scan_geometry as geom
except ImportError:                     # imported flat (unit tests, scripts)
    import scan_geometry as geom

_UNSET = geom.UNSET


class CenteringParams:
    """Tuning. ``min_v`` is the *active minimum drive speed*: the smallest
    command that reliably breaks static friction, held all the way to
    ``arrival_tol`` so the robot carries momentum into the target instead of
    creeping to a stall."""

    def __init__(self, kp=3.0, min_v=0.35, max_speed=0.85, arrival_tol=0.10,
                 settle_zone=0.30, alpha=0.5, stall_time=1.2,
                 stall_progress=0.02, stall_boost=1.4, min_taper=0.5):
        self.kp = kp                      # command per metre
        self.min_v = min_v
        self.max_speed = max_speed
        self.arrival_tol = arrival_tol    # metres; inside this = centred
        self.settle_zone = settle_zone    # metres; gentle gain taper inside
        self.alpha = alpha                # EMA factor (1.0 = no smoothing)
        self.min_taper = min_taper        # floor on the approach gain taper
        self.stall_time = stall_time          # s without progress = stalled
        self.stall_progress = stall_progress  # m of closing that counts
        self.stall_boost = stall_boost        # multiplier when stalled


def centering_command(cardinals, params=None, prev_vx=0.0, prev_vy=0.0,
                      boost=1.0):
    """P-control with an anti-stall floor -> ``(vx, vy, distance, arrived)``.

    ``vx`` maps to Twist.linear.x, ``vy`` to Twist.linear.y. The magnitude is
    clamped into ``[min_v, max_speed]`` for the whole approach: deceleration
    comes from the gain taper and the arrival test, never from letting the
    command decay into the friction band.
    """
    params = params or CenteringParams()

    ex, ey, dist = geom.center_error(cardinals)
    if dist < params.arrival_tol:
        return 0.0, 0.0, dist, True

    speed = params.kp * dist
    if dist < params.settle_zone:
        # Taper touches the *gain* only, never the friction floor below.
        speed *= max(dist / params.settle_zone, params.min_taper)
    speed = min(max(speed * boost, params.min_v), params.max_speed)

    # Direction from the unit error vector, magnitude from the clamped speed:
    # keeps the robot pointed at the target however lopsided the axis errors.
    vx, vy = (ex / dist) * speed, (ey / dist) * speed

    a = params.alpha
    vx = a * vx + (1.0 - a) * prev_vx
    vy = a * vy + (1.0 - a) * prev_vy

    # Floor re-asserted *after* smoothing, so the EMA can never hand the motors
    # a command too small to move the robot.
    vx, vy = geom.clamp_magnitude(vx, vy, params.min_v * boost, params.max_speed)
    return vx, vy, dist, False


class CenteringState:
    """EMA state, arrival counter and stall detector, so ``centering_command``
    can stay a pure function."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.arrival_count = 0
        self.stalled = False
        self._best_dist = None
        self._best_time = None

    def _update_stall(self, dist, now, params):
        """Detect "moving but not getting closer" and ask for more command.
        ``now`` is any monotonic float; the check is skipped without one."""
        if now is None:
            return 1.0
        if self._best_dist is None or dist < self._best_dist - params.stall_progress:
            self._best_dist = dist
            self._best_time = now
            self.stalled = False
            return 1.0
        if self._best_time is not None and (now - self._best_time) > params.stall_time:
            self.stalled = True
            self._best_time = now      # re-arm rather than latch forever
        return params.stall_boost if self.stalled else 1.0

    def step(self, cardinals, params=None, now=None):
        """One step -> ``(vx, vy, dist, arrived)``."""
        params = params or CenteringParams()
        boost = self._update_stall(geom.center_error(cardinals)[2], now, params)

        vx, vy, dist, arrived = centering_command(
            cardinals, params, self.prev_vx, self.prev_vy, boost=boost)
        self.prev_vx, self.prev_vy = vx, vy
        if arrived:
            self.arrival_count += 1
            self.stalled = False
        else:
            self.arrival_count = 0
        return vx, vy, dist, arrived


class CenteringController:
    """Centering with EMA, stall recovery and an optional drift check.

        ctrl = CenteringController()
        vx, vy, dist, arrived, drift = ctrl.step(scan, now=time.time())

    ``drift`` is reported but never acted on — the caller decides whether to
    re-align, which is what keeps this independent of ``etc.alignment``.
    """

    def __init__(self, params=None, confirm_frames=5, drift_tol=None):
        self.params = params or CenteringParams()
        self.confirm_frames = confirm_frames
        self.drift_tol = drift_tol      # degrees; None disables the check
        self._state = CenteringState()

    def reset(self):
        self._state.reset()

    @property
    def stalled(self):
        return self._state.stalled

    def step(self, ranges, now=None, cardinals=_UNSET, heading_err=_UNSET):
        """One step -> ``(vx, vy, dist, arrived, drift)``.

        ``arrived`` only goes True after ``confirm_frames`` consecutive
        in-tolerance frames, and the command is zero on those frames.
        ``cardinals`` / ``heading_err`` may be supplied when the caller already
        computed them for this scan.
        """
        if cardinals is _UNSET:
            cardinals = geom.cardinal_distances(ranges)
        if cardinals is None:
            return 0.0, 0.0, 0.0, False, False

        vx, vy, dist, _ = self._state.step(cardinals, self.params, now=now)
        confirmed = self._state.arrival_count >= self.confirm_frames

        drift = False
        if self.drift_tol is not None:
            if heading_err is _UNSET:
                heading_err = geom.wall_heading_error(ranges)
            drift = heading_err is not None and abs(heading_err) > self.drift_tol
        return vx, vy, dist, confirmed, drift


def center_distance(ranges):
    """Distance in metres from the room centre, or None for a bad scan."""
    cardinals = geom.cardinal_distances(ranges)
    return None if cardinals is None else geom.center_error(cardinals)[2]
