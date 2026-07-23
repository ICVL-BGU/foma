"""Hardware-independent homing / alignment / rotation geometry for FOMA.

This module contains the *pure math* used by the homing (`go_home_node`) and
precise-rotation (`rotation_node`) ROS nodes. It deliberately has **no ROS
dependency** so the logic can be unit-tested on any machine with NumPy
(see ``test/test_homing.py``).

Design goals (see CHANGELOG_AND_EXPLANATION.md):
  * Alignment (yaw correction) and centering (translation to room centre) are
    fully decoupled - each is a standalone function that takes a LiDAR scan
    (ranges in metres) and returns a command. Either can be reused on its own,
    e.g. continuous yaw correction while driving down a corridor.
  * Friction / dead-band compensation is a single reusable helper so the same
    behaviour is applied everywhere motion commands are produced.
  * Wall-orientation estimation is robust to the "45-degree corner" failure of
    the previous fixed-sector least-squares fit: it estimates a single room
    orientation from *all* wall segments and folds it into the [-45, 45)
    range, so a corner never corrupts the estimate.

Coordinate / sign conventions (kept identical to the original go_home_node so
on-robot behaviour is preserved):
  * ``ranges`` is a length-N array (N usually 360); index ``i`` is angle ``i``
    degrees, and a point is ``x = r*cos(i deg)``, ``y = r*sin(i deg)``.
  * Cardinal sampling used by centering: front=270, back=90, left=0, right=180.
  * ``error_x = (front-back)/2``  ``error_y = (left-right)/2``.
  * ``heading error`` is the room orientation folded to [-45, 45); 0 means the
    robot is aligned with the walls. Positive/negative sign matches the
    original ``get_heading_error`` (both fold wall orientation the same way),
    so the existing proportional gains keep the same closed-loop sign.
"""

import math
import numpy as np


# --------------------------------------------------------------------------- #
#  Tunable defaults (mirrors of the original go_home_node constants).          #
# --------------------------------------------------------------------------- #
class CenteringParams:
    """Parameters for the translate-to-centre controller."""

    def __init__(self, kp=1.3, min_v=0.18, max_speed=0.55, arrival_tol=0.15,
                 settle_zone=0.30):
        self.kp = kp
        self.min_v = min_v            # dead-band / static-friction floor
        self.max_speed = max_speed
        self.arrival_tol = arrival_tol
        self.settle_zone = settle_zone  # ramp min_v down inside this radius


class AlignParams:
    """Parameters for the yaw-alignment controller."""

    def __init__(self, kp=0.06, min_ang_v=0.16, max_ang_v=0.5, angle_tol=3.0):
        self.kp = kp
        self.min_ang_v = min_ang_v    # dead-band / static-friction floor
        self.max_ang_v = max_ang_v
        self.angle_tol = angle_tol    # degrees


# --------------------------------------------------------------------------- #
#  Low-level scan helpers.                                                     #
# --------------------------------------------------------------------------- #
def sample_distance(ranges, angle_deg, half_window=10, r_min=0.1, r_max=15.0):
    """Robust distance in a +/-``half_window`` cone around ``angle_deg``.

    Returns the median of valid readings, or ``None`` if none are valid.
    Matches the original ``get_dist`` behaviour.
    """
    ranges = np.asarray(ranges, dtype=float)
    n = len(ranges)
    idx = [(int(angle_deg) + i) % n for i in range(-half_window, half_window)]
    vals = ranges[idx]
    valid = vals[(vals > r_min) & (vals < r_max)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def cardinal_distances(ranges, front=270, back=90, left=0, right=180):
    """Return ``(front, back, left, right)`` distances (metres) or ``None``.

    ``None`` is returned for the whole tuple if any direction has no valid
    reading, so callers can bail out cleanly.
    """
    f = sample_distance(ranges, front)
    b = sample_distance(ranges, back)
    l = sample_distance(ranges, left)
    r = sample_distance(ranges, right)
    if None in (f, b, l, r):
        return None
    return f, b, l, r


def center_error(cardinals):
    """From ``(f, b, l, r)`` return ``(error_x, error_y, distance)``.

    Positive ``error_x`` -> robot is offset toward *front*; positive
    ``error_y`` -> offset toward *left*. Distance is the Euclidean offset from
    the geometric centre of the (assumed rectangular) room.
    """
    f, b, l, r = cardinals
    error_x = (f - b) / 2.0
    error_y = (l - r) / 2.0
    return error_x, error_y, math.hypot(error_x, error_y)


def apply_deadband(value, min_mag, max_mag):
    """Friction / dead-band compensation for a single command channel.

    * Boosts any non-zero command up to at least ``min_mag`` so the wheels
      actually overcome static friction instead of buzzing in place.
    * Clips the magnitude to ``max_mag``.
    * Leaves an exact-zero command at zero.
    """
    if value == 0.0:
        return 0.0
    mag = abs(value)
    if mag < min_mag:
        mag = min_mag
    if mag > max_mag:
        mag = max_mag
    return math.copysign(mag, value)


# --------------------------------------------------------------------------- #
#  Centering controller (translation only - decoupled from alignment).        #
# --------------------------------------------------------------------------- #
def centering_command(cardinals, params=None):
    """Proportional + dead-band translate-to-centre controller.

    Returns ``(vx, vy, distance, arrived)`` where ``vx`` maps to the robot's
    front/back axis (Twist.linear.x) and ``vy`` to left/right
    (Twist.linear.y), matching motor_control_node's twist handler.

    Inside the ``settle_zone`` the dead-band floor (``min_v``) is ramped down
    proportionally so the robot can creep in gently instead of oscillating.
    """
    if params is None:
        params = CenteringParams()

    ex, ey, dist = center_error(cardinals)
    if dist < params.arrival_tol:
        return 0.0, 0.0, dist, True

    # Inside the settle zone, scale the dead-band floor down so the robot
    # decelerates smoothly instead of overshooting at the fixed min_v.
    if dist < params.settle_zone:
        frac = dist / params.settle_zone          # 0 at centre … 1 at edge
        effective_min_v = params.min_v * frac
    else:
        effective_min_v = params.min_v

    vx = apply_deadband(ex * params.kp, effective_min_v, params.max_speed)
    vy = apply_deadband(ey * params.kp, effective_min_v, params.max_speed)
    return vx, vy, dist, False


# --------------------------------------------------------------------------- #
#  Wall-orientation estimation (robust, 45-degree-corner safe).               #
# --------------------------------------------------------------------------- #
def scan_to_points(ranges, r_min=0.15, r_max=12.0):
    """Convert a scan to ``(x, y, valid)`` arrays in the robot frame."""
    ranges = np.asarray(ranges, dtype=float)
    n = len(ranges)
    ang = np.deg2rad(np.arange(n))
    x = ranges * np.cos(ang)
    y = ranges * np.sin(ang)
    valid = (ranges > r_min) & (ranges < r_max)
    return x, y, valid


def fold_angle(angle_deg):
    """Fold an orientation into [-45, 45).

    Two wall directions 90 degrees apart map to the same value, so a
    rectangular room has one orientation error regardless of which wall we
    look at. This is what makes the estimate corner-proof.
    """
    return ((angle_deg + 45.0) % 90.0) - 45.0


def wall_heading_error(ranges, r_min=0.15, r_max=12.0, max_gap=0.30,
                       min_segments=8):
    """Estimate the room's yaw error (degrees, folded to [-45, 45)).

    Instead of fitting fixed cardinal sectors (which straddle a corner and
    break at ~45 degrees), this walks the whole scan, measures the orientation
    of every short wall segment between adjacent valid points, discards
    segments that jump across a corner/gap, folds each orientation into
    [-45, 45), and returns the median. The median rejects the handful of
    corner segments, so the estimate stays accurate at any yaw - including the
    problematic 45-degree case.

    Returns ``0.0`` when there is not enough wall structure to decide.
    """
    x, y, valid = scan_to_points(ranges, r_min, r_max)
    n = len(x)
    folded = []
    for i in range(n):
        j = (i + 1) % n
        if not (valid[i] and valid[j]):
            continue
        dx = x[j] - x[i]
        dy = y[j] - y[i]
        seg = math.hypot(dx, dy)
        # Skip degenerate points and segments that leap across a corner/gap
        # (those connect two different walls and would bias the estimate).
        if seg < 1e-4 or seg > max_gap:
            continue
        folded.append(fold_angle(math.degrees(math.atan2(dy, dx))))
    if len(folded) < min_segments:
        return None
    return float(np.median(folded))


def alignment_command(heading_err, params=None):
    """Proportional + dead-band yaw-alignment controller.

    Returns ``(angular_z, aligned)``. Decoupled from centering: give it any
    heading error (from walls, a corridor, etc.) and it produces a spin
    command that respects the static-friction floor.
    """
    if params is None:
        params = AlignParams()

    if abs(heading_err) < params.angle_tol:
        return 0.0, True

    v_ang = apply_deadband(heading_err * params.kp,
                           params.min_ang_v, params.max_ang_v)
    return v_ang, False


# --------------------------------------------------------------------------- #
#  Closed-loop precise rotation (LiDAR feedback).                             #
# --------------------------------------------------------------------------- #
class RotationTracker:
    """Accumulate absolute yaw travelled from folded wall-angle measurements.

    The folded wall angle only lives in [-45, 45), so a raw reading cannot
    distinguish 0 from 90 degrees. By unwrapping the *change* between
    successive readings (assuming each step turns less than 45 degrees, which
    holds at any sane spin rate + LiDAR update rate) we recover the true,
    unbounded rotation travelled. This is the LiDAR-based feedback that lets us
    spin a precise number of degrees with no IMU or wheel odometry.
    """

    def __init__(self):
        self.total = 0.0
        self._prev = None

    def reset(self):
        self.total = 0.0
        self._prev = None

    def update(self, folded_deg):
        """Feed a new folded wall angle; return cumulative signed rotation."""
        if self._prev is None:
            self._prev = folded_deg
            return self.total
        delta = folded_deg - self._prev
        # Unwrap the 90-degree fold discontinuity into (-45, 45].
        while delta > 45.0:
            delta -= 90.0
        while delta <= -45.0:
            delta += 90.0
        self.total += delta
        self._prev = folded_deg
        return self.total


class RotationController:
    """Bang-bang + pulsed spin controller for a fixed-speed rotary drive.

    The Pololu drive spins at a fixed speed (only the *sign* of the command
    matters at the motor), so we cannot slow down near the target by lowering
    magnitude. Instead we:
      * spin at full command until within ``slow_zone`` degrees of the target,
      * then pulse (duty-cycle) the command to creep the last few degrees and
        limit overshoot,
      * and stop within ``tol`` degrees.

    ``classify`` is a pure function (easy to unit-test); the actual on/off
    pulsing timing lives in the node.
    """

    STOP = "stop"
    FULL = "full"
    SLOW = "slow"

    def __init__(self, tol=2.0, slow_zone=15.0):
        self.tol = tol
        self.slow_zone = slow_zone

    def classify(self, remaining_deg):
        """Return (mode, sign) for the remaining angle to target."""
        if abs(remaining_deg) <= self.tol:
            return self.STOP, 0
        sign = 1 if remaining_deg > 0 else -1
        if abs(remaining_deg) <= self.slow_zone:
            return self.SLOW, sign
        return self.FULL, sign


# --------------------------------------------------------------------------- #
#  Room-model rendering (top-down "what the robot sees").                      #
# --------------------------------------------------------------------------- #
def render_room_view(ranges, heading_err=None, cardinals=None,
                     size=480, scale=None, r_max=12.0):
    """Render a top-down BGR image of the room as recognised by the robot.

    Draws the LiDAR dots, the modelled wall lines (from the estimated
    orientation + cardinal distances), the robot at the centre with a forward
    arrow, and - when centering - the offset-to-centre vector.

    Returns an ``(size, size, 3)`` uint8 BGR ndarray. ``cv2`` is imported
    lazily so this module still imports (and its math is testable) on machines
    without OpenCV.
    """
    import cv2  # local import: not needed for the pure-math unit tests

    img = np.full((size, size, 3), 20, dtype=np.uint8)   # dark background
    cx = cy = size // 2

    x, y, valid = scan_to_points(ranges, r_max=r_max)
    # metres -> pixels; auto-scale so the room fills ~80% of the view.
    if scale is None:
        if valid.any():
            span = float(np.max(np.hypot(x[valid], y[valid])))
        else:
            span = 1.0
        span = max(span, 0.5)
        scale = (size * 0.42) / span

    def to_px(px_m, py_m):
        # image x right, image y down; robot forward (world -y at 270 deg) = up
        return int(cx + px_m * scale), int(cy - py_m * scale)

    # LiDAR dots on the modelled walls.
    xs, ys = x[valid], y[valid]
    for pxm, pym in zip(xs, ys):
        u, v = to_px(pxm, pym)
        if 0 <= u < size and 0 <= v < size:
            cv2.circle(img, (u, v), 2, (180, 200, 220), -1)

    # Modelled wall lines from orientation + cardinal distances.
    if heading_err is not None and cardinals is not None:
        f, b, l, r = cardinals
        phi = math.radians(heading_err)
        # Unit vectors for the two wall families (rotated by the yaw error).
        ux, uy = math.cos(phi), math.sin(phi)          # front/back normal axis
        vx, vy = -math.sin(phi), math.cos(phi)         # left/right normal axis
        half = size  # long enough to span the view
        wall_defs = [
            (270, f, (ux, uy), (vx, vy)),   # front wall
            (90,  b, (-ux, -uy), (vx, vy)),  # back wall
            (0,   l, (vx, vy), (ux, uy)),   # left wall
            (180, r, (-vx, -vy), (ux, uy)),  # right wall
        ]
        for _, dist, normal, tangent in wall_defs:
            mxm = normal[0] * dist
            mym = normal[1] * dist
            p1 = to_px(mxm - tangent[0] * half, mym - tangent[1] * half)
            p2 = to_px(mxm + tangent[0] * half, mym + tangent[1] * half)
            cv2.line(img, p1, p2, (80, 180, 255), 2)

        # Offset-to-centre vector. In the robot frame the centre lies at
        # (-ey, ex): front axis (270 deg) is world (0,-1), left axis (0 deg) is
        # (1,0), and the centre is -error_x along front and -error_y along left.
        ex, ey, _ = center_error(cardinals)
        u, v = to_px(-ey, ex)
        cv2.arrowedLine(img, (cx, cy), (u, v), (80, 255, 80), 2, tipLength=0.2)

    # Robot marker + forward arrow (forward = up).
    cv2.circle(img, (cx, cy), 7, (0, 80, 255), -1)
    cv2.arrowedLine(img, (cx, cy), (cx, cy - 28), (0, 80, 255), 2, tipLength=0.35)

    if heading_err is not None:
        cv2.putText(img, f"yaw err: {heading_err:+.1f} deg", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1, cv2.LINE_AA)
    if cardinals is not None:
        _, _, dist = center_error(cardinals)
        cv2.putText(img, f"dist to centre: {dist:.3f} m", (8, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1, cv2.LINE_AA)

    return img