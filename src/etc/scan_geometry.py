"""Pure LiDAR-scan maths shared by alignment, centering and rotation.

No ROS, no state, no hardware — every function is a pure function of a range
array, so it is testable without a robot.

Conventions (kept identical to the legacy go_home.py so closed-loop signs are
preserved): ``ranges[i]`` is the beam at ``i`` degrees; a point is
``(r*cos i, r*sin i)``. ``cardinal_distances`` samples 270/90/0/180, which in
motor_control_node's frame (0=Forward, 90=Left, 180=Backward, 270=Right) gives
``error_x = (d270-d90)/2 -> Twist.linear.x`` (+ = right) and
``error_y = (d0-d180)/2 -> Twist.linear.y`` (+ = forward).
``wall_heading_error`` is the room orientation folded to [-45, 45); 0 = square.
"""

import math

import numpy as np

#: Shared "not supplied" sentinel for optional args whose ``None`` value is
#: itself meaningful (no walls found / unusable scan). Must be *one* object:
#: these defaults are passed between modules, so a per-module sentinel would
#: not compare identical and would be mistaken for a real value.
UNSET = object()


# --------------------------------------------------------------------------- #
#  Beam sampling                                                               #
# --------------------------------------------------------------------------- #
def sample_distance(ranges, angle_deg, half_window=10, r_min=0.1, r_max=15.0):
    """Median distance in a +/-``half_window`` cone, or None if no valid beam."""
    ranges = np.asarray(ranges, dtype=float)
    n = len(ranges)
    idx = [(int(angle_deg) + i) % n for i in range(-half_window, half_window)]
    vals = ranges[idx]
    valid = vals[(vals > r_min) & (vals < r_max) & np.isfinite(vals)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def cardinal_distances(ranges, front=270, back=90, left=0, right=180):
    """``(front, back, left, right)`` in metres, or None if any is unusable."""
    f = sample_distance(ranges, front)
    b = sample_distance(ranges, back)
    l = sample_distance(ranges, left)
    r = sample_distance(ranges, right)
    if None in (f, b, l, r):
        return None
    return f, b, l, r


def center_error(cardinals):
    """``(error_x, error_y, distance)`` offset from the room centre."""
    f, b, l, r = cardinals
    error_x = (f - b) / 2.0
    error_y = (l - r) / 2.0
    return error_x, error_y, math.hypot(error_x, error_y)


# --------------------------------------------------------------------------- #
#  Command shaping                                                             #
# --------------------------------------------------------------------------- #
def apply_deadband(value, min_mag, max_mag):
    """Boost a non-zero command to at least ``min_mag`` (static friction),
    clip to ``max_mag``, leave exact zero alone."""
    if value == 0.0:
        return 0.0
    mag = min(max(abs(value), min_mag), max_mag)
    return math.copysign(mag, value)


def clamp_magnitude(vx, vy, min_mag, max_mag):
    """Clamp the *length* of a command vector, keeping its heading.

    Dead-banding each axis independently boosts a near-zero axis to the floor
    and skews the travel direction by up to 45 degrees — the legacy bug.
    """
    mag = math.hypot(vx, vy)
    if mag < 1e-9:
        return 0.0, 0.0
    scale = 1.0
    if mag < min_mag:
        scale = min_mag / mag
    elif mag > max_mag:
        scale = max_mag / mag
    return vx * scale, vy * scale


def slew_limit(value, previous, max_step):
    """Cap how far a command may move in one step — this is what stops a noisy
    measurement flipping the drive from one saturation rail to the other."""
    if max_step is None or max_step <= 0.0:
        return value
    return min(max(value, previous - max_step), previous + max_step)


# --------------------------------------------------------------------------- #
#  Wall-orientation estimation                                                 #
# --------------------------------------------------------------------------- #
def scan_to_points(ranges, r_min=0.15, r_max=12.0):
    """Convert a scan to ``(x, y, valid)`` arrays in the robot frame."""
    ranges = np.asarray(ranges, dtype=float)
    ang = np.deg2rad(np.arange(len(ranges)))
    return (ranges * np.cos(ang), ranges * np.sin(ang),
            (ranges > r_min) & (ranges < r_max))


def fold_angle(angle_deg):
    """Fold an orientation into [-45, 45). Wall directions 90 degrees apart
    collapse to one value, which is what makes the estimate corner-proof."""
    return ((angle_deg + 45.0) % 90.0) - 45.0


def unwrap_folded_delta(delta_deg):
    """Unwrap a difference of two folded angles into (-45, 45]."""
    while delta_deg > 45.0:
        delta_deg -= 90.0
    while delta_deg <= -45.0:
        delta_deg += 90.0
    return delta_deg


def wall_heading_error(ranges, r_min=0.15, r_max=12.0, max_gap=0.50,
                       min_segments=4):
    """Room yaw error in degrees, folded to [-45, 45), or None.

    Measures the orientation of every short wall segment, discards ones that
    leap across a corner/gap, folds each into [-45, 45) and takes the median.
    The median rejects the remaining corner segments, so this stays accurate at
    any yaw — including the 45-degree case that broke the fixed-sector fit.
    """
    x, y, valid = scan_to_points(ranges, r_min, r_max)
    n = len(x)
    folded = []
    for i in range(n):
        j = (i + 1) % n
        if not (valid[i] and valid[j]):
            continue
        dx, dy = x[j] - x[i], y[j] - y[i]
        seg = math.hypot(dx, dy)
        if seg < 1e-4 or seg > max_gap:
            continue
        folded.append(fold_angle(math.degrees(math.atan2(dy, dx))))
    if len(folded) < min_segments:
        return None
    return float(np.median(folded))


def scan_match_delta_deg(prev_ranges, ranges, max_shift=30, r_min=0.15,
                         r_max=12.0):
    """Yaw change between two scans by circular cross-correlation (degrees).

    Unlike the folded wall angle this is unambiguous over +/-``max_shift``, so
    it can arbitrate a folded delta that landed in the wrong 90-degree bucket.
    Positive = counter-clockwise. None if either scan is unusable.
    """
    a = np.asarray(prev_ranges, dtype=float)
    b = np.asarray(ranges, dtype=float)
    if a.shape != b.shape or a.size == 0:
        return None

    ok = ((a > r_min) & (a < r_max) & np.isfinite(a) &
          (b > r_min) & (b < r_max) & np.isfinite(b))
    if np.count_nonzero(ok) < a.size // 3:
        return None

    a = np.where(ok, a - a[ok].mean(), 0.0)
    b = np.where(ok, b - b[ok].mean(), 0.0)

    shifts = np.arange(-max_shift, max_shift + 1)
    scores = np.array([float(np.dot(a, np.roll(b, int(s)))) for s in shifts])
    k = int(np.argmax(scores))
    best = float(shifts[k])

    # Parabolic refinement of the discrete peak, for sub-degree resolution.
    if 0 < k < len(scores) - 1:
        y0, y1, y2 = scores[k - 1], scores[k], scores[k + 1]
        denom = y0 - 2.0 * y1 + y2
        if abs(denom) > 1e-12:
            best += 0.5 * (y0 - y2) / denom
    return best


# --------------------------------------------------------------------------- #
#  Room-model rendering                                                        #
# --------------------------------------------------------------------------- #
def render_room_view(ranges, heading_err=None, cardinals=None,
                     size=480, scale=None, r_max=12.0):
    """Top-down BGR image of the room as the robot models it: LiDAR dots, the
    modelled walls, the robot with a heading arrow, the offset-to-centre
    vector. ``cv2`` is imported lazily so the maths stays testable without it.
    """
    import cv2

    img = np.full((size, size, 3), 20, dtype=np.uint8)
    cx = cy = size // 2

    x, y, valid = scan_to_points(ranges, r_max=r_max)
    if scale is None:
        span = float(np.max(np.hypot(x[valid], y[valid]))) if valid.any() else 1.0
        scale = (size * 0.42) / max(span, 0.5)

    def to_px(px_m, py_m):
        # image x right, image y down; robot forward (270 deg) = up
        return int(cx + px_m * scale), int(cy - py_m * scale)

    for pxm, pym in zip(x[valid], y[valid]):
        u, v = to_px(pxm, pym)
        if 0 <= u < size and 0 <= v < size:
            cv2.circle(img, (u, v), 2, (180, 200, 220), -1)

    if heading_err is not None and cardinals is not None:
        f, b, l, r = cardinals
        phi = math.radians(heading_err)
        ux, uy = math.cos(phi), math.sin(phi)       # front/back normal axis
        vx, vy = -math.sin(phi), math.cos(phi)      # left/right normal axis
        half = size
        for dist, normal, tangent in ((f, (ux, uy), (vx, vy)),
                                      (b, (-ux, -uy), (vx, vy)),
                                      (l, (vx, vy), (ux, uy)),
                                      (r, (-vx, -vy), (ux, uy))):
            mxm, mym = normal[0] * dist, normal[1] * dist
            p1 = to_px(mxm - tangent[0] * half, mym - tangent[1] * half)
            p2 = to_px(mxm + tangent[0] * half, mym + tangent[1] * half)
            cv2.line(img, p1, p2, (80, 180, 255), 2)

        # Centre lies at (-ey, ex) in the robot frame.
        ex, ey, dist = center_error(cardinals)
        cv2.arrowedLine(img, (cx, cy), to_px(-ey, ex), (80, 255, 80), 2,
                        tipLength=0.2)
        cv2.putText(img, f"dist to centre: {dist:.3f} m", (8, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1,
                    cv2.LINE_AA)

    cv2.circle(img, (cx, cy), 7, (0, 80, 255), -1)
    cv2.arrowedLine(img, (cx, cy), (cx, cy - 28), (0, 80, 255), 2, tipLength=0.35)
    if heading_err is not None:
        cv2.putText(img, f"yaw err: {heading_err:+.1f} deg", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1,
                    cv2.LINE_AA)
    return img
