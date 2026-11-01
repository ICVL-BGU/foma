"""Pure drift-measurement maths (drift compensation, stage 0).

No ROS, no hardware: every function works on plain numbers / numpy arrays so
it is unit-tested off-robot (test/test_drift_metrics.py) and reused by
analyze_bag.py on the station PC. Python 3.8 compatible (ROS Noetic).

Conventions
-----------
* Body angles: degrees, counter-clockwise, 0 = Forward, 90 = Left,
  180 = Backward, 270 = Right (same as lidar/scans and the GUI buttons).
* Wheel order: (left, right, top, bottom), motor-sign space of
  MotorControl.py, where translation forward drives left = +h, right = -h,
  sideways drives top = +v, bottom = -v, and rotation drives all four the same.
"""

import math

import numpy as np


# --------------------------------------------------------------------------- #
#  Angles                                                                      #
# --------------------------------------------------------------------------- #
def fold_deg(angle_deg):
    """Fold an orientation into [-45, 45): directions 90 degrees apart collapse
    together, which is what a square room can actually tell us."""
    return ((np.asarray(angle_deg, dtype=float) + 45.0) % 90.0) - 45.0


def unwrap_folded_delta(delta_deg):
    """Bring a difference of two folded angles into (-45, 45]."""
    d = float(delta_deg)
    while d > 45.0:
        d -= 90.0
    while d <= -45.0:
        d += 90.0
    return d


def _folded_circular_center(folded_deg):
    """Robust centre of folded (period-90) angles: circular mean on the
    4x-angle circle, then the median of the residuals around it."""
    a = np.radians(np.asarray(folded_deg, dtype=float) * 4.0)
    centre = math.degrees(math.atan2(np.mean(np.sin(a)), np.mean(np.cos(a)))) / 4.0
    resid = fold_deg(np.asarray(folded_deg, dtype=float) - centre)
    return float(fold_deg(centre + float(np.median(resid))))


# --------------------------------------------------------------------------- #
#  LiDAR wall orientation                                                      #
# --------------------------------------------------------------------------- #
def wall_angle_deg(ranges_m, angle_min_rad, angle_inc_rad, r_min=0.15,
                   r_max=12.0, stride=4, max_gap_m=0.5, max_bin_gap=8,
                   min_segments=20):
    """Orientation of the room walls in the body frame, folded to [-45, 45).

    Joins each valid return to the valid return ``stride`` places later,
    discards joins that jump across a gap or corner, folds every join's
    direction and takes a robust circular centre. None if the scan is unusable.
    Heading CHANGES are what matter: a square room hides multiples of 90 deg.
    """
    r = np.asarray(ranges_m, dtype=float)
    n = r.size
    if n == 0:
        return None
    idx = np.flatnonzero(np.isfinite(r) & (r > r_min) & (r < r_max))
    if idx.size < stride + min_segments:
        return None
    ang = angle_min_rad + angle_inc_rad * idx
    x = r[idx] * np.cos(ang)
    y = r[idx] * np.sin(ang)
    j = (np.arange(idx.size) + stride) % idx.size
    bin_gap = (idx[j] - idx) % n
    dx, dy = x[j] - x, y[j] - y
    seg = np.hypot(dx, dy)
    ok = (bin_gap <= max_bin_gap * stride) & (seg > 1e-4) & (seg <= max_gap_m)
    if np.count_nonzero(ok) < min_segments:
        return None
    folded = fold_deg(np.degrees(np.arctan2(dy[ok], dx[ok])))
    return _folded_circular_center(folded)


def track_heading(times, wall_angles):
    """Integrate robot yaw (deg, CCW +, starts at 0) from successive folded
    wall angles. A robot turning CCW by d sees the walls turn by -d.
    Entries that are None are skipped (the previous valid scan is kept).
    Valid only while the robot turns < 45 deg between two valid scans.
    Returns (times_out, yaw_out) for the valid scans."""
    t_out, yaw_out = [], []
    prev = None
    yaw = 0.0
    for t, w in zip(times, wall_angles):
        if w is None:
            continue
        if prev is not None:
            yaw -= unwrap_folded_delta(w - prev)
        prev = w
        t_out.append(t)
        yaw_out.append(yaw)
    return np.asarray(t_out, dtype=float), np.asarray(yaw_out, dtype=float)


# --------------------------------------------------------------------------- #
#  Ceiling-camera path                                                         #
# --------------------------------------------------------------------------- #
def chord_metrics(xy_mm):
    """Straightness of a path from its first to its last point.

    Returns dict: distance_mm, chord_angle_deg (map frame), max_dev_mm
    (signed perpendicular deviation with the largest magnitude, + = left of
    the chord in map coordinates). None if fewer than 2 points.
    """
    p = np.asarray(xy_mm, dtype=float)
    if p.ndim != 2 or p.shape[0] < 2:
        return None
    start, end = p[0], p[-1]
    d = end - start
    dist = float(np.hypot(d[0], d[1]))
    if dist < 1e-9:
        return {'distance_mm': 0.0, 'chord_angle_deg': float('nan'),
                'max_dev_mm': 0.0}
    u = d / dist
    rel = p - start
    perp = u[0] * rel[:, 1] - u[1] * rel[:, 0]
    k = int(np.argmax(np.abs(perp)))
    return {'distance_mm': dist,
            'chord_angle_deg': math.degrees(math.atan2(d[1], d[0])),
            'max_dev_mm': float(perp[k])}


def drift_angle_deg(chord_angle_deg, commanded_body_deg):
    """Angle between the actual travel direction and the commanded one,
    assuming the robot started aligned with the room axes (go_home align).

    The map-to-body rotation is a multiple of 90 deg (possibly mirrored), so
    only the folded difference is meaningful: 0 = travelled exactly along the
    command. The SIGN is consistent between runs of the same command, not
    between different commands.
    """
    return float(fold_deg(chord_angle_deg - (commanded_body_deg % 90.0)))


# --------------------------------------------------------------------------- #
#  Encoders                                                                    #
# --------------------------------------------------------------------------- #
def encoder_pair_metrics(d_left, d_right, d_top, d_bottom):
    """Body-motion components from encoder step changes (motor-sign space).

    u_h  : forward/backward translation seen by the left/right pair
    u_v  : sideways translation seen by the top/bottom pair
    rot_lr, rot_tb : rotation seen by each pair. With no slip and equal
           geometry they agree; their difference flags slip or mismatch.
    """
    return {
        'u_h': (d_left - d_right) / 2.0,
        'u_v': (d_top - d_bottom) / 2.0,
        'rot_lr': (d_left + d_right) / 2.0,
        'rot_tb': (d_top + d_bottom) / 2.0,
    }


def driven_axis(commanded_body_deg):
    """'h' (left/right pair), 'v' (top/bottom pair) or 'both' (diagonal)."""
    a = commanded_body_deg % 180.0
    if abs(a) < 1e-6 or abs(a - 180.0) < 1e-6:
        return 'h'
    if abs(a - 90.0) < 1e-6:
        return 'v'
    return 'both'


def safe_ratio(a, b):
    """a / b, or NaN when b is (almost) zero."""
    return float(a) / float(b) if abs(float(b)) > 1e-9 else float('nan')
