"""One LiDAR revolution -> fixed-bin range array (drift compensation, stage 0).

Pure Python, no ROS, no hardware, so it can be tested off-robot.

Angle convention is the SAME as lidar/scans (lidar_node.py): body angle in
degrees, counter-clockwise, 0 = Forward, 90 = Left, 180 = Backward,
270 = Right, computed as ``359 - ((raw + offset) % 360)``. lidar/scans rounds
that to 360 one-degree bins; here the angle is kept continuous and binned
at ``360 / n_bins`` degrees.
"""

import math

SCAN_FULL_BINS = 720          # 0.5 degree bins
NO_RETURN = float('nan')      # bin with no measurement in this revolution


def body_angle_deg(raw_angle_deg, lidar_offset_deg):
    """Raw sensor angle -> body angle in [0, 360), lidar/scans convention."""
    return (359.0 - ((raw_angle_deg + lidar_offset_deg) % 360.0)) % 360.0


def bin_revolution(samples, lidar_offset_deg, n_bins=SCAN_FULL_BINS):
    """Bin one revolution of ``(raw_angle_deg, distance_mm)`` samples.

    Returns a list of ``n_bins`` ranges in METRES (REP-103 units, unlike
    lidar/scans which is in mm). Bin ``i`` covers body angle
    ``i * 360 / n_bins``. Empty bins are NaN. When several samples fall in
    one bin the last one wins (samples arrive in time order).
    """
    ranges = [NO_RETURN] * n_bins
    res = 360.0 / n_bins
    for raw, dist_mm in samples:
        idx = int(round(body_angle_deg(raw, lidar_offset_deg) / res)) % n_bins
        ranges[idx] = dist_mm / 1000.0
    return ranges


def angle_increment_rad(n_bins=SCAN_FULL_BINS):
    return 2.0 * math.pi / n_bins
