"""Ceiling-camera pixel <-> floor position, and the go_to target limits.

Shared by localization_node (robot position), go_home_node (go_to targets)
and gui_node (click on the room image).  Pixels are at the camera's native
resolution (ROOM_CAMERA_NATIVE_SHAPE), world points in floor-map mm.
"""

import json

import numpy as np


def _theta(r_d, model):
    """Map radial distance r_d to incidence angle theta per distortion model."""
    if model == 'equisolid':
        return 2 * np.arcsin(np.clip(r_d / 2, -1.0, 1.0))
    if model == 'equidistant':
        return r_d
    if model == 'stereographic':
        return 2 * np.arctan(r_d / 2)
    if model == 'orthographic':
        return np.arcsin(np.clip(r_d, -1.0, 1.0))
    raise ValueError(f"Unknown fisheye model: {model}")


def _r_d(theta, model):
    """Inverse of _theta: incidence angle -> radial distance."""
    if model == 'equisolid':
        return 2 * np.sin(theta / 2)
    if model == 'equidistant':
        return theta
    if model == 'stereographic':
        return 2 * np.tan(theta / 2)
    if model == 'orthographic':
        return np.sin(theta)
    raise ValueError(f"Unknown fisheye model: {model}")


def load_mappers(path):
    """The mapper JSON: one block per tag, 'lidar' and 'foma'."""
    with open(path, 'r') as f:
        return json.load(f)


def map_point(params, x, y):
    """Map image px (x, y) at 2560x2560 -> world (x_w, y_w) mm via one mapper.

    Two steps: fisheye undistort (model read from JSON, not hardcoded) then 3x3
    homography. `params` is one nested mapper block (undistorter + homography).
    """
    u = params['undistorter']
    cx, cy, f = u['cx'], u['cy'], u['f']

    x_d = (x - cx) / f
    y_d = (y - cy) / f
    r_d = np.sqrt(x_d ** 2 + y_d ** 2)

    theta = _theta(r_d, u['model'])
    scale = np.tan(theta) / r_d if r_d > 1e-9 else 1.0
    undistorted_x = x_d * scale * f + cx
    undistorted_y = y_d * scale * f + cy

    H = np.array(params['homography'], dtype=float)
    world_h = H @ np.array([undistorted_x, undistorted_y, 1.0])
    return world_h[0] / world_h[2], world_h[1] / world_h[2]


def unmap_point(params, x_w, y_w):
    """Inverse of map_point: world mm -> image px at native resolution."""
    u = params['undistorter']
    cx, cy, f = u['cx'], u['cy'], u['f']

    H = np.array(params['homography'], dtype=float)
    img_h = np.linalg.solve(H, np.array([x_w, y_w, 1.0]))
    x_u = (img_h[0] / img_h[2] - cx) / f
    y_u = (img_h[1] / img_h[2] - cy) / f
    r_u = np.sqrt(x_u ** 2 + y_u ** 2)

    scale = _r_d(np.arctan(r_u), u['model']) / r_u if r_u > 1e-9 else 1.0
    return x_u * scale * f + cx, y_u * scale * f + cy


def target_bounds(bounds_mm, floor_shape, robot_size_m, wall_margin_m):
    """Allowed area for the robot centre, normalised like
    localization/location.world: (x_min, x_max, y_min, y_max).

    bounds_mm = room walls in floor-map mm (x_min, x_max, y_min, y_max).
    The centre keeps half the robot plus a margin away from every wall.
    """
    keep_mm = (robot_size_m / 2.0 + wall_margin_m) * 1000.0
    x0, x1, y0, y1 = bounds_mm
    return ((x0 + keep_mm) / floor_shape[1], (x1 - keep_mm) / floor_shape[1],
            (y0 + keep_mm) / floor_shape[0], (y1 - keep_mm) / floor_shape[0])


def clamp_target(x, y, bounds):
    """Nearest allowed point -> (x, y, moved)."""
    x0, x1, y0, y1 = bounds
    cx = min(max(x, x0), x1)
    cy = min(max(y, y0), y1)
    return cx, cy, (cx, cy) != (x, y)
