import math
import unittest

import numpy as np

import _stubs

_stubs.install()
import drift_metrics as dm  # noqa: E402

N = 720
INC = 2 * math.pi / N


def square_room_scan(x0=0.3, y0=-0.4, yaw_deg=0.0, half=2.5, noise_m=0.0, seed=0,
                     dropout=0.0):
    """Ranges (m) a robot at (x0, y0) with yaw sees in a 5x5 m room."""
    rng = np.random.default_rng(seed)
    yaw = math.radians(yaw_deg)
    out = np.full(N, np.nan)
    for i in range(N):
        a = yaw + i * INC
        dx, dy = math.cos(a), math.sin(a)
        ts = []
        for p, d in ((x0, dx), (y0, dy)):
            if abs(d) > 1e-12:
                for wall in (-half, half):
                    t = (wall - p) / d
                    if t > 0:
                        ts.append(t)
        r = min(ts)
        if rng.random() >= dropout:
            out[i] = r + rng.normal(0.0, noise_m)
    return out


class Angles(unittest.TestCase):
    def test_fold(self):
        for a, f in ((0, 0), (44, 44), (46, -44), (90, 0), (-91, -1), (135, -45), (-45, -45)):
            self.assertAlmostEqual(float(dm.fold_deg(a)), f)

    def test_unwrap(self):
        self.assertAlmostEqual(dm.unwrap_folded_delta(-44 - 44), 2.0)
        self.assertAlmostEqual(dm.unwrap_folded_delta(44 - (-44)), -2.0)


class WallAngle(unittest.TestCase):
    def test_recovers_yaw_any_angle(self):
        for yaw in (0.0, 3.0, -7.5, 20.0, 44.0, -44.0, 61.0):
            w = dm.wall_angle_deg(square_room_scan(yaw_deg=yaw, noise_m=0.01, dropout=0.3),
                                  0.0, INC)
            self.assertIsNotNone(w)
            # walls appear rotated by -yaw in the body frame
            err = dm.unwrap_folded_delta(w - float(dm.fold_deg(-yaw)))
            self.assertLess(abs(err), 0.5, (yaw, w))

    def test_empty_scan(self):
        self.assertIsNone(dm.wall_angle_deg(np.full(N, np.nan), 0.0, INC))

    def test_track_heading_through_many_quarter_turns(self):
        yaws = np.arange(0.0, 200.0, 7.0)          # CCW, 7 deg per scan
        walls = [dm.wall_angle_deg(square_room_scan(yaw_deg=y, noise_m=0.005), 0.0, INC)
                 for y in yaws]
        t, track = dm.track_heading(np.arange(len(yaws)), walls)
        self.assertEqual(len(track), len(yaws))
        self.assertLess(np.max(np.abs(track - yaws)), 1.0)


class Path(unittest.TestCase):
    def test_chord_and_drift(self):
        # 2 m along +x with a 3 deg drift to +y, bulging 20 mm
        L = 2000.0
        d = math.radians(3.0)
        s = np.linspace(0, 1, 50)
        pts = np.c_[L * s * math.cos(d), L * s * math.sin(d)] + \
            np.c_[np.zeros(50), 20 * np.sin(np.pi * s)]
        cm = dm.chord_metrics(pts)
        self.assertAlmostEqual(cm['distance_mm'], L, delta=1.0)
        self.assertAlmostEqual(cm['chord_angle_deg'], 3.0, delta=0.05)
        self.assertAlmostEqual(cm['max_dev_mm'], 20.0 * math.cos(d), delta=0.5)
        # forward command, map axis rotated 90 deg from body: still 3 deg drift
        self.assertAlmostEqual(dm.drift_angle_deg(93.0, 0), 3.0)
        # diagonal command
        self.assertAlmostEqual(dm.drift_angle_deg(-47.0, 315), -2.0)


class Encoders(unittest.TestCase):
    def test_pair_metrics(self):
        # pure forward: left +100, right -100 ; idle pair still
        m = dm.encoder_pair_metrics(100, -100, 0, 0)
        self.assertEqual((m['u_h'], m['u_v'], m['rot_lr'], m['rot_tb']), (100, 0, 0, 0))
        # pure rotation: all +50
        m = dm.encoder_pair_metrics(50, 50, 50, 50)
        self.assertEqual((m['u_h'], m['u_v'], m['rot_lr'], m['rot_tb']), (0, 0, 50, 50))

    def test_driven_axis(self):
        self.assertEqual(dm.driven_axis(0), 'h')
        self.assertEqual(dm.driven_axis(180), 'h')
        self.assertEqual(dm.driven_axis(90), 'v')
        self.assertEqual(dm.driven_axis(270), 'v')
        self.assertEqual(dm.driven_axis(45), 'both')


if __name__ == '__main__':
    unittest.main()
