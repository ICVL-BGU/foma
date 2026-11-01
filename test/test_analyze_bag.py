"""analyze_bag.py processing on a synthetic forward run (no rosbag needed)."""

import math
import unittest

import numpy as np

import _stubs

_stubs.install()
import analyze_bag as ab  # noqa: E402
import drift_metrics as dm  # noqa: E402
from test_drift_metrics import square_room_scan, INC  # noqa: E402


def synthetic_run(drift_deg=2.0, yaw_total_deg=-1.5):
    t0, t1, settle = 10.0, 12.0, 1.2
    wt = np.arange(8.0, 16.0, 0.02)
    moving = np.clip((wt - t0), 0, t1 + 0.7 - t0)          # steps keep coming during ramp-down
    left, right = 100.0 * moving, -95.0 * moving
    top, bottom = 2.0 * moving, -2.0 * moving
    pos = np.c_[left, right, top, bottom]
    d = {
        'markers': [], 'wheel_t': wt, 'wheel_pos': pos, 'wheel_vel': np.zeros_like(pos),
        'wheel_eff': np.where((wt >= t0) & (wt <= t1), 127.0, 0.0)[:, None] * np.array([1, -1, 0, 0]),
        'wheel_offset': np.zeros_like(wt),
    }
    # ceiling: 600 mm along map +x with a drift angle, normalised by 5000 mm
    lt = np.arange(8.0, 16.0, 0.04)
    frac = np.clip((lt - t0) / (t1 + 0.7 - t0), 0, 1)
    a = math.radians(drift_deg)
    xy_mm = np.c_[2500 + 600 * frac * math.cos(a), 2500 + 600 * frac * math.sin(a)]
    d['loc_t'], d['loc_xy'] = lt, xy_mm / 5000.0
    # LiDAR: yaw ramps during the run
    st = np.arange(8.0, 16.0, 0.18)
    yaw = yaw_total_deg * np.clip((st - t0) / (t1 + 0.7 - t0), 0, 1)
    d['scan_t'] = st
    d['wall'] = [dm.wall_angle_deg(square_room_scan(yaw_deg=y, noise_m=0.005), 0.0, INC) for y in yaw]
    d['twist_t'] = np.arange(t0, t1, 0.05)
    d['twist'] = np.tile([0.0, 1.0, 0.0], (len(d['twist_t']), 1))
    seg = dict(index=0, repeat=0, label='forward', kind='translate', angle_deg=0, speed=1.0,
               t0=t0, t1=t1)
    return d, seg, settle


class AnalyzeSegment(unittest.TestCase):
    def test_forward_run_metrics(self):
        d, seg, settle = synthetic_run()
        ht, hy = dm.track_heading(d['scan_t'], d['wall'])
        row = ab.analyse_segment(seg, d, ht, hy, settle, 5000.0)
        self.assertAlmostEqual(row['dist_mm'], 600.0, delta=2.0)
        self.assertAlmostEqual(row['drift_deg'], 2.0, delta=0.1)
        self.assertAlmostEqual(row['lateral_mm'], 600 * math.sin(math.radians(2.0)), delta=1.0)
        self.assertAlmostEqual(row['dyaw_deg'], -1.5, delta=0.4)
        self.assertAlmostEqual(row['driven_ratio'], 100.0 / 95.0, delta=0.01)
        self.assertAlmostEqual(row['idle_over_driven'], 2.0 / 97.5, delta=0.002)
        self.assertAlmostEqual(row['rot_lr'], 2.5 * 2.7, delta=0.5)   # (100-95)/2 per s
        self.assertAlmostEqual(row['mm_per_step'], 600.0 / (97.5 * 2.7), delta=0.05)
        self.assertEqual(row['peak_duty'], 127.0)

    def test_segments_from_markers_and_twist(self):
        markers = [dict(event='session_start', t=0.0),
                   dict(event='start', index=0, label='forward', kind='translate', angle_deg=0, t=1.0),
                   dict(event='end', index=0, label='forward', kind='translate', angle_deg=0, t=3.0)]
        segs = ab.segments_from_markers(markers)
        self.assertEqual(len(segs), 1)
        self.assertEqual((segs[0]['t0'], segs[0]['t1']), (1.0, 3.0))

        t = np.arange(0.0, 10.0, 0.05)
        tw = np.zeros((len(t), 3))
        tw[(t >= 1) & (t < 3)] = [-1.0, 0.0, 0.0]      # GUI "left": x=-sin(90)
        tw[(t >= 5) & (t < 6)] = [0.0, 0.0, 1.0]       # GUI cw
        segs = ab.segments_from_twist(t, tw)
        self.assertEqual([s['label'] for s in segs], ['90deg', 'cw'])
        self.assertAlmostEqual(segs[0]['angle_deg'], 90.0)

    def test_print_does_not_crash_on_missing_fields(self):
        ab.print_table([{'label': 'x'}, {'label': 'y', 'dist_mm': float('nan')}])
        ab.print_summary([{'label': 'x', 'drift_deg': 1.0}, {'label': 'x', 'drift_deg': 3.0}])


if __name__ == '__main__':
    unittest.main()
