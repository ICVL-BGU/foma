#!/usr/bin/env python3
"""Drift metrics from a rosbag (drift compensation, stage 0). Read-only.

Run on a machine with ROS Noetic (needs the `rosbag` Python module):
  python3 analyze_bag.py baseline_axes.bag
  python3 analyze_bag.py baseline_axes.bag --csv out.csv --export-dir export/

Per segment (from drift_test/segment markers written by test_patterns.py, or
cut from motor_control/twist when no markers exist) it reports:
  * encoders  : step changes per wheel and the derived body components
                (see drift_metrics.encoder_pair_metrics), the speed ratio of
                the driven pair and how much the "idle" pair turned.
  * LiDAR     : heading change in degrees (CCW +) from the folded wall angle,
                and the wall angle at the start (0 = square to the room).
  * ceiling   : distance travelled, drift angle vs. the command, sideways
                drift, and the largest deviation from a straight line.

All times are bag receive times on the recording machine, so Pi/station
clock offset does not matter. The estimated offset is printed for reference.
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import drift_metrics as dm  # noqa: E402

T_SEGMENT = '/drift_test/segment'
T_WHEEL = '/motor_control/wheel_state'
T_SCAN = '/lidar/scan_full'
T_LOC = '/localization/location'
T_TWIST = '/motor_control/twist'


def load(bag_path):
    import rosbag
    data = {'markers': [], 'wheel_t': [], 'wheel_pos': [], 'wheel_vel': [],
            'wheel_eff': [], 'wheel_offset': [], 'scan_t': [], 'wall': [],
            'loc_t': [], 'loc_xy': [], 'twist_t': [], 'twist': []}
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages(topics=[T_SEGMENT, T_WHEEL, T_SCAN, T_LOC, T_TWIST]):
            ts = t.to_sec()
            if topic == T_SEGMENT:
                try:
                    m = json.loads(msg.data)
                    m['t'] = ts
                    data['markers'].append(m)
                except ValueError:
                    pass
            elif topic == T_WHEEL:
                data['wheel_t'].append(ts)
                data['wheel_pos'].append(list(msg.position))
                data['wheel_vel'].append(list(msg.velocity))
                data['wheel_eff'].append(list(msg.effort))
                data['wheel_offset'].append(ts - msg.header.stamp.to_sec())
            elif topic == T_SCAN:
                data['scan_t'].append(ts)
                data['wall'].append(dm.wall_angle_deg(np.asarray(msg.ranges, dtype=float),
                                                      msg.angle_min, msg.angle_increment))
            elif topic == T_LOC:
                data['loc_t'].append(ts)
                data['loc_xy'].append((msg.world.x, msg.world.y))
            elif topic == T_TWIST:
                data['twist_t'].append(ts)
                data['twist'].append((msg.linear.x, msg.linear.y, msg.angular.z))
    for k in ('wheel_t', 'wheel_offset', 'scan_t', 'loc_t', 'twist_t'):
        data[k] = np.asarray(data[k], dtype=float)
    for k in ('wheel_pos', 'wheel_vel', 'wheel_eff', 'loc_xy', 'twist'):
        data[k] = np.asarray(data[k], dtype=float)
    return data


def segments_from_markers(markers):
    segs, open_ = [], {}
    for m in markers:
        if m.get('event') == 'start':
            open_[m.get('index')] = m
        elif m.get('event') == 'end' and m.get('index') in open_:
            s = open_.pop(m.get('index'))
            segs.append(dict(s, t0=s['t'], t1=m['t']))
    return segs


def segments_from_twist(t, tw, gap=0.3):
    """Fallback: contiguous non-zero commands on motor_control/twist."""
    segs = []
    if len(t) == 0:
        return segs
    moving = np.abs(tw).max(axis=1) > 1e-6
    start = None
    last = None
    for ti, mv, cmd in zip(t, moving, tw):
        if mv and start is None:
            start, first = ti, cmd
        if mv:
            last = ti
        if start is not None and not mv and ti - last > gap:
            segs.append(_twist_seg(len(segs), start, last, first))
            start = None
    if start is not None:
        segs.append(_twist_seg(len(segs), start, last, first))
    return segs


def _twist_seg(i, t0, t1, cmd):
    x, y, z = cmd
    if abs(z) > 1e-6 and abs(x) < 1e-6 and abs(y) < 1e-6:
        return dict(index=i, label='cw' if z > 0 else 'ccw', kind='rotate',
                    angle_deg=None, speed=abs(z), t0=t0, t1=t1)
    ang = math.degrees(math.atan2(-x, y)) % 360.0      # inverse of the GUI mapping
    return dict(index=i, label=f'{ang:.0f}deg', kind='translate',
                angle_deg=ang, speed=math.hypot(x, y), t0=t0, t1=t1)


def _nearest(t_arr, t):
    if len(t_arr) == 0:
        return None
    return int(np.argmin(np.abs(t_arr - t)))


def _window(t_arr, a, b):
    return np.flatnonzero((t_arr >= a) & (t_arr <= b))


def analyse_segment(seg, d, heading_t, heading, settle, floor_mm):
    t0, t1 = seg['t0'], seg['t1']
    te = t1 + settle
    row = {k: seg.get(k) for k in ('index', 'repeat', 'label', 'kind', 'angle_deg', 'speed')}
    row['duration_s'] = t1 - t0

    # --- encoders ---------------------------------------------------------
    i0, i1 = _nearest(d['wheel_t'], t0), _nearest(d['wheel_t'], te)
    if i0 is not None and i1 is not None and i1 > i0:
        dl, dr, dt_, db = (d['wheel_pos'][i1] - d['wheel_pos'][i0]).tolist()
        row.update(d_left=dl, d_right=dr, d_top=dt_, d_bottom=db)
        row.update(dm.encoder_pair_metrics(dl, dr, dt_, db))
        if seg.get('kind') == 'translate' and seg.get('angle_deg') is not None:
            axis = dm.driven_axis(seg['angle_deg'])
            if axis == 'h':
                row['driven_ratio'] = dm.safe_ratio(abs(dl), abs(dr))
                row['idle_over_driven'] = dm.safe_ratio(row['u_v'], row['u_h'])
            elif axis == 'v':
                row['driven_ratio'] = dm.safe_ratio(abs(dt_), abs(db))
                row['idle_over_driven'] = dm.safe_ratio(row['u_h'], row['u_v'])
        win = _window(d['wheel_t'], t0, t1)
        if win.size:
            row['peak_duty'] = float(np.abs(d['wheel_eff'][win]).max())

    # --- LiDAR heading ----------------------------------------------------
    if len(heading_t):
        before = np.flatnonzero(heading_t <= t0)
        after = _window(heading_t, te, te + 0.6)
        if before.size and after.size:
            row['dyaw_deg'] = float(np.median(heading[after]) - heading[before[-1]])
    wall_before = [w for ts, w in zip(d['scan_t'], d['wall']) if ts <= t0 and w is not None]
    if wall_before:
        row['start_wall_deg'] = wall_before[-1]

    # --- ceiling camera -----------------------------------------------------
    if len(d['loc_t']):
        a = _window(d['loc_t'], t0 - 0.6, t0)
        b = _window(d['loc_t'], te, te + 0.6)
        path = _window(d['loc_t'], t0, te)
        if a.size and b.size:
            xy = d['loc_xy'] * floor_mm
            start = np.median(xy[a], axis=0)
            end = np.median(xy[b], axis=0)
            pts = np.vstack([start, xy[path], end]) if path.size else np.vstack([start, end])
            cm = dm.chord_metrics(pts)
            if cm is not None:
                row['dist_mm'] = cm['distance_mm']
                row['max_dev_mm'] = cm['max_dev_mm']
                if seg.get('kind') == 'translate' and seg.get('angle_deg') is not None \
                        and cm['distance_mm'] > 1.0:
                    drift = dm.drift_angle_deg(cm['chord_angle_deg'], seg['angle_deg'])
                    row['drift_deg'] = drift
                    row['lateral_mm'] = cm['distance_mm'] * math.sin(math.radians(drift))
                    row['lateral_mm_per_m'] = 1000.0 * math.sin(math.radians(drift))
                    if 'dyaw_deg' in row:
                        row['dyaw_deg_per_m'] = row['dyaw_deg'] / (cm['distance_mm'] / 1000.0)
                    if 'u_h' in row:
                        axis = dm.driven_axis(seg['angle_deg'])
                        driven = abs(row['u_h']) if axis == 'h' else abs(row['u_v']) if axis == 'v' \
                            else math.hypot(row['u_h'], row['u_v'])
                        row['mm_per_step'] = dm.safe_ratio(cm['distance_mm'], driven)
    return row


COLUMNS = ['index', 'repeat', 'label', 'kind', 'angle_deg', 'speed', 'duration_s',
           'd_left', 'd_right', 'd_top', 'd_bottom', 'u_h', 'u_v', 'rot_lr', 'rot_tb',
           'driven_ratio', 'idle_over_driven', 'peak_duty', 'start_wall_deg', 'dyaw_deg',
           'dist_mm', 'drift_deg', 'lateral_mm', 'lateral_mm_per_m', 'max_dev_mm',
           'dyaw_deg_per_m', 'mm_per_step']

SHOW = [('label', 10, '{}'), ('dist_mm', 8, '{:.0f}'), ('drift_deg', 7, '{:+.1f}'),
        ('lateral_mm', 8, '{:+.0f}'), ('max_dev_mm', 8, '{:+.0f}'), ('dyaw_deg', 7, '{:+.1f}'),
        ('driven_ratio', 7, '{:.3f}'), ('idle_over_driven', 8, '{:+.3f}'),
        ('rot_lr', 7, '{:+.0f}'), ('rot_tb', 7, '{:+.0f}'), ('mm_per_step', 7, '{:.3f}')]


def _fmt(v, f):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return '-'
    try:
        return f.format(v)
    except (ValueError, TypeError):
        return str(v)


def print_table(rows):
    print('  '.join(name.rjust(w) for name, w, _ in SHOW))
    for r in rows:
        print('  '.join(_fmt(r.get(name), f).rjust(w) for name, w, f in SHOW))


def print_summary(rows):
    labels = []
    for r in rows:
        if r.get('label') not in labels:
            labels.append(r.get('label'))
    keys = ['drift_deg', 'lateral_mm_per_m', 'dyaw_deg', 'dyaw_deg_per_m', 'driven_ratio', 'idle_over_driven']
    print('\nPer direction: mean +/- std (n)')
    for lab in labels:
        group = [r for r in rows if r.get('label') == lab]
        parts = []
        for k in keys:
            vals = np.array([r[k] for r in group if isinstance(r.get(k), float) and not math.isnan(r[k])])
            if vals.size:
                parts.append(f"{k}={vals.mean():+.3f}+/-{vals.std():.3f}({vals.size})")
        print(f"  {lab}: " + '  '.join(parts))


def export(d, heading_t, heading, segs, out_dir):
    os.makedirs(out_dir, exist_ok=True)

    def write(name, header, rows):
        with open(os.path.join(out_dir, name), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(rows)

    write('wheel_state.csv',
          ['t', 'pos_l', 'pos_r', 'pos_t', 'pos_b', 'vel_l', 'vel_r', 'vel_t', 'vel_b',
           'duty_l', 'duty_r', 'duty_t', 'duty_b'],
          [[t] + list(p) + list(v) + list(e) for t, p, v, e in
           zip(d['wheel_t'], d['wheel_pos'], d['wheel_vel'], d['wheel_eff'])])
    write('ceiling.csv', ['t', 'x_norm', 'y_norm'],
          [[t, x, y] for t, (x, y) in zip(d['loc_t'], d['loc_xy'])])
    write('lidar_heading.csv', ['t', 'yaw_deg'], list(zip(heading_t, heading)))
    write('lidar_wall.csv', ['t', 'wall_deg'],
          [[t, '' if w is None else w] for t, w in zip(d['scan_t'], d['wall'])])
    write('twist.csv', ['t', 'linear_x', 'linear_y', 'angular_z'],
          [[t] + list(c) for t, c in zip(d['twist_t'], d['twist'])])
    write('segments.csv', ['index', 'label', 'kind', 'angle_deg', 'speed', 't0', 't1'],
          [[s.get('index'), s.get('label'), s.get('kind'), s.get('angle_deg'), s.get('speed'),
            s['t0'], s['t1']] for s in segs])


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('bag')
    p.add_argument('--csv', help="write per-segment results to this CSV")
    p.add_argument('--export-dir', help="also dump compact raw CSVs here (to share for analysis)")
    p.add_argument('--settle', type=float, default=1.2,
                   help="seconds after the command ends before measuring the end pose")
    p.add_argument('--floor-mm', type=float, default=5000.0,
                   help="ROOM_FLOOR_MAP_SHAPE in mm (localization/location is normalised by it)")
    a = p.parse_args(argv)

    d = load(a.bag)
    print(f"wheel_state: {len(d['wheel_t'])} msgs, scan_full: {len(d['scan_t'])} msgs "
          f"({sum(w is not None for w in d['wall'])} usable), localization: {len(d['loc_t'])} msgs")
    if len(d['scan_t']) > 1:
        print(f"scan_full rate: {1.0 / np.median(np.diff(d['scan_t'])):.2f} Hz")
    if len(d['wheel_t']) > 1:
        print(f"wheel_state rate: {1.0 / np.median(np.diff(d['wheel_t'])):.1f} Hz, "
              f"Pi->recorder clock offset ~ {np.median(d['wheel_offset']):+.3f} s")

    segs = segments_from_markers(d['markers'])
    if not segs:
        print("No drift_test/segment markers; cutting segments from motor_control/twist.")
        segs = segments_from_twist(d['twist_t'], d['twist'])
    heading_t, heading = dm.track_heading(d['scan_t'], d['wall'])
    rows = [analyse_segment(s, d, heading_t, heading, a.settle, a.floor_mm) for s in segs]

    print(f"\n{len(rows)} segments. Units: mm, deg (yaw CCW +), encoder steps.")
    print_table(rows)
    print_summary(rows)

    if a.csv:
        with open(a.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=COLUMNS, extrasaction='ignore')
            w.writeheader()
            w.writerows(rows)
        print(f"\nPer-segment CSV: {a.csv}")
    if a.export_dir:
        export(d, heading_t, heading, segs, a.export_dir)
        print(f"Raw CSVs: {a.export_dir}")


if __name__ == '__main__':
    main()
