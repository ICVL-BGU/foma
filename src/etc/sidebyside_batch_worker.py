#!/usr/bin/env python3
"""Detached batch builder for side-by-side videos.

Spawned once at GUI close instead of running sidebyside_worker after every
trial (which thrashes the machine when several trials finish close together).

It scans TRIAL_ROOT for *today's* session folders, finds every trial dir that
has both room_video.mp4 and foma_video.mp4 but no side_by_side.mp4 yet, and
builds them with a bounded pool of N concurrent sidebyside_worker processes.

Invocation:
    sidebyside_batch_worker.py TRIAL_ROOT [--date YYYYMMDD] [--jobs N]
                               [--room-cam-shape WxH] [--force] [--log LOG]

Run via subprocess.Popen(start_new_session=True) so it outlives the GUI.
"""
import argparse
import datetime
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed

WORKER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sidebyside_worker.py")


def find_trials(root, date_prefix, force):
    """Yield trial dirs (under today's sessions) that need a side-by-side."""
    if not os.path.isdir(root):
        return
    for session in sorted(os.listdir(root)):
        if not session.startswith(date_prefix):
            continue
        spath = os.path.join(root, session)
        if not os.path.isdir(spath):
            continue
        # Trials may sit directly in the session dir or one level down.
        for dirpath, _dirs, files in os.walk(spath):
            if 'room_video.mp4' in files and 'foma_video.mp4' in files:
                if not force and os.path.isfile(os.path.join(dirpath, 'side_by_side.mp4')):
                    continue
                yield dirpath


def build_one(trial_dir, room_cam_shape):
    room = os.path.join(trial_dir, "room_video.mp4")
    foma = os.path.join(trial_dir, "foma_video.mp4")
    out = os.path.join(trial_dir, "side_by_side.mp4")
    log = os.path.join(trial_dir, "sidebyside.log")
    cmd = ["nice", "-n", "15", "ionice", "-c", "3",
           sys.executable, WORKER, room, foma, out,
           "--log", log, "--room-cam-shape", room_cam_shape]
    rc = subprocess.call(cmd)
    return trial_dir, rc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('root')
    ap.add_argument('--date', default=None, help='YYYYMMDD; default today')
    ap.add_argument('--jobs', type=int, default=2, help='concurrent side-by-side builds')
    ap.add_argument('--room-cam-shape', default='1280x1280')
    ap.add_argument('--force', action='store_true', help='rebuild even if side_by_side.mp4 exists')
    ap.add_argument('--log', default=None)
    args = ap.parse_args()

    if args.log:
        sys.stdout = open(args.log, 'a', buffering=1)
        sys.stderr = sys.stdout

    date_prefix = args.date or datetime.date.today().strftime('%Y%m%d')
    trials = list(find_trials(args.root, date_prefix, args.force))
    print(f'batch: date={date_prefix} jobs={args.jobs} pending={len(trials)}')
    for t in trials:
        print(f'  queued: {t}')
    if not trials:
        print('batch: nothing to build; done.')
        return 0

    jobs = max(1, args.jobs)
    failures = 0
    with ThreadPoolExecutor(max_workers=jobs) as ex:
        futs = {ex.submit(build_one, t, args.room_cam_shape): t for t in trials}
        for fut in as_completed(futs):
            trial_dir, rc = fut.result()
            print(f'batch: {"OK" if rc == 0 else f"FAIL({rc})"} {trial_dir}')
            failures += (rc != 0)
    print(f'batch: done. built={len(trials) - failures} failed={failures}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
