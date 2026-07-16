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
import collections
import datetime
import os
import subprocess
import sys
import threading
import time

WORKER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sidebyside_worker.py")

# reframe_worker prints this as its final log line.
REFRAME_DONE_SENTINEL = "reframe_worker done."

# How long a job may stay blocked before we force-dispatch it anyway, so a
# stuck/never-finishing reframe can never starve a job forever.
JOB_BLOCK_DEADLINE_S = 1200.0

# When every remaining job is blocked, workers sleep this long before retrying.
ALL_BLOCKED_SLEEP_S = 2.0


def reframe_state(trial_dir):
    """Return 'done', 'blocked' or 'absent' for a trial folder.

    - 'absent' : no reframe.log -> reframe not running/disabled -> safe to go.
    - 'blocked': reframe.log exists but no done-sentinel yet -> still rewriting.
    - 'done'   : reframe.log contains the done-sentinel.
    """
    log = os.path.join(trial_dir, "reframe.log")
    if not os.path.exists(log):
        return "absent"
    try:
        with open(log, "r") as f:
            return "done" if REFRAME_DONE_SENTINEL in f.read() else "blocked"
    except OSError:
        return "blocked"


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
    reframe_log = os.path.join(trial_dir, "reframe.log")
    cmd = ["nice", "-n", "15", "ionice", "-c", "3",
           sys.executable, WORKER, room, foma, out,
           "--log", log, "--reframe-log", reframe_log,
           "--room-cam-shape", room_cam_shape]
    rc = subprocess.call(cmd)
    return trial_dir, rc


def worker_loop(wid, jobs, lock, deadlines, room_cam_shape, results):
    """Pop a job; if its videos are still being rewritten by reframe_worker
    (blocked) put it back at the END of the pool and try a different job.
    Only idle-sleep when every remaining job is blocked."""
    while True:
        trial = None
        all_blocked = False
        with lock:
            if not jobs:
                return  # pool drained
            for _ in range(len(jobs)):
                cand = jobs.popleft()
                if reframe_state(cand) == "blocked" and time.time() < deadlines.get(cand, 0.0):
                    jobs.append(cand)  # send to back, try another
                    continue
                trial = cand
                break
            else:
                all_blocked = True  # nothing ready right now

        if trial is None:
            if all_blocked:
                time.sleep(ALL_BLOCKED_SLEEP_S)
            continue

        trial_dir, rc = build_one(trial, room_cam_shape)
        print(f'batch: {"OK" if rc == 0 else f"FAIL({rc})"} {trial_dir}', flush=True)
        with lock:
            results.append(rc)


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

    nworkers = max(1, min(args.jobs, len(trials)))
    jobs_q = collections.deque(trials)
    deadlines = {t: time.time() + JOB_BLOCK_DEADLINE_S for t in trials}
    lock = threading.Lock()
    results = []

    threads = [threading.Thread(target=worker_loop,
                                args=(i, jobs_q, lock, deadlines, args.room_cam_shape, results),
                                daemon=True)
               for i in range(nworkers)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    failures = sum(1 for rc in results if rc != 0)
    print(f'batch: done. built={len(results) - failures} failed={failures}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
