#!/usr/bin/env python3
"""Detached worker: builds a side-by-side video from the room and foma trial
videos using ffmpeg. The smaller-height video is stretched to match the larger
one (no letterboxing); widths are preserved per input.

Invocation:
    sidebyside_worker.py ROOM FOMA OUTPUT [--log LOG]

Run via subprocess.Popen with start_new_session=True so it survives the
caller's lifetime, matching the reframe_worker pattern.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time


def probe_stream(path):
    out = subprocess.check_output([
        'ffprobe', '-v', 'error', '-select_streams', 'v:0',
        '-show_entries', 'stream=height,avg_frame_rate,r_frame_rate',
        '-of', 'json', path,
    ])
    s = json.loads(out)['streams'][0]
    h = int(s['height'])
    rate = s.get('avg_frame_rate') or s.get('r_frame_rate') or '0/1'
    num, _, den = rate.partition('/')
    fps = (float(num) / float(den)) if den and float(den) != 0 else float(num or 0)
    if fps <= 0:
        fps = 25.0  # safe fallback
    return h, fps


def wait_for_file(path, timeout_s=10.0):
    """Writers may still be flushing the moot file when worker spawns;
    wait briefly until the file appears and stops growing."""
    end = time.time() + timeout_s
    last = -1
    while time.time() < end:
        if os.path.exists(path):
            size = os.path.getsize(path)
            if size > 0 and size == last:
                return True
            last = size
        time.sleep(0.5)
    return os.path.exists(path) and os.path.getsize(path) > 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('room')
    ap.add_argument('foma')
    ap.add_argument('output')
    ap.add_argument('--log', default=None)
    args = ap.parse_args()

    if args.log:
        # Redirect stdout/stderr to log file (caller may already have done so)
        sys.stdout = open(args.log, 'a', buffering=1)
        sys.stderr = sys.stdout

    if shutil.which('ffmpeg') is None or shutil.which('ffprobe') is None:
        print('ffmpeg/ffprobe not found on PATH; aborting.')
        return 2

    for p in (args.room, args.foma):
        if not wait_for_file(p):
            print(f'Input missing or empty: {p}')
            return 2

    try:
        h_room, fps_room = probe_stream(args.room)
        h_foma, fps_foma = probe_stream(args.foma)
    except Exception as e:
        print(f'ffprobe failed: {e}')
        return 2

    H = max(h_room, h_foma)
    if H % 2:
        H += 1  # libx264 requires even dims

    # Inputs may have different fps; normalize both to the higher rate so
    # hstack frames align in time (lower-fps stream is frame-duplicated).
    F = max(fps_room, fps_foma)

    filter_complex = (
        f'[0:v]fps={F},scale=trunc(oh*a/2)*2:{H},setsar=1[a];'
        f'[1:v]fps={F},scale=trunc(oh*a/2)*2:{H},setsar=1[b];'
        f'[a][b]hstack=inputs=2[v]'
    )

    def build_cmd(use_nvenc):
        """h264_cuvid decodes on the GPU; h264_nvenc encodes. Filter still
        runs on CPU (frames downloaded between decode and encode) because
        scale_npp is gated behind a separate ffmpeg build flag. Even so,
        encode+decode on GPU is the dominant cost."""
        base = ['ffmpeg', '-y']
        if use_nvenc:
            base += ['-hwaccel', 'cuda', '-c:v', 'h264_cuvid', '-i', args.room,
                     '-hwaccel', 'cuda', '-c:v', 'h264_cuvid', '-i', args.foma]
        else:
            base += ['-i', args.room, '-i', args.foma]
        base += [
            '-filter_complex', filter_complex,
            '-map', '[v]',
            '-c:v', 'h264_nvenc' if use_nvenc else 'libx264',
            '-preset', 'fast' if use_nvenc else 'veryfast',
            '-pix_fmt', 'yuv420p',
            '-r', str(F),
            '-vsync', 'cfr',
            '-an',
            args.output,
        ]
        return base

    # Try NVENC first; on failure (exit != 0), fall back to libx264.
    cmd = build_cmd(True)
    print('Running (NVENC):', ' '.join(cmd))
    rc = subprocess.call(cmd)
    if rc != 0:
        print(f'NVENC exit code {rc}; retrying with libx264.')
        cmd = build_cmd(False)
        print('Running (CPU):', ' '.join(cmd))
        rc = subprocess.call(cmd)
    print(f'ffmpeg exit code: {rc}')
    return rc


if __name__ == '__main__':
    sys.exit(main())
