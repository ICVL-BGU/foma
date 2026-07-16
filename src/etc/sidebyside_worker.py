#!/usr/bin/env python3
"""Detached worker: builds a side-by-side video from the room and foma trial
videos using ffmpeg. The smaller-height video is stretched to match the
larger one (no letterboxing); widths are preserved per input.

The foma video is first re-encoded into a temporary annotated copy with
fish-direction and FOMA-drive-direction overlays drawn from
fish_location.csv + foma_speed.csv (read from the foma video's own folder),
mirroring what the GUI draws live.

Invocation:
    sidebyside_worker.py ROOM FOMA OUTPUT [--swap-foma] [--swap-room] [--log LOG]

--swap-foma / --swap-room: swap R<->B channels of that input before encode,
used to fix videos recorded before the camera-node BGR/RGB fix.

Run via subprocess.Popen with start_new_session=True so it survives the
caller's lifetime, matching the reframe_worker pattern.
"""
import argparse
import bisect
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time

import cv2


# ----------------------------- helpers --------------------------------------

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
        fps = 25.0
    return h, fps


def wait_for_file(path, timeout_s=10.0):
    """Writers may still be flushing the moov atom when worker spawns; wait
    briefly until the file appears and stops growing."""
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


REFRAME_DONE_SENTINEL = "reframe_worker done."


def wait_for_reframe(log_path, appear_timeout_s=30.0, done_timeout_s=900.0):
    """Block until reframe_worker finishes rewriting the videos on disk.

    reframe_worker prints REFRAME_DONE_SENTINEL as its last log line. We wait
    for that line so we never read foma_video.mp4 mid-rewrite (it re-encodes
    the file in place).

    If the log never appears within appear_timeout_s we assume reframe is not
    running (disabled) and proceed. Returns True if it's safe to proceed.
    """
    if not log_path:
        return True
    # Wait for the log to appear (reframe spawns roughly when we do).
    end_appear = time.time() + appear_timeout_s
    while time.time() < end_appear:
        if os.path.exists(log_path):
            break
        time.sleep(0.5)
    if not os.path.exists(log_path):
        print(f'reframe log never appeared ({log_path}); assuming disabled, proceeding.')
        return True
    # Log exists; wait for the done sentinel.
    end_done = time.time() + done_timeout_s
    while time.time() < end_done:
        try:
            with open(log_path, 'r') as f:
                if REFRAME_DONE_SENTINEL in f.read():
                    print('reframe_worker finished; proceeding.')
                    return True
        except OSError:
            pass
        time.sleep(1.0)
    print(f'timed out waiting for reframe to finish ({log_path}); proceeding anyway.')
    return True


def _read_csv(path):
    rows = []
    if not os.path.isfile(path):
        return rows
    with open(path, newline='') as f:
        rdr = csv.reader(f)
        header = next(rdr, None)
        for r in rdr:
            try:
                rows.append([float(x) for x in r])
            except ValueError:
                continue  # skip malformed
    return rows


def _nearest(rows, times, t):
    """Return the row whose timestamp is closest to t. None if rows empty."""
    if not rows:
        return None
    i = bisect.bisect_left(times, t)
    if i == 0:
        return rows[0]
    if i >= len(rows):
        return rows[-1]
    return rows[i - 1] if (t - times[i - 1]) <= (times[i] - t) else rows[i]


def _anchor_times(rows, t0, video_dur):
    """Map a stream's absolute timestamps to video time. The station clocks are
    NTP-synced, so absolute stamps across machines ARE comparable: anchor to the
    shared t0 (video t=0) so a late-starting stream (e.g. fish detector warmup)
    lands at its true offset. Guard: if the stream's first stamp sits outside the
    video window (legacy recording with unsynced clocks, >video_dur skew), fall
    back to self-anchor so the overlay still animates instead of freezing."""
    if not rows:
        return []
    first = rows[0][0]
    if first < t0 - 2.0 or first > t0 + video_dur + 2.0:
        base = first  # clock skew -> self-anchor
    else:
        base = t0
    return [r[0] - base for r in rows]


# ----------------------------- overlay --------------------------------------

def _draw_fish_overlay(frame, fish_row):
    """fish_row: [time, x, y, angle_deg]. Mirrors gui_node __update_left_display
    fish-direction block. Draws shapes (they rotate with the frame) and returns
    deferred text items [(x, y, text, scale, thickness), ...] drawn upright
    after the frame is rotated, anchored at the pre-rotation point."""
    if fish_row is None:
        return []
    try:
        px = int(fish_row[1]); py = int(fish_row[2]); angle = float(fish_row[3])
    except (IndexError, ValueError):
        return []
    cv2.circle(frame, (px, py), 5, (0, 0, 255), -1)
    L = 30
    ex = px + int(L * math.cos(math.radians(angle)))
    ey = py - int(L * math.sin(math.radians(angle)))
    cv2.arrowedLine(frame, (px, py), (ex, ey), (0, 255, 0), 2, tipLength=0.3)
    return [(px + 10, py - 10, f"Dir: {angle:.1f}", 0.5, 1)]


def _draw_foma_overlay(frame, speed_row, w, h):
    """speed_row: [time, v, h, r] where v=linear.x, h=linear.y, r=angular.z.
    Mirrors gui_node __update_left_display FOMA-direction block. See
    _draw_fish_overlay for the shape/text split."""
    if speed_row is None:
        return []
    try:
        vx = float(speed_row[1]); vy = float(speed_row[2]); rz = float(speed_row[3])
    except (IndexError, ValueError):
        return []
    cx, cy = w // 2, h // 2
    texts = []
    linear = math.hypot(vx, vy)
    if linear > 0:
        angle = math.degrees(math.atan2(vy, vx))
        L = 30
        ex = int(cx + L * math.cos(math.radians(angle)))
        ey = int(cy - L * math.sin(math.radians(angle)))
        cv2.arrowedLine(frame, (cx, cy), (ex, ey), (0, 255, 255), 2, tipLength=0.3)
        texts.append((ex + 10, ey - 10, f"{linear:.2f}", 0.5, 1))
    if abs(rz) > 1e-6:
        texts.append((cx + 20, cy + 20, f"w={rz:.2f}", 0.5, 1))
    return texts


def _draw_foma_location_overlay(frame, loc_row, sx, sy):
    """loc_row: [time, x_w, y_w, x_i, y_i]. x_i,y_i are pixels in the
    ROOM_CAMERA_FRAME_SHAPE space; sx,sy scale them to the room video size.
    Mirrors gui_node __update_foma_location dot on the room camera."""
    if loc_row is None:
        return
    try:
        px = int(loc_row[3] * sx); py = int(loc_row[4] * sy)
    except (IndexError, ValueError):
        return
    cv2.circle(frame, (px, py), 14, (0, 255, 0), -1)
    cv2.putText(frame, "FOMA", (px + 18, py - 18),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3, cv2.LINE_AA)


def annotate_room_video(in_path, out_path, foma_loc_csv, cam_shape, swap_rb=False):
    """Draw the FOMA room location dot on the room (ceiling) video. foma_location
    is stamped with ceiling-camera capture time == room video frame time, so
    index it by its own elapsed time mapped to frame time (idx/fps)."""
    loc = _read_csv(foma_loc_csv)
    if not loc:
        print('room overlay: no foma_location csv data; skipping')
        return False
    cap = cv2.VideoCapture(in_path)
    if not cap.isOpened():
        print(f'room overlay: cannot open {in_path}')
        return False
    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    shape_w, shape_h = cam_shape
    sx = w / shape_w if shape_w else 1.0
    sy = h / shape_h if shape_h else 1.0

    loc_t = [r[0] - loc[0][0] for r in loc]

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(out_path, fourcc, fps, (w, h))
    if not out.isOpened():
        cap.release()
        print(f'room overlay: cannot open writer {out_path}')
        return False

    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if swap_rb:
            frame = frame[:, :, ::-1].copy()
        t = idx / fps
        _draw_foma_location_overlay(frame, _nearest(loc, loc_t, t), sx, sy)
        out.write(frame)
        idx += 1
    cap.release(); out.release()
    print(f'room overlay: wrote {idx} annotated frames -> {out_path}')
    return True


def annotate_foma_video(in_path, out_path, fish_csv, speed_csv, foma_loc_csv,
                        swap_rb=False):
    cap = cv2.VideoCapture(in_path)
    if not cap.isOpened():
        print(f'overlay: cannot open {in_path}')
        return False
    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    fish = _read_csv(fish_csv)
    speed = _read_csv(speed_csv)
    if not fish and not speed:
        cap.release()
        print('overlay: no fish/speed csv data; skipping annotation')
        return False
    # Shared video t=0 = earliest first-sample across the streams that start at
    # trial start (foma_location, foma_speed). Stamps are NTP-synced across the
    # robot + station, so fish/speed map onto video time absolutely; a stream
    # whose first stamp lands outside the video window (legacy unsynced clocks)
    # self-anchors instead (see _anchor_times).
    foma_loc = _read_csv(foma_loc_csv)
    firsts = [r[0][0] for r in (foma_loc, speed, fish) if r]
    t0 = min(firsts) if firsts else 0.0
    video_dur = (int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0) / fps) if fps else 0.0
    fish_t = _anchor_times(fish, t0, video_dur)
    speed_t = _anchor_times(speed, t0, video_dur)

    # Output is rotated 90 deg CW, so the writer frame size is (h, w) swapped.
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(out_path, fourcc, fps, (h, w))
    if not out.isOpened():
        cap.release()
        print(f'overlay: cannot open writer {out_path}')
        return False

    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if swap_rb:
            # Swap R<->B before drawing so the overlay colors render correctly.
            frame = frame[:, :, ::-1].copy()
        t = idx / fps
        # Draw vector shapes in the native (un-rotated) frame so the math
        # matches the GUI/CSV coords, then rotate the whole frame 90 deg CW so
        # the vectors rotate together with the video. Text is deferred and
        # drawn upright after rotation (rotating it would make it unreadable).
        texts = _draw_fish_overlay(frame, _nearest(fish, fish_t, t))
        texts += _draw_foma_overlay(frame, _nearest(speed, speed_t, t), w, h)
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        # 90 CW point map: (x, y) -> (h - 1 - y, x), h = pre-rotation height.
        for tx, ty, s, scale, thick in texts:
            rx, ry = h - 1 - ty, tx
            cv2.putText(frame, s, (rx, ry), cv2.FONT_HERSHEY_SIMPLEX, scale,
                        (255, 255, 255), thick, cv2.LINE_AA)
        out.write(frame)
        idx += 1
    cap.release(); out.release()
    print(f'overlay: wrote {idx} annotated frames -> {out_path}')
    return True


# ----------------------------- main -----------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('room')
    ap.add_argument('foma')
    ap.add_argument('output')
    ap.add_argument('--swap-foma', action='store_true',
                    help='Swap R<->B channels of the foma video (fixes pre-fix recordings).')
    ap.add_argument('--swap-room', action='store_true',
                    help='Swap R<->B channels of the room video.')
    ap.add_argument('--room-cam-shape', default='1280x1280',
                    help='ROOM_CAMERA_FRAME_SHAPE WxH, for scaling the foma '
                         'location dot onto the room video resolution.')
    ap.add_argument('--log', default=None)
    ap.add_argument('--reframe-log', default=None,
                    help='Path to reframe.log; wait until reframe_worker finishes '
                         'rewriting videos on disk before processing.')
    args = ap.parse_args()

    try:
        _sw, _sh = args.room_cam_shape.lower().split('x')
        room_cam_shape = (int(_sw), int(_sh))
    except (ValueError, AttributeError):
        room_cam_shape = (1280, 1280)

    if args.log:
        sys.stdout = open(args.log, 'a', buffering=1)
        sys.stderr = sys.stdout

    if shutil.which('ffmpeg') is None or shutil.which('ffprobe') is None:
        print('ffmpeg/ffprobe not found on PATH; aborting.')
        return 2

    # Serialize against reframe_worker: it re-encodes foma_video.mp4 in
    # place, so we must not read inputs until it has finished.
    wait_for_reframe(args.reframe_log)

    for p in (args.room, args.foma):
        if not wait_for_file(p):
            print(f'Input missing or empty: {p}')
            return 2

    # Foma overlay pass (CPU OpenCV; one-shot, post-trial). CSVs live in the
    # same folder as the foma video by convention (csv_writer_node writes them
    # into the trial folder).
    foma_input = args.foma
    csv_folder = os.path.dirname(os.path.abspath(args.foma))
    tmp_annotated = tempfile.mktemp(suffix='_annotated.mp4', dir=csv_folder)
    fish_csv = os.path.join(csv_folder, 'fish_location.csv')
    speed_csv = os.path.join(csv_folder, 'foma_speed.csv')
    foma_loc_csv = os.path.join(csv_folder, 'foma_location.csv')
    foma_swapped_in_annotation = False
    if annotate_foma_video(args.foma, tmp_annotated, fish_csv, speed_csv,
                           foma_loc_csv, swap_rb=args.swap_foma):
        foma_input = tmp_annotated
        foma_swapped_in_annotation = args.swap_foma
    else:
        print('overlay: annotation skipped; using raw foma video')
        try:
            os.unlink(tmp_annotated)
        except OSError:
            pass
        tmp_annotated = None

    # Room overlay pass: draw the FOMA location dot on the room (ceiling) video.
    room_input = args.room
    tmp_room = tempfile.mktemp(suffix='_room_annotated.mp4', dir=csv_folder)
    room_swapped_in_annotation = False
    if annotate_room_video(args.room, tmp_room, foma_loc_csv, room_cam_shape,
                           swap_rb=args.swap_room):
        room_input = tmp_room
        room_swapped_in_annotation = args.swap_room
    else:
        print('room overlay: annotation skipped; using raw room video')
        try:
            os.unlink(tmp_room)
        except OSError:
            pass
        tmp_room = None

    try:
        h_room, fps_room = probe_stream(room_input)
        h_foma, fps_foma = probe_stream(foma_input)
    except Exception as e:
        print(f'ffprobe failed: {e}')
        rc = 2
    else:
        H = max(h_room, h_foma)
        if H % 2:
            H += 1

        F = max(fps_room, fps_foma)

        # R<->B swap via colorchannelmixer (matrix sets new_R=old_B, new_B=old_R).
        SWAP = 'colorchannelmixer=0:0:1:0:0:1:0:0:1:0:0:0,'
        room_pre = SWAP if (args.swap_room and not room_swapped_in_annotation) else ''
        # foma input might already be swapped during annotation; only ffmpeg-swap
        # when annotation skipped AND user requested swap.
        foma_pre = SWAP if (args.swap_foma and not foma_swapped_in_annotation) else ''

        # hstack needs both streams at the same fps. The cameras record at
        # different real rates (e.g. room 25, foma 19.7), so the slower stream
        # must be brought up to F. Plain fps= duplicates frames -> visible
        # judder on that half. minterpolate synthesizes motion-compensated
        # in-between frames instead, so the upsampled side stays smooth. Only
        # interpolate a stream that is meaningfully below F (the faster one just
        # passes through fps=).
        def retime(src_fps):
            if F - src_fps > 0.5:
                return f"minterpolate=fps={F}:mi_mode=mci:mc_mode=aobmc,"
            return f"fps={F},"

        filter_complex = (
            f'[0:v]{room_pre}{retime(fps_room)}scale=trunc(oh*a/2)*2:{H},setsar=1[a];'
            f'[1:v]{foma_pre}{retime(fps_foma)}scale=trunc(oh*a/2)*2:{H},setsar=1[b];'
            f'[a][b]hstack=inputs=2[v]'
        )

        def build_cmd(use_nvenc):
            base = ['ffmpeg', '-y']
            if use_nvenc:
                # No explicit -c:v cuvid: the annotated inputs are OpenCV mp4v
                # (MPEG-4 Part 2), not h264, so forcing h264_cuvid spews
                # "Invalid NAL unit 0" and fails the whole GPU pass. -hwaccel
                # cuda lets ffmpeg pick the right decoder per input codec.
                base += ['-hwaccel', 'cuda', '-i', room_input,
                         '-hwaccel', 'cuda', '-i', foma_input]
            else:
                base += ['-i', room_input, '-i', foma_input]
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

        cmd = build_cmd(True)
        print('Running (NVENC):', ' '.join(cmd))
        rc = subprocess.call(cmd)
        if rc != 0:
            print(f'NVENC exit code {rc}; retrying with libx264.')
            cmd = build_cmd(False)
            print('Running (CPU):', ' '.join(cmd))
            rc = subprocess.call(cmd)
        print(f'ffmpeg exit code: {rc}')

    for tmp in (tmp_annotated, tmp_room):
        if tmp is not None:
            try:
                os.unlink(tmp)
            except OSError:
                pass

    return rc


if __name__ == '__main__':
    sys.exit(main())
