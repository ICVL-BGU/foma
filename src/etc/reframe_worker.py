#!/usr/bin/env python3
"""
reframe_worker.py

Standalone worker that re-encodes video files in an output folder so their FPS =
frames / (stop_time - start_time). Designed to be invoked as a separate process.

Usage:
    python reframe_worker.py START_S STOP_S OUTPUT_FOLDER [--force] [--no-ffmpeg] [--log LOGFILE]

Example:
    python reframe_worker.py 1698518400.0 1698518460.0 /home/user/trial_output/20251021_1802_id-test --log /tmp/reframe.log
"""
import argparse
import os
import sys
import time
import tempfile
import shutil
import subprocess
import logging

# We import cv2 lazily (only if needed)
try:
    import cv2
    import numpy as np
except Exception:
    cv2 = None
    np = None


def setup_logger(logfile_path=None):
    logger = logging.getLogger("reframe_worker")
    logger.setLevel(logging.INFO)
    fmt = logging.Formatter("%(asctime)s %(levelname)s: %(message)s")
    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(fmt)
    logger.addHandler(sh)
    if logfile_path:
        fh = logging.FileHandler(logfile_path)
        fh.setFormatter(fmt)
        logger.addHandler(fh)
    return logger


def ffprobe_duration(path, ffprobe_path):
    if not ffprobe_path:
        return None
    try:
        p = subprocess.run([ffprobe_path, "-v", "error", "-show_entries", "format=duration",
                            "-of", "default=noprint_wrappers=1:nokey=1", path],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=10)
        out = p.stdout.strip()
        return float(out) if out else None
    except Exception:
        return None


def has_audio(path, ffprobe_path):
    if not ffprobe_path:
        return False
    try:
        p = subprocess.run([ffprobe_path, "-v", "error", "-select_streams", "a",
                            "-show_entries", "stream=index", "-of", "csv=p=0", path],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=8)
        return bool(p.stdout.strip())
    except Exception:
        return False


def reframe_single_file(vf, start_s, stop_s, logger, force_reencode=False, try_ffmpeg_first=True, duration_tolerance_s=None):
    """
    Re-encode one file. Returns a tuple (success: bool, message: str, new_fps: float or None, frames: int or None)
    """
    ffmpeg_path = shutil.which("ffmpeg")
    ffprobe_path = shutil.which("ffprobe")

    duration = stop_s - start_s
    if duration <= 0:
        return False, "invalid duration", None, None

    if duration_tolerance_s is None:
        duration_tolerance_s = max(0.5, 0.03 * duration)

    if not os.path.exists(vf):
        return False, "file not found", None, None

    # Count frames
    frames = None
    if cv2 is None:
        logger.warning("OpenCV not available; cannot count frames reliably - aborting file")
        return False, "opencv missing", None, None

    cap = cv2.VideoCapture(vf)
    if not cap.isOpened():
        return False, "cv2 cannot open", None, None

    try:
        prop_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    except Exception:
        prop_count = 0

    frames = prop_count
    if frames <= 0 or frames > 1e9:
        logger.info(f"framecount unreliable for {vf}; performing manual count")
        frames = 0
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        while True:
            ret, _ = cap.read()
            if not ret:
                break
            frames += 1
    cap.release()

    if frames == 0:
        return False, "no frames", None, 0

    new_fps = frames / float(duration)

    _, ext = os.path.splitext(vf)
    tmp_files = []
    final_tmp = None

    # ---------- Try ffmpeg re-encode (fast) ----------
    if ffmpeg_path and not force_reencode and try_ffmpeg_first:
        try:
            fd, tmpout = tempfile.mkstemp(suffix=ext)
            os.close(fd)
            tmp_files.append(tmpout)
            fps_arg = f"{new_fps:.6f}"
            # use -r before -i to force input interpretation, and -r after to set output fps
            cmd = [
                ffmpeg_path,
                "-y",
                "-r", fps_arg,
                "-i", vf,
                "-c:v", "libx264",
                "-preset", "veryfast",
                "-crf", "23",
                "-r", fps_arg,
                "-c:a", "copy",
                tmpout
            ]
            logger.info(f"running ffmpeg (fast) for {os.path.basename(vf)} ...")
            p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if p.returncode == 0:
                out_dur = ffprobe_duration(tmpout, ffprobe_path)
                if out_dur is None or abs(out_dur - duration) <= duration_tolerance_s:
                    final_tmp = tmpout
                    logger.info(f"ffmpeg re-encode succeeded for {os.path.basename(vf)} (tmp={tmpout})")
                else:
                    logger.warning(f"ffmpeg produced wrong duration ({out_dur:.3f}s) for {os.path.basename(vf)}; will fallback")
                    try:
                        os.remove(tmpout)
                    except Exception:
                        pass
            else:
                stderr_snip = p.stderr.decode("utf-8", errors="ignore")[:400]
                logger.warning(f"ffmpeg re-encode failed for {os.path.basename(vf)}; stderr snippet: {stderr_snip}")
                try:
                    os.remove(tmpout)
                except Exception:
                    pass
        except Exception as e:
            logger.warning(f"ffmpeg exception for {os.path.basename(vf)}: {e}")
            try:
                if tmpout and os.path.exists(tmpout):
                    os.remove(tmpout)
            except Exception:
                pass

    # ---------- OpenCV fallback ----------
    if final_tmp is None:
        try:
            fd, tmp_video = tempfile.mkstemp(suffix=ext)
            os.close(fd)
            tmp_files.append(tmp_video)
            cap = cv2.VideoCapture(vf)
            if not cap.isOpened():
                return False, "opencv cannot open for re-encode", None, frames

            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if w <= 0 or h <= 0:
                ret, frame = cap.read()
                if not ret or frame is None:
                    cap.release()
                    try:
                        os.remove(tmp_video)
                    except Exception:
                        pass
                    return False, "cannot determine frame size", None, frames
                h, w = frame.shape[0], frame.shape[1]
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            out = cv2.VideoWriter(tmp_video, fourcc, new_fps, (w, h))
            if not out.isOpened():
                cap.release()
                try:
                    os.remove(tmp_video)
                except Exception:
                    pass
                return False, "opencv writer failed", None, frames

            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            written = 0
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                if frame is None:
                    continue
                if len(frame.shape) == 2:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                elif frame.shape[2] == 4:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                out.write(frame)
                written += 1
            cap.release()
            out.release()
            final_tmp = tmp_video
            logger.info(f"OpenCV re-encode finished for {os.path.basename(vf)}, wrote {written} frames")
        except Exception as e:
            logger.exception(f"OpenCV re-encode exception for {os.path.basename(vf)}: {e}")
            try:
                if tmp_video and os.path.exists(tmp_video):
                    os.remove(tmp_video)
            except Exception:
                pass
            return False, "opencv exception", None, frames

    # ---------- If final_tmp exists and original had audio, mux it (optional) ----------
    if final_tmp and ffmpeg_path and has_audio(vf, shutil.which("ffprobe")):
        try:
            fd, tmp_mux = tempfile.mkstemp(suffix=ext)
            os.close(fd)
            tmp_files.append(tmp_mux)
            cmd_mux = [
                ffmpeg_path, "-y", "-fflags", "+genpts",
                "-i", final_tmp, "-i", vf,
                "-map", "0:v:0", "-map", "1:a:0",
                "-c:v", "copy", "-c:a", "copy",
                "-shortest",
                tmp_mux
            ]
            pm = subprocess.run(cmd_mux, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if pm.returncode == 0:
                try:
                    if final_tmp != tmp_mux and os.path.exists(final_tmp):
                        os.remove(final_tmp)
                except Exception:
                    pass
                final_tmp = tmp_mux
                logger.info(f"audio mux succeeded for {os.path.basename(vf)}")
            else:
                logger.warning(f"audio mux failed for {os.path.basename(vf)}; keeping video only")
                try:
                    os.remove(tmp_mux)
                except Exception:
                    pass
        except Exception as e:
            logger.warning(f"audio mux exception for {os.path.basename(vf)}: {e}")
            try:
                if tmp_mux and os.path.exists(tmp_mux):
                    os.remove(tmp_mux)
            except Exception:
                pass

    # ---------- Atomic replace original -> backup, final_tmp -> original ----------
    if final_tmp:
        bak = vf + ".bak_reframe"
        try:
            if os.path.exists(bak):
                try:
                    os.remove(bak)
                except Exception:
                    pass
            os.replace(vf, bak)
            try:
                os.replace(final_tmp, vf)
                # success -> remove backup
                try:
                    os.remove(bak)
                except Exception:
                    pass
                # cleanup tmp files
                for t in tmp_files:
                    try:
                        if os.path.exists(t) and os.path.abspath(t) != os.path.abspath(vf):
                            os.remove(t)
                    except Exception:
                        pass
                return True, "success", new_fps, frames
            except Exception as e:
                logger.error(f"failed to move temp into place for {os.path.basename(vf)}: {e}")
                # attempt restore
                try:
                    if os.path.exists(bak):
                        os.replace(bak, vf)
                except Exception as e2:
                    logger.error(f"failed to restore backup for {os.path.basename(vf)}: {e2}")
                # cleanup final_tmp
                try:
                    if os.path.exists(final_tmp):
                        os.remove(final_tmp)
                except Exception:
                    pass
                return False, "move failed", None, frames
        except Exception as e:
            logger.error(f"failed to rename original to backup for {os.path.basename(vf)}: {e}")
            try:
                if final_tmp and os.path.exists(final_tmp):
                    os.remove(final_tmp)
            except Exception:
                pass
            return False, "rename backup failed", None, frames
    else:
        return False, "no final tmp", None, frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("start_s", type=float, help="trial start time (seconds since epoch)")
    ap.add_argument("stop_s", type=float, help="trial stop time (seconds since epoch)")
    ap.add_argument("output_folder", type=str, help="trial output folder with video files")
    ap.add_argument("--force", action="store_true", dest="force", help="force re-encode path (skip ffmpeg fast path)")
    ap.add_argument("--no-ffmpeg", action="store_true", dest="no_ffmpeg", help="do not use ffmpeg at all (force OpenCV fallback)")
    ap.add_argument("--log", type=str, dest="logpath", default=None, help="path for worker log file")
    args = ap.parse_args()

    logger = setup_logger(args.logpath)
    logger.info(f"reframe_worker starting for folder={args.output_folder} start={args.start_s} stop={args.stop_s}")

    # iterate files
    files = [
        os.path.join(args.output_folder, "room_video.mp4"),
        os.path.join(args.output_folder, "room_map.mp4"),
        os.path.join(args.output_folder, "foma_video.mp4"),
    ]
    for vf in files:
        ok, msg, new_fps, frames = reframe_single_file(
            vf,
            args.start_s,
            args.stop_s,
            logger,
            force_reencode=args.force or args.no_ffmpeg,
            try_ffmpeg_first=not args.no_ffmpeg
        )
        short = os.path.basename(vf)
        if ok:
            logger.info(f"{short}: success fps={new_fps:.3f} frames={frames}")
        else:
            logger.error(f"{short}: failed: {msg}")

    logger.info("reframe_worker done.")


if __name__ == "__main__":
    main()
