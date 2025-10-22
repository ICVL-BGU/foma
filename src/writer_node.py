#!/usr/bin/env python3
import rospy
import cv2
import csv
from foma.srv import Write, WriteRequest, WriteResponse
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from abstract_node import AbstractNode
import datetime
import os
import numpy as np
from geometry_msgs.msg import Twist, Vector3, TwistStamped
from foma.msg import FomaLocation
import math
from etc.settings import *
import shutil
import tempfile
import subprocess
import threading

class WriterNode(AbstractNode):
    def __init__(self):
        super().__init__('writer_node', 'Writer Node')

        self.__write = False
        self.__foma_speed = Twist()
        self.__init_attributes()

        rospy.Subscriber('ceiling_camera/image', Image, self.__room_image_callback)
        rospy.Subscriber('fish_camera/image', CompressedImage, self.__foma_image_callback)
        rospy.Subscriber('fish_detection/state', TwistStamped, self.__fish_state_callback)
        rospy.Subscriber('localization/location', FomaLocation, self.__foma_location_callback)
        rospy.Subscriber('motor_control/speed', Twist, self.__foma_speed_callback)

        rospy.Service('writer_node/write', Write, self.__set_write)
        
        self.bridge = CvBridge()
        rospy.on_shutdown(self.__on_shutdown)

    def __init_attributes(self):
        self.__room_video_writer = None
        self.__room_map_writer = None
        self.__foma_video_writer = None
        self.__foma_location_file = None
        self.__foma_location_csv_writer = None
        self.__fish_location_file = None
        self.__fish_location_csv_writer = None
        self.__trial_output_folder = ""
        self.__trial_timestamp = ""
        self.__subject_id = ""
        self.__start_time = None
        self.__stop_time = None

    def __start_trial(self):
        output_folder = '~/trial_output'
        self.__trial_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M") # change to sent timestamp later
        self.__trial_output_folder = os.path.join(os.path.expanduser(output_folder), f"{self.__trial_timestamp}_id-{self.__subject_id}")
        if not os.path.exists(self.__trial_output_folder):
            self.loginfo(f"Creating output folder {self.__trial_output_folder}")
            os.makedirs(self.__trial_output_folder)

        # File names
        room_video_filename = os.path.join(self.__trial_output_folder, f"room_video.mp4")
        room_map_filename = os.path.join(self.__trial_output_folder, f"room_map.mp4")
        foma_video_filename = os.path.join(self.__trial_output_folder, f"foma_video.mp4")
        foma_location_filename = os.path.join(self.__trial_output_folder, f"foma_location.csv")
        fish_location_filename = os.path.join(self.__trial_output_folder, f"fish_location.csv")

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # MP4 format

        # Writers
        self.__room_video_writer = cv2.VideoWriter(room_video_filename, fourcc, ROOM_CAMERA_FPS, ROOM_CAMERA_FRAME_SHAPE)
        
        self.__room_map = np.ones((ROOM_MAP_FRAME_SHAPE[1], ROOM_MAP_FRAME_SHAPE[0], 3), dtype=np.uint8) * 255
        self.__room_map_writer = cv2.VideoWriter(room_map_filename, fourcc, ROOM_MAP_FPS, ROOM_MAP_FRAME_SHAPE)
        
        self.__foma_video_writer = cv2.VideoWriter(foma_video_filename, fourcc, FOMA_CAMERA_FPS, FOMA_CAMERA_FRAME_SHAPE)

        self.__foma_location_file = open(foma_location_filename, 'a', newline='')
        self.__foma_location_csv_writer = csv.writer(self.__foma_location_file)

        if os.stat(foma_location_filename).st_size == 0:
            self.__foma_location_csv_writer.writerow(["time", "x_w", "y_w", "x_i", "y_i"])
            self.__foma_location_file.flush()  # Ensure the header is written immediately

        self.__fish_location_file = open(fish_location_filename, 'a', newline='')
        self.__fish_location_csv_writer = csv.writer(self.__fish_location_file)

        if os.stat(fish_location_filename).st_size == 0:
            self.__fish_location_csv_writer.writerow(["time", "x", "y", "angle"])
            self.__fish_location_file.flush()  # Ensure the header is written immediately

    def __stop_trial(self):
        rospy.sleep(0.1)  # Ensure all frames are processed
        self.loginfo("Releasing video writers and closing files.")
        if self.__room_video_writer is not None:
            self.__room_video_writer.release()
        if self.__room_map_writer is not None:
            self.__room_map_writer.release()
        if self.__foma_video_writer is not None:
            self.__foma_video_writer.release()
        if self.__foma_location_file is not None:
            self.__foma_location_file.close()
        if self.__fish_location_file is not None:
            self.__fish_location_file.close()

    def __set_write(self, write: WriteRequest):
        if write.msg == "start":
            self.__start_time = write.stamp
            self.__subject_id = write.subject_id
            self.__start_trial()
            self.__write = True

        elif write.msg == "stop" and self.__write:
            self.__stop_time = write.stamp
            self.__write = False
            self.__stop_trial()

            start_t = self.__start_time
            stop_t = self.__stop_time
            folder = self.__trial_output_folder

            # spawn a daemon thread so it won't block shutdown
            threading.Thread(
                target=self.__reframe_videos,
                kwargs={
                    "start_time": start_t,
                    "stop_time": stop_t,
                    "output_folder": folder,
                    "force_reencode": False,
                    "try_ffmpeg_first": True
                },
                daemon=True
            ).start()

        return WriteResponse(success = True)
    
    def __room_image_callback(self, img_msg: Image):
        if not self.__write:
            return
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        self.__room_video_writer.write(img)

    def __foma_image_callback(self, img_msg: CompressedImage):
        if not self.__write:
            return
        img = self.bridge.compressed_imgmsg_to_cv2(img_msg)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        self.__foma_video_writer.write(img)

    def __fish_state_callback(self, state: TwistStamped):
        if not self.__write:
            return
        if state.twist.angular == Vector3(0, 0, 0):
            return
        row = [
                state.header.stamp.to_sec(),
                state.twist.linear.x,
                state.twist.linear.y,
                math.degrees(math.atan2(state.twist.angular.y, state.twist.angular.x))
        ]
        self.__fish_location_csv_writer.writerow(row)
        self.__fish_location_file.flush()

    def __foma_location_callback(self, location: FomaLocation):
        if not self.__write:
            return
        foma_img_location = Vector3(
            location.image.x * ROOM_CAMERA_FRAME_SHAPE[1],
            location.image.y * ROOM_CAMERA_FRAME_SHAPE[0],
            0
        )
        foma_world_location = Vector3(
            location.world.x * ROOM_MAP_FRAME_SHAPE[0],
            location.world.y * ROOM_MAP_FRAME_SHAPE[1],
            0
        )
        
        # Ensure coordinates stay within bounds
        x = np.clip(foma_world_location.x, 0, ROOM_CAMERA_FRAME_SHAPE[1] - 1).astype(int)
        y = np.clip(foma_world_location.y, 0, ROOM_CAMERA_FRAME_SHAPE[0] - 1).astype(int)
        cv2.circle(self.__room_map, (x, y), 5, (0, 255, 0), -1)

        row = [
                location.header.stamp.to_sec(),
                foma_world_location.x,
                foma_world_location.y,
                foma_img_location.x,
                foma_img_location.y
            ]
        self.__foma_location_csv_writer.writerow(row)
        self.__foma_location_file.flush()

        frame = cv2.cvtColor(self.__room_map, cv2.COLOR_RGB2BGR)
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
        self.__room_map_writer.write(frame)

    def __foma_speed_callback(self, speed: Twist):
        self.__foma_speed = speed

    def __reframe_videos(self, start_time,
                        stop_time,
                        output_folder,
                        force_reencode=False,
                        try_ffmpeg_first=True):
        """
        Re-encode pipeline that replaces originals atomically.
        Only logs a single line per file: either a compact success message
        or a compact failure message.
        """

        # Validate times (expect rospy.Time)
        if start_time is None or stop_time is None:
            self.logerr("reframe_videos: start_time or stop_time is None. Aborting.")
            return {}

        try:
            start_s = float(start_time.to_sec())
            stop_s = float(stop_time.to_sec())
        except Exception as e:
            self.logerr(f"reframe_videos: error converting times to seconds: {e}")
            return {}

        duration = stop_s - start_s
        if duration <= 0:
            self.logerr(f"reframe_videos: invalid duration ({duration}s). Aborting.")
            return {}

        video_filenames = [
            os.path.join(output_folder, "room_video.mp4"),
            os.path.join(output_folder, "room_map.mp4"),
            os.path.join(output_folder, "foma_video.mp4"),
        ]

        ffmpeg_path = shutil.which("ffmpeg")
        ffprobe_path = shutil.which("ffprobe")

        def _ffprobe_duration(path):
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

        def _has_audio(path):
            if not ffprobe_path:
                return False
            try:
                p = subprocess.run([ffprobe_path, "-v", "error", "-select_streams", "a",
                                    "-show_entries", "stream=index", "-of", "csv=p=0", path],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=8)
                return bool(p.stdout.strip())
            except Exception:
                return False

        # tolerance: allow small mismatch (half-second or 3% of duration)
        duration_tolerance_s = max(0.5, 0.03 * duration)

        for vf in video_filenames:
            # per-file status info (no logs until the end)
            info = {
                "frames": None,
                "new_fps": None,
                "method": None,
                "success": False,
                "notes": []  # short reasons / notes accumulated
            }

            shortname = os.path.basename(vf)

            if not os.path.exists(vf):
                info["notes"].append("file not found")
                self.logerr(f"reframe_videos: {shortname}: failed: file not found")
                continue

            # count frames (fast property, fallback to manual)
            cap = cv2.VideoCapture(vf)
            if not cap.isOpened():
                info["notes"].append("cv2 open failed")
                self.logerr(f"reframe_videos: {shortname}: failed: cv2 cannot open file")
                continue

            try:
                prop_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
            except Exception:
                prop_count = 0

            frames = prop_count
            if frames <= 0 or frames > 1e9:
                # count manually but don't log here, just add a note
                info["notes"].append("framecount unreliable; counted manually")
                frames = 0
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                while True:
                    ret, _ = cap.read()
                    if not ret:
                        break
                    frames += 1
            cap.release()

            if frames == 0:
                info["notes"].append("no frames")
                self.logerr(f"reframe_videos: {shortname}: failed: no frames found")
                continue

            new_fps = frames / float(duration)
            info["frames"] = int(frames)
            info["new_fps"] = float(new_fps)

            _, ext = os.path.splitext(vf)
            tmpout = None
            final_tmp = None
            tmp_files_to_clean = []

            # ---------- Try ffmpeg re-encode to temp file (fast path) ----------
            if ffmpeg_path and not force_reencode and try_ffmpeg_first:
                try:
                    tmpfd, tmpout = tempfile.mkstemp(suffix=ext)
                    os.close(tmpfd)
                    tmp_files_to_clean.append(tmpout)
                    fps_arg = f"{new_fps:.6f}"
                    cmd = [
                        ffmpeg_path,
                        "-y",
                        "-r", fps_arg,        # treat input as this fps
                        "-i", vf,
                        "-c:v", "libx264",
                        "-preset", "veryfast",
                        "-crf", "23",
                        "-r", fps_arg,        # output fps
                        "-c:a", "copy",
                        tmpout
                    ]
                    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    if p.returncode == 0:
                        out_dur = _ffprobe_duration(tmpout) if ffprobe_path else None
                        if out_dur is None or abs(out_dur - duration) <= duration_tolerance_s:
                            final_tmp = tmpout
                            info["method"] = "ffmpeg_reencode"
                        else:
                            info["notes"].append("ffmpeg duration mismatch")
                            try:
                                os.remove(tmpout)
                            except Exception:
                                pass
                    else:
                        info["notes"].append("ffmpeg failed")
                        try:
                            os.remove(tmpout)
                        except Exception:
                            pass
                except Exception:
                    info["notes"].append("ffmpeg exception")
                    try:
                        if tmpout and os.path.exists(tmpout):
                            os.remove(tmpout)
                    except Exception:
                        pass

            # ---------- If ffmpeg failed, fallback to OpenCV re-encode deterministically ----------
            if final_tmp is None:
                tmpfd, tmp_video = tempfile.mkstemp(suffix=ext)
                os.close(tmpfd)
                tmp_files_to_clean.append(tmp_video)
                written = 0
                try:
                    cap = cv2.VideoCapture(vf)
                    if not cap.isOpened():
                        info["notes"].append("opencv open failed for re-encode")
                        try:
                            os.remove(tmp_video)
                        except Exception:
                            pass
                        # final single log below will show failure
                    else:
                        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        if w <= 0 or h <= 0:
                            ret, frame = cap.read()
                            if not ret or frame is None:
                                info["notes"].append("cannot determine frame size")
                                cap.release()
                                try:
                                    os.remove(tmp_video)
                                except Exception:
                                    pass
                            else:
                                h, w = frame.shape[0], frame.shape[1]
                                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

                        if info["notes"] and info["notes"][-1].startswith("cannot"):
                            # skip writing
                            pass
                        else:
                            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                            out = cv2.VideoWriter(tmp_video, fourcc, new_fps, (w, h))
                            if not out.isOpened():
                                info["notes"].append("opencv writer failed")
                                cap.release()
                                try:
                                    os.remove(tmp_video)
                                except Exception:
                                    pass
                            else:
                                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
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
                                info["method"] = "opencv_reencode"
                except Exception:
                    info["notes"].append("opencv exception")
                    try:
                        if tmp_video and os.path.exists(tmp_video):
                            os.remove(tmp_video)
                    except Exception:
                        pass

            # ---------- If final_tmp exists and original had audio, attempt mux into a new temp (use ffmpeg) ----------
            tmp_mux = None
            if final_tmp and ffmpeg_path and _has_audio(vf):
                try:
                    tmpfd2, tmp_mux = tempfile.mkstemp(suffix=ext)
                    os.close(tmpfd2)
                    tmp_files_to_clean.append(tmp_mux)
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
                        info["method"] = (info.get("method") or "") + "_with_audio_mux"
                    else:
                        info["notes"].append("audio mux failed")
                        try:
                            os.remove(tmp_mux)
                        except Exception:
                            pass
                except Exception:
                    info["notes"].append("audio mux exception")
                    try:
                        if tmp_mux and os.path.exists(tmp_mux):
                            os.remove(tmp_mux)
                    except Exception:
                        pass

            # ---------- Now replace original atomically: rename original->backup, move final_tmp->original, remove backup ----------
            if final_tmp:
                bak = vf + ".bak_reframe"
                try:
                    if os.path.exists(bak):
                        try:
                            os.remove(bak)
                        except Exception:
                            pass
                    os.replace(vf, bak)    # atomic rename
                    try:
                        os.replace(final_tmp, vf)
                        # success: remove backup
                        try:
                            os.remove(bak)
                        except Exception:
                            pass
                        # cleanup any leftover tmp files in list
                        for tfile in tmp_files_to_clean:
                            try:
                                if os.path.exists(tfile):
                                    os.remove(tfile)
                            except Exception:
                                pass
                        info["success"] = True
                    except Exception as e_move:
                        info["notes"].append("move temp failed")
                        # attempt to restore backup -> original
                        try:
                            if os.path.exists(bak):
                                os.replace(bak, vf)
                                info["notes"].append("backup restored")
                        except Exception:
                            info["notes"].append("backup restore failed")
                        try:
                            if os.path.exists(final_tmp):
                                os.remove(final_tmp)
                        except Exception:
                            pass
                except Exception:
                    info["notes"].append("rename to backup failed")
                    try:
                        if final_tmp and os.path.exists(final_tmp):
                            os.remove(final_tmp)
                    except Exception:
                        pass
                finally:
                    # ensure we didn't leave stray temp files
                    for tfile in tmp_files_to_clean:
                        try:
                            if os.path.exists(tfile):
                                # keep only final_tmp moved - others delete
                                if final_tmp and os.path.abspath(tfile) == os.path.abspath(final_tmp):
                                    continue
                                os.remove(tfile)
                        except Exception:
                            pass
            else:
                info["notes"].append("no final temp produced")

            # ---- Single-line logging per file ----
            if info["success"]:
                # concise success: filename, new fps, frames, method
                self.loginfo(f"reframe_videos: {shortname}: success fps={info['new_fps']:.3f}, frames={info['frames']}, method={info['method']}")
            else:
                # concise failure: filename + last note (or joined notes)
                reason = info["notes"][-1] if info["notes"] else "unknown error"
                # prefer short descriptive messages
                pretty = {
                    "file not found": "file not found",
                    "cv2 open failed": "cannot open with OpenCV",
                    "no frames": "no frames",
                    "ffmpeg failed": "ffmpeg failed",
                    "ffmpeg exception": "ffmpeg exception",
                    "ffmpeg duration mismatch": "ffmpeg produced wrong duration",
                    "ffmpeg re-encode duration mismatch": "ffmpeg produced wrong duration",
                    "ffmpeg duration mismatch": "ffmpeg produced wrong duration",
                    "ffmpeg exception": "ffmpeg exception",
                    "ffmpeg duration mismatch": "ffmpeg produced wrong duration",
                    "audio mux failed": "audio mux failed",
                    "audio mux exception": "audio mux exception",
                    "move temp failed": "moving new file into place failed",
                    "backup restore failed": "failed to restore backup after move failure",
                    "rename to backup failed": "failed to rename original to backup",
                    "opencv open failed for re-encode": "cannot open for re-encode",
                    "cannot determine frame size": "cannot determine frame size",
                    "opencv writer failed": "OpenCV writer failed",
                    "opencv exception": "OpenCV exception",
                    "no final temp produced": "no final temp produced",
                }.get(reason, reason)
                self.logerr(f"reframe_videos: {shortname}: failed: {pretty}")

    def __on_shutdown(self):
        self.loginfo("Shutting down WriterNode...")
        self.__stop_trial()

if __name__ == "__main__":
    rospy.init_node('ceiling_camera', anonymous=False)
    WriterNode()
    rospy.spin()
