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

        rospy.Subscriber('ceiling_camera/image', Image, self.__room_image_callback)
        rospy.Subscriber('fish_camera/image', CompressedImage, self.__foma_image_callback)
        rospy.Subscriber('fish_detection/state', TwistStamped, self.__fish_state_callback)
        rospy.Subscriber('localization/location', FomaLocation, self.__foma_location_callback)
        rospy.Subscriber('motor_control/speed', Twist, self.__foma_speed_callback)

        rospy.Service('writer_node/write', Write, self.__set_write)
        
        self.bridge = CvBridge()
        rospy.on_shutdown(self.__on_shutdown)

    def __start_trial(self):
        output_folder = '~/trial_output'
        self.__trial_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M")
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
        self.__room_video_writer.release()
        self.__room_map_writer.release()
        self.__foma_video_writer.release()
        self.__foma_location_file.close()
        self.__fish_location_file.close()

    def __set_write(self, write: WriteRequest):
        if write.msg == "start":
            self.__start_time = write.time
            self.__subject_id = write.subject_id
            self.__write = True
            self.__start_trial()
        elif write.msg == "stop":
            self.__stop_time = write.time
            self.__write = False
            self.__stop_trial()

            start_t = self.__start_time
            stop_t = self.__stop_time
            folder = self.__trial_output_folder

            # optionally supply a custom file list:
            files = [
                os.path.join(folder, "room_video.mp4"),
                os.path.join(folder, "room_map.mp4"),
                os.path.join(folder, "foma_video.mp4"),
            ]

            # spawn a daemon thread so it won't block shutdown
            threading.Thread(
                target=self.__reframe_videos,
                args=(start_t, stop_t, folder, files),
                kwargs={"logfunc": self.loginfo, "force_reencode": False},
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

    def __reframe_videos(start_time,
                           stop_time,
                           trial_output_folder,
                           video_filenames=None,
                           force_reencode=False,
                           try_ffmpeg_first=True,
                           logfunc=print):
        """
        Recalculate and apply FPS for recorded videos using wall-clock trial duration.
        This function is thread-safe as it doesn't access self.*; pass required values in.

        Args:
            start_time (rospy.Time): trial start time (must have to_sec()).
            stop_time (rospy.Time): trial stop time (must have to_sec()).
            trial_output_folder (str): folder containing videos.
            video_filenames (list[str] | None): full paths or None to use defaults
            force_reencode (bool): skip fast remux and go to re-encode
            try_ffmpeg_first (bool): try ffmpeg container remux first
            logfunc (callable): function(msg) for logging (pass self.loginfo if available)

        Returns:
            dict: per-file result info (frames, duration_s, calculated_fps, method, success, msg)
        """
        log = logfunc if callable(logfunc) else print

        # Validate times (expect rospy.Time)
        if start_time is None or stop_time is None:
            log("recalc_videos_fps: start_time or stop_time is None. Aborting.")
            return {}

        # convert to seconds
        try:
            start_s = float(start_time.to_sec())
            stop_s = float(stop_time.to_sec())
        except Exception as e:
            log(f"recalc_videos_fps: error converting times to seconds: {e}")
            return {}

        duration = stop_s - start_s
        if duration <= 0:
            log(f"recalc_videos_fps: invalid duration ({duration}s). Aborting.")
            return {}

        # default files
        if video_filenames is None:
            folder = trial_output_folder
            if not folder:
                log("recalc_videos_fps: trial_output_folder not provided. Aborting.")
                return {}
            video_filenames = [
                os.path.join(folder, "room_video.mp4"),
                os.path.join(folder, "room_map.mp4"),
                os.path.join(folder, "foma_video.mp4"),
            ]

        results = {}
        ffmpeg_path = shutil.which("ffmpeg")

        for vf in video_filenames:
            res = {"frames": None, "duration_s": duration, "calculated_fps": None,
                   "method": None, "success": False, "msg": ""}
            results[vf] = res

            if not os.path.exists(vf):
                res["msg"] = "file not found"
                log(f"recalc_videos_fps: {vf} not found, skipping.")
                continue

            # open with OpenCV to get frame count
            cap = cv2.VideoCapture(vf)
            if not cap.isOpened():
                res["msg"] = "cv2.VideoCapture failed to open file"
                log(f"recalc_videos_fps: cv2 couldn't open {vf}")
                continue

            try:
                prop_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
            except Exception:
                prop_count = 0

            frames = prop_count
            # fallback to manual count if property unreliable
            if frames <= 0 or frames > 1e9:
                log(f"recalc_videos_fps: unreliable frame count ({prop_count}) for {vf}, counting frames...")
                frames = 0
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                while True:
                    ret, _ = cap.read()
                    if not ret:
                        break
                    frames += 1
                log(f"recalc_videos_fps: counted {frames} frames for {vf}")
            else:
                log(f"recalc_videos_fps: cv2 reports {frames} frames for {vf}")

            cap.release()

            if frames == 0:
                res["msg"] = "no frames found"
                log(f"recalc_videos_fps: {vf} contains 0 frames, skipping.")
                continue

            new_fps = frames / float(duration)
            res["frames"] = int(frames)
            res["calculated_fps"] = float(new_fps)

            applied = False

            # ffmpeg remux (fast, preserves audio) if available
            if ffmpeg_path and not force_reencode and try_ffmpeg_first:
                tmpfd, tmpout = tempfile.mkstemp(suffix=os.path.splitext(vf)[1])
                os.close(tmpfd)
                try:
                    cmd = [ffmpeg_path, "-y", "-i", vf, "-c", "copy", "-r", f"{new_fps}", tmpout]
                    log(f"recalc_videos_fps: ffmpeg remux cmd: {' '.join(cmd)}")
                    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    if p.returncode == 0:
                        shutil.move(tmpout, vf)
                        res.update(method="ffmpeg_remux", success=True, msg="ffmpeg remuxed container (no re-encode).")
                        applied = True
                        log(f"recalc_videos_fps: ffmpeg remux succeeded for {vf}")
                    else:
                        res["msg"] = "ffmpeg remux failed: " + (p.stderr.decode("utf-8", errors="ignore")[:500])
                        log(f"recalc_videos_fps: ffmpeg remux failed for {vf}. stderr: {res['msg']}")
                        try: os.remove(tmpout)
                        except Exception: pass
                except Exception as e:
                    res["msg"] = f"ffmpeg remux exception: {e}"
                    log(f"recalc_videos_fps: ffmpeg remux exception for {vf}: {e}")
                    try: os.remove(tmpout)
                    except Exception: pass

            # ffmpeg re-encode fallback (preserves audio if copyable)
            if not applied and ffmpeg_path and not force_reencode:
                tmpfd, tmpout = tempfile.mkstemp(suffix=os.path.splitext(vf)[1])
                os.close(tmpfd)
                try:
                    cmd = [
                        ffmpeg_path, "-y", "-i", vf,
                        "-r", f"{new_fps}",
                        "-c:v", "libx264", "-preset", "veryfast", "-crf", "23",
                        "-c:a", "copy",
                        tmpout
                    ]
                    log(f"recalc_videos_fps: ffmpeg re-encode cmd: {' '.join(cmd)}")
                    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    if p.returncode == 0:
                        shutil.move(tmpout, vf)
                        res.update(method="ffmpeg_reencode", success=True, msg="ffmpeg re-encoded with new FPS.")
                        applied = True
                        log(f"recalc_videos_fps: ffmpeg re-encode succeeded for {vf}")
                    else:
                        res["msg"] = "ffmpeg re-encode failed: " + (p.stderr.decode("utf-8", errors="ignore")[:500])
                        log(f"recalc_videos_fps: ffmpeg re-encode failed for {vf}. stderr: {res['msg']}")
                        try: os.remove(tmpout)
                        except Exception: pass
                except Exception as e:
                    res["msg"] = f"ffmpeg re-encode exception: {e}"
                    log(f"recalc_videos_fps: ffmpeg re-encode exception for {vf}: {e}")
                    try: os.remove(tmpout)
                    except Exception: pass

            # OpenCV re-encode fallback (no audio)
            if not applied:
                try:
                    cap = cv2.VideoCapture(vf)
                    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    tmpfd, tmpout = tempfile.mkstemp(suffix=os.path.splitext(vf)[1])
                    os.close(tmpfd)
                    out = cv2.VideoWriter(tmpout, fourcc, new_fps, (w, h))
                    if not out.isOpened():
                        res["msg"] = "OpenCV VideoWriter failed to open"
                        log(f"recalc_videos_fps: OpenCV VideoWriter failed for {vf}")
                        try: os.remove(tmpout)
                        except Exception: pass
                    else:
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
                            out.write(frame)
                            written += 1
                        cap.release()
                        out.release()
                        if written == frames:
                            shutil.move(tmpout, vf)
                            res.update(method="opencv_reencode",
                                       success=True,
                                       msg=f"OpenCV re-encoded, wrote {written} frames at {new_fps:.3f} fps (audio lost).")
                            applied = True
                            log(f"recalc_videos_fps: OpenCV re-encode succeeded for {vf}, wrote {written} frames")
                        else:
                            res["msg"] = f"OpenCV re-encode wrote {written} frames (expected {frames})."
                            log(f"recalc_videos_fps: OpenCV wrote {written}/{frames} frames for {vf}")
                            try: os.remove(tmpout)
                            except Exception: pass
                except Exception as e:
                    res["msg"] = f"OpenCV re-encode exception: {e}"
                    log(f"recalc_videos_fps: exception during OpenCV re-encode for {vf}: {e}")

            if not res["success"] and not res["msg"]:
                res["msg"] = "Could not apply new FPS with available methods."

        return results

    def __on_shutdown(self):
        self.loginfo("Shutting down WriterNode...")
        self.__stop_trial()

if __name__ == "__main__":
    rospy.init_node('ceiling_camera', anonymous=False)
    WriterNode()
    rospy.spin()
