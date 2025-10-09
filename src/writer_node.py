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
from collections import deque
import threading

class WriterNode(AbstractNode):
    def __init__(self):
        super().__init__('writer_node', 'Writer Node')

        self.__write = False
        self.__foma_speed = Twist()

        self.__queues = {
            'room_image': deque(maxlen=100),
            'room_map': deque(maxlen=100),
            'foma_image': deque(maxlen=100),
            'fish_state': deque(maxlen=100),
            'foma_location': deque(maxlen=100)
        }

        rospy.Subscriber('ceiling_camera/image', Image, self.__room_image_callback)
        rospy.Subscriber('fish_camera/image', CompressedImage, self.__foma_image_callback)
        rospy.Subscriber('fish_detection/state', TwistStamped, self.__fish_state_callback)
        rospy.Subscriber('localization/location', FomaLocation, self.__foma_location_callback)
        rospy.Subscriber('motor_control/speed', Twist, self.__foma_speed_callback)

        rospy.Service('writer_node/write', Write, self.__set_write)
        
        self.bridge = CvBridge()
        rospy.on_shutdown(self.__on_shutdown)

    def __write_loop(self):
        while self.__write or np.any([len(q) > 0 for q in self.__queues.values()]):
            try:
                img = self.__queues['room_image'].popleft()
                self.__room_video_writer.write(img)
            except IndexError:
                pass
            try:
                map_frame = self.__queues['room_map'].popleft()
                self.__room_map_writer.write(map_frame)
            except IndexError:
                pass
            try:
                foma_img = self.__queues['foma_image'].popleft()
                self.__foma_video_writer.write(foma_img)
            except IndexError:
                pass
            try:
                row = self.__queues['fish_state'].popleft()
                self.__fish_location_csv_writer.writerow(row)
                self.__fish_location_file.flush()
            except IndexError:
                pass
            try:
                location = self.__queues['foma_location'].popleft()
                self.__foma_location_csv_writer.writerow(location)
                self.__foma_location_file.flush()
            except IndexError:
                pass
            try:
                room_map = self.__queues['room_map'].popleft()
                self.__room_map_writer.write(room_map)
            except IndexError:
                pass

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
        if np.any([len(q) > 0 for q in self.__queues.values()]):
            rospy.Timer(rospy.Duration(0.05), lambda event: self.__stop_trial(), oneshot=True)
        self.__room_video_writer.release()
        self.__room_map_writer.release()
        self.__foma_video_writer.release()
        self.__foma_location_file.close()
        self.__fish_location_file.close()

    def __set_write(self, write: WriteRequest):
        if write.msg == "start":
            self.__subject_id = write.subject_id
            self.__write = True
            self.__start_trial()
            threading.Thread(target=self.__write_loop, daemon=True).start()
        elif write.msg == "stop":
            self.__write = False
            self.__stop_trial()
        elif write.msg == "pause":
            self.__write = False
        elif write.msg == "continue":
            self.__write = True
        return WriteResponse(success = True)
    
    def __room_image_callback(self, img_msg: Image):
        if not self.__write:
            return
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        self.__queues['room_image'].append(img)

    def __foma_image_callback(self, img_msg: CompressedImage):
        if not self.__write:
            return
        img = self.bridge.compressed_imgmsg_to_cv2(img_msg)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        self.__queues['foma_image'].append(img)

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
        self.__queues['fish_state'].append(row)

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
        self.__queues['foma_location'].append(row)

        frame = cv2.cvtColor(self.__room_map, cv2.COLOR_RGB2BGR)
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
        self.__queues['room_map'].append(frame)

    def __foma_speed_callback(self, speed: Twist):
        self.__foma_speed = speed

    def __on_shutdown(self):
        self.loginfo("Shutting down WriterNode...")
        self.__stop_trial()

if __name__ == "__main__":
    rospy.init_node('ceiling_camera', anonymous=False)
    WriterNode()
    rospy.spin()
