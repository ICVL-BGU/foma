#!/usr/bin/env python3
import rospy
import cv2
import csv
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from foma.srv import Write, WriteRequest, WriteResponse
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from abstract_node import AbstractNode
import datetime
import os
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from foma.msg import FomaLocation
from std_msgs.msg import Int16MultiArray
import math

class WriterNode(AbstractNode):
    def __init__(self):
        super().__init__('writer_node', 'Writer Node')

        rospy.Service('write', Write, self.__set_write)

        self.__write = False
        self.__foma_speed = Twist()

        rospy.Subscriber('ceiling_camera/image', Image, self.__room_image_callback)
        rospy.Subscriber('fish_camera/image', CompressedImage, self.__foma_image_callback)
        rospy.Subscriber('fish_detection/state', Twist, self.__fish_state_callback)
        rospy.Subscriber('localization/location', FomaLocation, self.__foma_location_callback)
        rospy.Subscriber('motor_control/speed', Twist, self.__foma_speed_callback)

        self.bridge = CvBridge()
        rospy.on_shutdown(self._on_shutdown)

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

        # FPS
        room_fps = 25  # Default FPS (adjust based on the camera FPS)
        map_fps = 10  # Default FPS (adjust based on the camera FPS)
        foma_fps = 20  # Default FPS (adjust based on the camera FPS)

        # Frame shapes
        self.__room_frame_shape = (2560, 2560)
        self.__map_frame_shape = (1000, 1000)
        self.__foma_frame_shape = (640, 640)

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # MP4 format

        # Writers
        self.__room_video_writer = cv2.VideoWriter(room_video_filename, fourcc, room_fps, self.room_frame_shape)
        
        self.__room_map = np.ones((self.__map_frame_shape[1], self.__map_frame_shape[0], 3), dtype=np.uint8) * 255
        self.__room_map_writer = cv2.VideoWriter(room_map_filename, fourcc, map_fps, self.__map_frame_shape)
        
        self.__foma_video_writer = cv2.VideoWriter(foma_video_filename, fourcc, foma_fps, self.__foma_frame_shape)

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
            self.__subject_id = write.subject_id
            self.__start_trial()
            self.__write = True
        elif write.msg == "stop":
            self.__write = False
            self.__stop_trial()
        elif write.msg == "pause":
            self.__write = False
        elif write.msg == "continue":
            self.__write = True
        return SetBoolResponse(success = True, message = "Writer set to {}.".format(self.__write))
    
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

    def __fish_state_callback(self, state: Twist):
        if not self.__write:
            return
        if state.angular == Vector3(0, 0, 0):
            return
        timestamp = rospy.get_time()
        self.__fish_location_csv_writer.writerow([
                timestamp,
                state.linear.x,
                state.linear.y,
                math.degrees(math.atan2(state.angular.y, state.angular.x))
        ])
        self.__fish_location_file.flush()

    def __foma_location_callback(self, location: FomaLocation):
        if not self.__write:
            return
        foma_img_location = Vector3(
            location.image.x * self.__room_frame_shape[1],
            location.image.y * self.__room_frame_shape[0],
            0
        )
        foma_world_location = Vector3(
            location.world.x * self.__map_frame_shape[0],
            location.world.y * self.__map_frame_shape[1],
            0
        )
        
        # Ensure coordinates stay within bounds
        x = np.clip(foma_world_location.x, 0, self.__room_frame_shape[1] - 1).astype(int)
        y = np.clip(foma_world_location.y, 0, self.__room_frame_shape[0] - 1).astype(int)
        cv2.circle(self.__room_map, (x, y), 5, (0, 255, 0), -1)

        timestamp = rospy.get_time()
        self.__foma_location_csv_writer.writerow([
                timestamp,
                foma_world_location.x,
                foma_world_location.y,
                foma_img_location.x,
                foma_img_location.y
        ])
        self.__foma_location_file.flush()

        frame = cv2.cvtColor(self.__room_map, cv2.COLOR_RGB2BGR)
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
        self.__room_map_writer.write(frame)

    def __foma_speed_callback(self, speed: Twist):
        self.__foma_speed = speed

    def _on_shutdown(self):
        self.loginfo("Shutting down WriterNode...")
        if self.__write:
            self.__stop_trial()

if __name__ == "__main__":
    rospy.init_node('ceiling_camera', anonymous=False)
    WriterNode()
