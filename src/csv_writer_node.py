#!/usr/bin/env python3
import rospy
import csv
from foma.srv import Write, WriteRequest, WriteResponse
from abstract_node import AbstractNode
import datetime
import os
from geometry_msgs.msg import Twist, Vector3, TwistStamped
from foma.msg import FomaLocation
import math
from etc.settings import *

class CSVWriterNode(AbstractNode):
    def __init__(self):
        # Get node name from ROS parameter (for multiple instances)
        node_name = rospy.get_name().split('/')[-1]
        super().__init__(node_name, f'CSV Writer Node ({node_name})')

        # Get ROS parameters for this specific CSV file
        self.__topic = rospy.get_param('~topic')
        self.__msg_type = rospy.get_param('~msg_type')
        self.__filename = rospy.get_param('~filename')
        self.__fields = rospy.get_param('~fields')
        self.__data_type = rospy.get_param('~data_type', 'generic')  # foma_location, fish_state, foma_speed, or generic
        
        self.__write = False
        self.__init_attributes()

        # Subscribe to the configured topic
        self.__setup_subscriber()

        rospy.Service(f'{node_name}/write', Write, self.__set_write)
        rospy.on_shutdown(self.__on_shutdown)

    def __init_attributes(self):
        self.__csv_file = None
        self.__csv_writer = None
        self.__folder = ""

    def __setup_subscriber(self):
        """Setup subscriber for the configured topic"""
        if not self.__topic or not self.__msg_type:
            self.logerr("Missing topic or msg_type parameter")
            return
        
        # Create callback based on message type and data type
        # CSV writer needs large queue to capture ALL data without dropping
        if self.__msg_type == 'FomaLocation':
            rospy.Subscriber(self.__topic, FomaLocation, self.__foma_location_callback, queue_size=1000)
        elif self.__msg_type == 'TwistStamped':
            if self.__data_type == 'fish_state':
                rospy.Subscriber(self.__topic, TwistStamped, self.__fish_state_callback, queue_size=1000)
            elif self.__data_type == 'foma_speed':
                rospy.Subscriber(self.__topic, TwistStamped, self.__foma_speed_callback, queue_size=1000)
            else:
                rospy.Subscriber(self.__topic, TwistStamped, self.__generic_twist_callback, queue_size=1000)
        else:
            self.logerr(f"Unsupported message type: {self.__msg_type}")

    def __start_trial(self):
        # Use the folder path provided by GUI (subject_id parameter now contains folder path)
        
        if not os.path.exists(self.__folder):
            self.logerr(f"Trial folder does not exist: {self.__folder}")
            return

        # Create CSV file
        filepath = os.path.join(self.__folder, self.__filename)
        self.__csv_file = open(filepath, 'a', newline='')
        self.__csv_writer = csv.writer(self.__csv_file)
        
        # Write header if file is empty
        if os.stat(filepath).st_size == 0:
            self.__csv_writer.writerow(self.__fields)
            self.__csv_file.flush()
        
        self.loginfo(f"Created CSV writer at {filepath}")

    def __stop_trial(self):
        rospy.sleep(0.1)  # Ensure all data is processed
        self.loginfo("Closing CSV file.")
        
        if self.__csv_file is not None:
            self.__csv_file.close()
            self.__csv_file = None
            self.__csv_writer = None

    def __set_write(self, write: WriteRequest):
        if write.msg == "start":
            self.__folder = write.folder
            self.__start_trial()
            self.__write = True
            self.loginfo(f"Started CSV writing for subject {self.__folder}")

        elif write.msg == "stop" and self.__write:
            self.__write = False
            self.__stop_trial()
            self.loginfo("Stopped CSV writing")

        return WriteResponse(success=True)

    def __fish_state_callback(self, state: TwistStamped):
        """Write fish state data to CSV"""
        if not self.__write or self.__csv_writer is None:
            return
        if state.twist.angular == Vector3(0, 0, 0):
            return
        
        row = [
            state.header.stamp.to_sec(),
            state.twist.linear.x,
            state.twist.linear.y,
            math.degrees(math.atan2(state.twist.angular.y, state.twist.angular.x))
        ]
        self.__csv_writer.writerow(row)
        self.__csv_file.flush()

    def __foma_location_callback(self, location: FomaLocation):
        """Write FOMA location data to CSV"""
        if not self.__write or self.__csv_writer is None:
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
        
        row = [
            location.header.stamp.to_sec(),
            foma_world_location.x,
            foma_world_location.y,
            foma_img_location.x,
            foma_img_location.y
        ]
        self.__csv_writer.writerow(row)
        self.__csv_file.flush()

    def __foma_speed_callback(self, speed_msg: TwistStamped):
        """Write FOMA speed data to CSV"""
        if not self.__write or self.__csv_writer is None:
            return
        
        row = [
            speed_msg.header.stamp.to_sec(),
            speed_msg.twist.linear.x,
            speed_msg.twist.linear.y,
            speed_msg.twist.angular.z
        ]
        self.__csv_writer.writerow(row)
        self.__csv_file.flush()

    def __generic_twist_callback(self, msg: TwistStamped):
        """Generic callback for TwistStamped messages"""
        if not self.__write or self.__csv_writer is None:
            return
        
        row = [
            msg.header.stamp.to_sec(),
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z
        ]
        self.__csv_writer.writerow(row)
        self.__csv_file.flush()

    def __on_shutdown(self):
        self.loginfo("Shutting down CSVWriterNode...")
        self.__stop_trial()

if __name__ == "__main__":
    rospy.init_node('csv_writer_node', anonymous=True)
    CSVWriterNode()
    rospy.spin()
