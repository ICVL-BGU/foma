#!/usr/bin/env python3
import rospy
import cv2
from foma.srv import Write, WriteRequest, WriteResponse
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from abstract_node import AbstractNode
import datetime
import os
import numpy as np
from geometry_msgs.msg import Vector3
from foma.msg import FomaLocation
from etc.settings import *
import sys
import subprocess
import threading
import queue

class VideoWriterNode(AbstractNode):
    def __init__(self):
        # Get node name from ROS parameter (for multiple instances)
        node_name = rospy.get_name().split('/')[-1]
        super().__init__(node_name, f'Video Writer Node ({node_name})')

        # Get ROS parameters for this specific video file
        self.__topic = rospy.get_param('~topic')
        self.__msg_type = rospy.get_param('~msg_type')
        self.__filename = rospy.get_param('~filename')
        self.__fourcc = rospy.get_param('~fourcc', 'mp4v')
        self.__fps = rospy.get_param('~fps', 25)
        self.__frame_shape = tuple(rospy.get_param('~frame_shape', [640, 480]))
        self.__video_type = rospy.get_param('~video_type', 'standard')  # standard or trajectory_map
        
        # Reframe worker settings (only for this node)
        self.__enable_reframe_worker = rospy.get_param('~enable_reframe_worker', False)
        self.__reframe_worker_script = rospy.get_param('~reframe_worker_script', 'etc/reframe_worker.py')
        self.__reframe_force = rospy.get_param('~reframe_force', False)
        self.__reframe_no_ffmpeg = rospy.get_param('~reframe_no_ffmpeg', False)
        
        self.__write = False
        self.__init_attributes()

        if not self.__topic or not self.__msg_type:
            self.logerr("Missing topic or msg_type parameter")

        rospy.Service(f'{node_name}/write', Write, self.__set_write)
        
        self.bridge = CvBridge()
        rospy.on_shutdown(self.__on_shutdown)

    def __init_attributes(self):
        self.__video_writer = None
        self.__map_image = None  # For trajectory map
        self.__folder = ""
        self.__start_time = None
        self.__stop_time = None
        # Multi-threaded writing
        self.__frame_queue = None
        self.__writer_thread = None
        self.__writer_running = False
        self.__subscriber = None

    def __create_subscriber(self):
        """Create subscriber for configured topic. Called on start_trial."""
        if self.__msg_type == 'Image':
            return rospy.Subscriber(self.__topic, Image, self.__image_callback, queue_size=500, buff_size=2**28)
        if self.__msg_type == 'CompressedImage':
            return rospy.Subscriber(self.__topic, CompressedImage, self.__compressed_image_callback, queue_size=500, buff_size=2**27)
        if self.__msg_type == 'FomaLocation':
            return rospy.Subscriber(self.__topic, FomaLocation, self.__foma_location_callback, queue_size=1000)
        self.logerr(f"Unsupported message type for video: {self.__msg_type}")
        return None
    
    def __writer_thread_loop(self):
        """Background thread that writes frames to disk"""
        while self.__writer_running:
            try:
                frame = self.__frame_queue.get(timeout=0.1)
                if frame is not None and self.__video_writer is not None:
                    self.__video_writer.write(frame)
            except queue.Empty:
                continue
            except Exception as e:
                self.logerr(f"Error in writer thread: {e}")

    def __start_trial(self):
        if not os.path.exists(self.__folder):
            self.logerr(f"Trial folder does not exist: {self.__folder}")
            return

        # Create video writer
        filepath = os.path.join(self.__folder, self.__filename)
        fourcc = cv2.VideoWriter_fourcc(*self.__fourcc)
        
        self.__video_writer = cv2.VideoWriter(filepath, fourcc, self.__fps, self.__frame_shape)
        self.loginfo(f"Created video writer at {filepath}")
        
        # Initialize trajectory map if needed
        if self.__video_type == 'trajectory_map':
            self.__map_image = np.ones((self.__frame_shape[1], self.__frame_shape[0], 3), 
                                      dtype=np.uint8) * 255
            self.loginfo("Initialized trajectory map")
        
        # Start writer thread
        self.__frame_queue = queue.Queue(maxsize=500)
        self.__writer_running = True
        self.__writer_thread = threading.Thread(target=self.__writer_thread_loop, daemon=True)
        self.__writer_thread.start()
        self.loginfo("Started writer thread")

        self.__subscriber = self.__create_subscriber()
        if self.__subscriber is not None:
            self.loginfo(f"Subscribed to {self.__topic}")

    def __stop_trial(self):
        # Stop writer thread first
        if self.__writer_running:
            self.loginfo("Stopping writer thread...")
            self.__writer_running = False
            if self.__writer_thread is not None:
                self.__writer_thread.join(timeout=5.0)
                if self.__writer_thread.is_alive():
                    self.logwarn("Writer thread did not stop cleanly")
                else:
                    self.loginfo("Writer thread stopped")
        
        # Flush any remaining frames
        if self.__frame_queue is not None:
            remaining = self.__frame_queue.qsize()
            if remaining > 0:
                self.loginfo(f"Flushing {remaining} remaining frames...")
                while not self.__frame_queue.empty():
                    try:
                        frame = self.__frame_queue.get_nowait()
                        if frame is not None and self.__video_writer is not None:
                            self.__video_writer.write(frame)
                    except queue.Empty:
                        break
        
        rospy.sleep(0.1)  # Ensure all frames are processed
        self.loginfo("Releasing video writer.")
        
        if self.__video_writer is not None:
            self.__video_writer.release()
            self.__video_writer = None
            self.loginfo("Released video writer")
        
        self.__map_image = None
        self.__frame_queue = None
        
        # Run reframe worker if enabled
        if self.__enable_reframe_worker:
            self.__run_reframe_worker()

    def __run_reframe_worker(self):
        """Run the reframe worker script to process videos"""
        if self.__start_time is None or self.__stop_time is None:
            self.logwarn("Cannot run reframe worker: missing time information")
            return
        
        start_t = self.__start_time
        stop_t = self.__stop_time
        folder = self.__folder
        
        # Path to worker script
        worker_path = os.path.join(os.path.dirname(__file__), self.__reframe_worker_script)
        
        if not os.path.exists(worker_path):
            self.logwarn(f"Reframe worker script not found: {worker_path}")
            return
        
        start_s = float(start_t.to_sec())
        stop_s = float(stop_t.to_sec())
        
        # Log file for the worker
        logpath = os.path.join(folder, f"reframe.log")
        
        cmd = [
            sys.executable,  # Uses same Python interpreter
            worker_path,
            str(start_s),
            str(stop_s),
            folder,
            "--log", logpath
        ]
        
        # Add optional flags
        if self.__reframe_force:
            cmd.insert(-2, "--force")
        if self.__reframe_no_ffmpeg:
            cmd.insert(-2, "--no-ffmpeg")
        
        # Spawn detached process; send stdout/stderr to the log file
        try:
            logf = open(logpath, "a")
            p = subprocess.Popen(cmd, stdout=logf, stderr=logf, 
                               start_new_session=True, close_fds=True)
            self.loginfo(f"Spawned reframe worker pid={p.pid} log={logpath}")
        except Exception as e:
            self.logerr(f"Failed to spawn reframe worker: {e}")

    def __set_write(self, write: WriteRequest):
        if write.msg == "start":
            self.__start_time = write.stamp
            self.__folder = write.folder
            self.__start_trial()
            self.__write = True
            self.loginfo(f"Started video writing for subject {self.__folder}")

        elif write.msg == "stop" and self.__write:
            self.__stop_time = write.stamp
            self.__write = False
            # Unregister sub first so no new callbacks fire during teardown
            if self.__subscriber is not None:
                self.__subscriber.unregister()
                self.__subscriber = None
                self.loginfo(f"Unsubscribed from {self.__topic}")
            # Stop trial in separate thread to avoid blocking GUI
            stop_thread = threading.Thread(target=self.__stop_trial, daemon=True)
            stop_thread.start()
            self.loginfo("Stopping video writing (non-blocking)")

        return WriteResponse(success=True)

    def __image_callback(self, img_msg: Image):
        """Write Image message to video"""
        if not self.__write or self.__video_writer is None or self.__frame_queue is None:
            return
        
        # Convert and queue frame (fast operation)
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        
        try:
            self.__frame_queue.put(img, block=True, timeout=0.5)
        except queue.Full:
            self.logwarn("Frame queue full, dropping frame!")

    def __compressed_image_callback(self, img_msg: CompressedImage):
        """Write CompressedImage message to video"""
        if not self.__write or self.__video_writer is None or self.__frame_queue is None:
            return
        
        # Convert and queue frame (fast operation)
        img = self.bridge.compressed_imgmsg_to_cv2(img_msg)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        
        try:
            self.__frame_queue.put(img, block=True, timeout=0.5)
        except queue.Full:
            self.logwarn("Frame queue full, dropping frame!")

    def __foma_location_callback(self, location: FomaLocation):
        """Write FOMA location as trajectory map to video"""
        if not self.__write or self.__video_writer is None or self.__frame_queue is None:
            return
        if self.__map_image is None:
            self.logwarn("No map image initialized")
            return
        
        foma_world_location = Vector3(
            location.world.x * self.__frame_shape[0],
            location.world.y * self.__frame_shape[1],
            0
        )
        
        # Ensure coordinates stay within bounds
        x = int(np.clip(foma_world_location.x, 0, self.__frame_shape[0] - 1))
        y = int(np.clip(foma_world_location.y, 0, self.__frame_shape[1] - 1))
        
        # Draw on the persistent map
        cv2.circle(self.__map_image, (x, y), 5, (0, 255, 0), -1)
        
        # Create frame for video
        frame = cv2.cvtColor(self.__map_image, cv2.COLOR_RGB2BGR)
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
        
        # Queue frame for writing
        try:
            self.__frame_queue.put(frame, block=False)
        except queue.Full:
            self.logwarn("Frame queue full, dropping trajectory frame!")

    def __on_shutdown(self):
        self.loginfo("Shutting down VideoWriterNode...")
        if self.__subscriber is not None:
            self.__subscriber.unregister()
            self.__subscriber = None
        self.__stop_trial()

if __name__ == "__main__":
    rospy.init_node('video_writer_node', anonymous=True)
    VideoWriterNode()
    rospy.spin()
