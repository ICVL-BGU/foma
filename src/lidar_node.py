#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from abstract_node import AbstractNode
from adafruit_rplidar import RPLidar, RPLidarException
import numpy as np
import threading
from std_msgs.msg import Int16MultiArray
from etc.lidar_revolution import bin_revolution, angle_increment_rad, SCAN_FULL_BINS

class LIDARNode(AbstractNode):
    def __init__(self):
        super().__init__('lidar', 'LIDAR')
        self.__lidar_pub = rospy.Publisher('lidar/scans', LaserScan, queue_size=1, tcp_nodelay=True)
        self.__blocked_pub = rospy.Publisher('lidar/blocked', Int16MultiArray, queue_size=1, tcp_nodelay=True)
        # Drift compensation, stage 0: one message per complete revolution,
        # stamped, in metres, NaN = no return. lidar/scans is unchanged.
        self.__scan_full_pub = rospy.Publisher('lidar/scan_full', LaserScan, queue_size=5, tcp_nodelay=True)
        self.__rev_samples = []
        self.__rev_start = None

        self.__lidar_offset = rospy.get_param('/LIDAR_OFFSET')
        safety_distance = rospy.get_param('/SAFETY_DISTANCE')
        self.__safety_distance_vector = safety_distance / np.cos(np.abs(np.arange(-45, 45) * np.pi / 180))

        self.__stop_event = threading.Event()
        try:
            self.lidar = RPLidar(None, rospy.get_param('/LIDAR_PORT'),
                                 baudrate=rospy.get_param('/LIDAR_PORT_SPEED'), timeout=3)
            self.lidar.stop()
            rospy.sleep(0.1)
            self.lidar.start()
            
        except RPLidarException as e:
            self.lidar = None
            self.logerr(e)

        self.scan_msg = LaserScan()
        self.scan_msg.ranges = np.ones(360)*15000

        if self.lidar:
            self._scan_thread = threading.Thread(target=self._scan_loop, daemon=True)
            self._scan_thread.start()

        rospy.on_shutdown(self.__on_shutdown)

    def _scan_loop(self):
        """Background thread: continuously reads from the lidar and updates scan_msg.ranges."""
        # Exit the loop if ROS is shutting down or if the stop event is set
        while not rospy.is_shutdown() and not self.__stop_event.is_set():
            try:
                for new_scan, quality, angle, distance in self.lidar.iter_measurements():
                    # _, angles, distances = zip(*scan)
                    if new_scan:
                        self.__publish_revolution()
                    if quality > 0:
                        if distance < 30:
                            continue
                        raw_angle = angle
                        angle = 359 - round(angle + self.__lidar_offset) % 360
                        self.scan_msg.ranges[angle] = distance
                        self.__rev_samples.append((raw_angle, distance))

            except RPLidarException as e:
                self.__drop_revolution()
                self.logerr(e)
                if str(e).startswith("Failed to connect to the sensor"):
                    self.lidar.disconnect()
                    rospy.sleep(0.1)
                    self.logwarn("Attempting to reconnect to LIDAR...")
                    self.lidar.connect()
                else:
                    self.lidar.stop()
                    rospy.sleep(0.1)
                    self.logwarn("Stopping LIDAR due to error, will attempt to restart.")
                    self.lidar.start()
                rospy.sleep(0.05)

            except Exception as e:
                self.__drop_revolution()
                self.logerr(f"Unexpected scan-loop error: {e}")
                rospy.sleep(0.05)

    def __drop_revolution(self):
        """Discard a partial revolution (sensor error / reconnect)."""
        self.__rev_samples = []
        self.__rev_start = None

    def __publish_revolution(self):
        """Publish the revolution that just ended on lidar/scan_full.

        header.stamp = time the revolution started, scan_time = its duration.
        The first (partial) revolution after start or an error is dropped.
        Failures are logged and swallowed so lidar/scans keeps flowing.
        """
        now = rospy.Time.now()
        samples, start = self.__rev_samples, self.__rev_start
        self.__rev_samples = []
        self.__rev_start = now
        if start is None or not samples:
            return
        try:
            msg = LaserScan()
            msg.header.stamp = start
            msg.header.frame_id = 'lidar'
            msg.angle_min = 0.0
            msg.angle_increment = angle_increment_rad(SCAN_FULL_BINS)
            msg.angle_max = msg.angle_increment * (SCAN_FULL_BINS - 1)
            msg.scan_time = (now - start).to_sec()
            msg.time_increment = 0.0
            msg.range_min = 0.03
            msg.range_max = 40.0
            msg.ranges = bin_revolution(samples, self.__lidar_offset, SCAN_FULL_BINS)
            self.__scan_full_pub.publish(msg)
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"LIDAR: scan_full publish failed: {e}")

    def run(self):
        """Main thread: publishes the latest scan_msg at a fixed rate."""
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.__lidar_pub.publish(self.scan_msg)
            self.publish_blocked()
            rate.sleep()

    def publish_blocked(self):
        # Reshape scans into (4, 90) to match safety_vec shape for elementwise comparison
        scans_reshaped = self.scan_msg.ranges.reshape(4, 90)
        distance_checks = (scans_reshaped < self.__safety_distance_vector)
        distance_checks = distance_checks.reshape(-1)  # back to 360‐length

        blocked_indices = np.where(distance_checks)[0]
        blocked_array = Int16MultiArray()
        blocked_array.data = blocked_indices.tolist()
        self.__blocked_pub.publish(blocked_array)

    def __on_shutdown(self):
        self.__stop_event.set()
        if self._scan_thread.is_alive():
            # join with timeout to avoid blocking shutdown indefinitely
            self._scan_thread.join(timeout=1.0)

        if self.lidar:
            self.logwarn("LIDARNode: Stopping sensor.")
            self.lidar.stop()
            self.logwarn("LIDARNode: Stopping motor.")
            self.lidar.stop_motor()
            self.logwarn("LIDARNode: Disconnecting serial.")
            self.lidar.disconnect()

if __name__ == "__main__":
    rospy.init_node('lidar_node')
    lidar_handler = LIDARNode()
    lidar_handler.run()
    rospy.spin()