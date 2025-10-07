#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from abstract_node import AbstractNode
from adafruit_rplidar import RPLidar, RPLidarException
import numpy as np

LIDAR_OFFSET = 90

class LIDARNode(AbstractNode):
    def __init__(self):
        super().__init__('lidar', 'LIDAR')
        self.lidar_pub = rospy.Publisher('lidar/scans', LaserScan, queue_size=10)
        try:
            self.lidar = RPLidar(None, '/dev/lidar', baudrate = 256000, timeout = 3)
            self.lidar.stop()
            rospy.sleep(0.1)
            self.lidar.start()
            
        except RPLidarException as e:
            self.lidar = None
            self.logwarn(e)

        self.scan_msg = LaserScan()
        self.scan_msg.ranges = np.ones(360)*15000

        if self.lidar:
            self._scan_thread = threading.Thread(target=self._scan_loop, daemon=True)
            self._scan_thread.start()

        rospy.on_shutdown(self.__on_shutdown)

    def _scan_loop(self):
        """Background thread: continuously reads from the lidar and updates scan_msg.ranges."""
        while not rospy.is_shutdown():
            try:
                for _, quality, angle, distance in self.lidar.iter_measurements():
                    # _, angles, distances = zip(*scan)
                    if quality > 0:
                        if distance < 30:
                            # self.logwarn(f"LIDAR distance < 30 for angle {angle}, skipping this measurement.")
                            continue
                        angle = 359 - round(angle + LIDAR_OFFSET) % 360
                        self.scan_msg.ranges[angle] = distance
                    # self.loginfo(f"Angle: {angle}, Distance: {distance}")

            except RPLidarException as e:
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
                # try to reconnect
                # try:
                #     self.lidar.reset()
                # except Exception as e2:
                #     self.logerr(f"Reconnect failed: {e2}")
                rospy.sleep(0.05)

            except Exception as e:
                self.logerr(f"Unexpected scan-loop error: {e}")
                rospy.sleep(0.05)

    def run(self):
        """Main thread: publishes the latest scan_msg at a fixed rate."""
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.lidar_pub.publish(self.scan_msg)
            rate.sleep()

    def __on_shutdown(self):
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