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
        except RPLidarException as e:
            self.lidar = None
            rospy.logwarn(e)
        self.scanning = False
        self.scans = np.ones(360)*15000
        rospy.on_shutdown(self.__on_shutdown)

    def run(self):
        while not rospy.is_shutdown():
            try:
                # if not self._system_on:
                #     continue

                for scan in self.lidar.iter_scans():
                    _, angles, distances = zip(*scan)
                    angles = 359 - ((np.floor(angles).astype(int) + LIDAR_OFFSET) % 360)
                    try:
                        self.scans[angles] = distances
                    except:
                        for i, angle in enumerate(angles):
                            self.scans[angle] = distances[i]

                    laser_scan = LaserScan()
    
                    # Assuming angles are given in degrees, converting to radians for ROS
                    # rad_angles = np.deg2rad(angles)
                    
                    # Setting up some LaserScan fields based on the data you have
                    laser_scan.header.stamp = rospy.Time.now()
                    laser_scan.header.frame_id = 'laser_frame'  # Change this to your LIDAR's frame ID if different
                    # laser_scan.angle_min = min(rad_angles)
                    # laser_scan.angle_max = max(rad_angles)
                    # laser_scan.angle_increment = (laser_scan.angle_max - laser_scan.angle_min) / len(angles)
                    # laser_scan.range_min = 0.1  # Adjust as per your LIDAR's specifications
                    # laser_scan.range_max = 30.0  # Adjust as per your LIDAR's specifications
                    
                    # Populate the distances into the ranges field
                    laser_scan.ranges = self.scans
                    # print(laser_scan.ranges)
                    # print(np.argmin(laser_scan.ranges))
                    self.lidar_pub.publish(laser_scan)
            
            except RPLidarException as e:
                rospy.logerr(e)
                self.lidar.disconnect()
                rospy.sleep(0.005)
                self.lidar.connect()
                rospy.sleep(0.005)

            except AttributeError as e:
                self._system_on = False
                rospy.logerr(e)
            
            except Exception as e:
                rospy.logerr(e)

            except:
                rospy.logerr("Something happend.")
            
            rospy.sleep(0.05)

    def __on_shutdown(self):
        if self.lidar:
            rospy.logwarn("LIDARNode: Stopping sensor.")
            self.lidar.stop()
            rospy.logwarn("LIDARNode: Stopping motor.")
            self.lidar.stop_motor()
            rospy.logwarn("LIDARNode: Disconnecting serial.")
            self.lidar.disconnect()

if __name__ == "__main__":
    rospy.init_node('lidar_node')
    
    lidar_handler = LIDARNode()
    lidar_handler.run()
    rospy.spin()