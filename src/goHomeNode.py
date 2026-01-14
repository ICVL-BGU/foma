#!/usr/bin/env python3
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse

class GoHomeNode:
    def __init__(self): 
        self.arrival_tol = rospy.get_param("~arrival_tol", 0.05)
        self.enabled = False
        self.max_speed = 0.8
        self.kp = 2.0 

        # Publishers 
        self.cmd_pub = rospy.Publisher('motor_control/twist', Twist, queue_size=10)
        self.pub_enabled = rospy.Publisher("go_home/enabled", Bool, queue_size=1, latch=True)
        
        # Subscribers
        rospy.Subscriber("lidar/scans", LaserScan, self.on_lidar)

        # Services
        self.srv = rospy.Service("/go_home/enable", SetBool, self.on_enable)
        
        self.pub_enabled.publish(Bool(False))
        rospy.loginfo("GoHomeNode active and ready.")

    def on_enable(self, req):
        self.enabled = bool(req.data)
        self.pub_enabled.publish(Bool(self.enabled))
        if not self.enabled:
            self.stop_robot()
        return SetBoolResponse(success=True, message=f"GoHome active: {self.enabled}")

    def on_lidar(self, msg):
        if not self.enabled:
            return

        ranges = np.array(msg.ranges) / 1000.0 
        
        def get_dist(angle):
            idx = [(angle + i) % 360 for i in range(-15, 15)]
            vals = ranges[idx]
            valid = vals[(vals > 0.1) & (vals < 15.0)]
            return np.median(valid) if len(valid) > 0 else None

        l = get_dist(0)
        r = get_dist(180)
        b = get_dist(90)
        f = get_dist(270)

        if None in [f, b, l, r]:
            return

        error_x = (f - b) / 2.0
        error_y = (l - r) / 2.0
        
        dist_to_center = math.hypot(error_x, error_y)
        
        if dist_to_center < self.arrival_tol:
            self._finish()
            return

        cmd = Twist()
        cmd.linear.x = error_x * 1.5
        cmd.linear.y = error_y * 1.5

        speed = math.hypot(cmd.linear.x, cmd.linear.y)
        if speed > self.max_speed:
            scale = self.max_speed / speed
            cmd.linear.x *= scale
            cmd.linear.y *= scale

        self.cmd_pub.publish(cmd)


    def _get_avg(self, ranges, angle):
        indices = [(angle + i) % 360 for i in range(-5, 5)]
        vals = ranges[indices]
        valid = vals[(vals > 0.1) & (vals < 10.0)] 
        return np.mean(valid) if len(valid) > 0 else None

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def _finish(self):
        self.enabled = False
        self.pub_enabled.publish(Bool(False))
        self.stop_robot()

if __name__ == "__main__": 
    rospy.init_node("go_home_node")
    try:
        node = GoHomeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass