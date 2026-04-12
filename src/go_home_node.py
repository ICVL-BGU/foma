#!/usr/bin/env python3
import rospy
import math
import numpy as np
from abstract_node import AbstractNode
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse

class GoHomeNode(AbstractNode):
    def __init__(self): 
        super().__init__('go_home', 'Go Home')
        # הגדלת טווח טעות ל-12 ס"מ כדי למנוע היתקעות
        self.arrival_tol = 0.12 
        self.enabled = False
        self.max_speed = 0.8
        
        self.state = 0 
        self.started_turning = False 

        self.cmd_pub = rospy.Publisher('motor_control/twist', Twist, queue_size=1, tcp_nodelay=True)
        self.pub_enabled = rospy.Publisher("go_home/enabled", Bool, queue_size=1, latch=True)
        
        rospy.Subscriber("lidar/scans", LaserScan, self.on_lidar, queue_size=1, tcp_nodelay=True)
        self.srv = rospy.Service("/go_home/enable", SetBool, self.on_enable)
        
        self.pub_enabled.publish(Bool(False))

    def on_enable(self, req):
        self.enabled = bool(req.data)
        self.state = 0
        self.started_turning = False
        self.pub_enabled.publish(Bool(self.enabled))
        if not self.enabled:
            self.stop_robot()
        return SetBoolResponse(success=True, message=f"GoHome active: {self.enabled}")

    def get_heading_error(self, ranges):
        valid_mask = (ranges > 0.1) & (ranges < 15.0)
        if not np.any(valid_mask): return 0
        temp_ranges = np.where(valid_mask, ranges, np.inf)
        closest_idx = np.argmin(temp_ranges)
        return (closest_idx + 45) % 90 - 45

    def on_lidar(self, msg):
        if not self.enabled:
            return

        ranges = np.array(msg.ranges) / 1000.0 
        
        def get_dist(angle):
            idx = [(int(angle) + i) % 360 for i in range(-15, 15)]
            vals = ranges[idx]
            valid = vals[(vals > 0.1) & (vals < 15.0)]
            return np.median(valid) if len(valid) > 0 else None

        l, r, b, f = get_dist(0), get_dist(180), get_dist(90), get_dist(270)
        if None in [f, b, l, r]: return

        error_x = (f - b) / 2.0
        error_y = (l - r) / 2.0
        dist_to_center = math.hypot(error_x, error_y)
        
        cmd = Twist()

        # שלב 0: הגעה מהירה למרכז
        if self.state == 0:
            # אם הגענו לטווח ה-12 ס"מ, עוצרים ועוברים לשלב הבא
            if dist_to_center < self.arrival_tol:
                self.state = 1
                rospy.loginfo("Arrived at Center Zone")
            else:
                # חישוב מהירות עם סף מינימלי גבוה (0.15) כדי שלא יזחל
                vx = error_x * 2.0
                vy = error_y * 2.0
                
                # החלת מהירות מינימלית "חצופה" כדי שיזוז מהר
                min_v = 0.15
                if abs(vx) < min_v: vx = min_v if error_x > 0 else -min_v
                if abs(vy) < min_v: vy = min_v if error_y > 0 else -min_v

                cmd.linear.x = np.clip(vx, -self.max_speed, self.max_speed)
                cmd.linear.y = np.clip(vy, -self.max_speed, self.max_speed)

        # שלב 1: התיישרות החלטית
        elif self.state == 1:
            h_err = self.get_heading_error(ranges)
            if abs(h_err) > 4.0: # טווח טעות של 4 מעלות ליישור
                cmd.angular.z = 0.35 if h_err > 0 else -0.35
            else:
                self.state = 2
                rospy.loginfo("Aligned. Turning 90...")

        # שלב 2: סיבוב 90 מעלות
        elif self.state == 2:
            h_err = self.get_heading_error(ranges)
            cmd.angular.z = 0.45 # מהירות סיבוב מורגשת
            if abs(h_err) > 25:
                self.started_turning = True
            
            if self.started_turning and abs(h_err) < 5.0:
                self._finish()
                return

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def _finish(self):
        self.enabled = False
        self.state = 0
        self.pub_enabled.publish(Bool(False))
        self.stop_robot()
        rospy.loginfo("Done.")

if __name__ == "__main__":
    rospy.init_node("go_home_node")
    try:
        node = GoHomeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass