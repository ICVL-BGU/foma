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
        self.arrival_tol = rospy.get_param("~arrival_tol", 0.05)
        self.enabled = False
        self.max_speed = 0.8
        
        # מצב 0: ניווט למרכז מצב 1: סיבוב 90 מעלות
        self.state = 0 
        self.started_turning = False # דגל למניעת עצירה מוקדמת בסיבוב

        # פרמטרי יישור
        self.kp_yaw = 0.06  
        self.yaw_tolerance = 1.5 

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

    def get_heading_error(self, ranges_mm):
        # סינון רעשים
        valid_mask = (ranges_mm > 100) & (ranges_mm < 15000)
        if not np.any(valid_mask):
            return 0

        # הנקודה הכי קרובה היא הניצב לקיר
        closest_angle = np.where(valid_mask, ranges_mm, np.inf).argmin()
        
        # חישוב סטייה מהציר הקרוב ביותר
        error = (closest_angle + 45) % 90 - 45
        return error

    def on_lidar(self, msg):
        if not self.enabled:
            return

        ranges_mm = np.array(msg.ranges)
        heading_error = self.get_heading_error(ranges_mm)
        
        # פונקציה פנימית לקבלת מרחק במטרים מזווית מסוימת
        def get_dist(target_angle):
            idx = [(int(target_angle) + i) % 360 for i in range(-5, 5)]
            vals = ranges_mm[idx]
            valid = vals[(vals > 100) & (vals < 15000)]
            return np.median(valid) / 1000.0 if len(valid) > 0 else None

        # דגימת מרחקים לפי הצירים (במטרים)
        l = get_dist(0)   # שמאל
        r = get_dist(180) # ימין
        b = get_dist(90)  # אחורה
        f = get_dist(270) # קדימה

        if None in [f, b, l, r]:
            return

        # חישוב שגיאות מיקום במטרים
        error_x = (f - b) / 2.0
        error_y = (l - r) / 2.0
        dist_to_center = math.hypot(error_x, error_y)

        cmd = Twist()

        if self.state == 0: 
            # תיקון זוויתי (תמיד פעיל)
            cmd.angular.z = heading_error * self.kp_yaw 
            
            # אם הזווית גדולה מ-30, רק מסתובבים. אם קטנה - נוסעים למרכז
            if abs(heading_error) < 30:
                cmd.linear.x = error_x * 1.5
                cmd.linear.y = error_y * 1.5
                
                # הגבלת מהירות
                speed = math.hypot(cmd.linear.x, cmd.linear.y)
                if speed > self.max_speed:
                    scale = self.max_speed / speed
                    cmd.linear.x *= scale
                    cmd.linear.y *= scale
            else:
                cmd.linear.x = 0
                cmd.linear.y = 0

            # בדיקה אם הגענו למרכז ואנחנו מיושרים
            if dist_to_center < self.arrival_tol and abs(heading_error) < self.yaw_tolerance:
                rospy.loginfo("Reached center and aligned. Starting 90-degree turn.")
                self.state = 1
                self.started_turning = False

        elif self.state == 1: 
            # פקודת סיבוב קבועה
            cmd.angular.z = 0.5 
            
            if abs(heading_error) > 10:
                self.started_turning = True
            
            if self.started_turning and abs(heading_error) < self.yaw_tolerance:
                rospy.loginfo("90-degree turn completed.")
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
        rospy.loginfo("GoHome: Finished successfully.")

if __name__ == "__main__": 
    rospy.init_node("go_home_node")
    node = GoHomeNode()
    rospy.spin()