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
        
        # Parameters for control logic
        self.arrival_tol = 0.15     
        self.angle_tol = 3.0        
        self.enabled = False
        self.max_speed = 0.5         
        self.min_v = 0.18            
        self.min_ang_v = 0.16        
        self.state = 0              # 0 = moving to center, 1 = aligning to walls
        
        # Publishers
        self.cmd_pub = rospy.Publisher('motor_control/twist', Twist, queue_size=1, tcp_nodelay=True)
        self.pub_enabled = rospy.Publisher("go_home/enabled", Bool, queue_size=1, latch=True)
        
        # Subscribers
        rospy.Subscriber("lidar/scans", LaserScan, self.on_lidar, queue_size=1, tcp_nodelay=True)
        
        # Services
        self.srv = rospy.Service("/go_home/enable", SetBool, self.on_enable)
        
        self.publish_status()
        rospy.loginfo("[GoHome] Node Started Successfully")

    def publish_status(self):
        msg = Bool()
        msg.data = self.enabled
        self.pub_enabled.publish(msg)

    def on_enable(self, req):
        self.enabled = bool(req.data)
        self.state = 0
        self.publish_status()
        if not self.enabled:
            self.stop_robot()
        return SetBoolResponse(success=True, message=f"GoHome status: {self.enabled}")

    def get_heading_error(self, ranges):
        """ Calculate heading error based on LLS with broad sampling for stability """
        try:
            angles_rad = np.deg2rad(np.arange(len(ranges)))
            x_all = ranges * np.cos(angles_rad)
            y_all = ranges * np.sin(angles_rad)
            
            errors = []
            # Sample around 4 main directions (0, 90, 180, 270) with a ±25 degree window to get stable wall angles
            for base in [0, 90, 180, 270]:
                idx = np.arange(base - 25, base + 25) % len(ranges)
                mask = (ranges[idx] > 0.15) & (ranges[idx] < 12.0)
                
                if np.sum(mask) < 40: continue 
                
                x, y = x_all[idx][mask], y_all[idx][mask]
                n = len(x)
                
                # Least Squares Linear Regression
                denom = (n * np.sum(x**2) - (np.sum(x))**2)
                if abs(denom) < 1e-5: continue
                
                m = (n * np.sum(x*y) - np.sum(x)*np.sum(y)) / denom
                wall_angle = np.rad2deg(np.arctan(m))
                
                # Normalize error to [-45, 45] range
                errors.append((wall_angle - base + 45) % 90 - 45)
            
            return np.mean(errors) if errors else 0
        except Exception as e:
            rospy.logerr(f"LLS Error: {e}")
            return 0

    def on_lidar(self, msg):
        if not self.enabled:
            return

        # Convert ranges to numpy array and scale to meters
        ranges = np.array(msg.ranges) / 1000.0 
        
        def get_dist(angle_deg):
            # Sample a ±10 degree window around the target angle for robustness
            idx = [(int(angle_deg) + i) % len(ranges) for i in range(-10, 10)]
            vals = ranges[idx]
            valid = vals[(vals > 0.1) & (vals < 15.0)]
            return np.median(valid) if len(valid) > 0 else None

        # Get distances in 4 cardinal directions with robust sampling
        f, b, l, r = get_dist(270), get_dist(90), get_dist(0), get_dist(180)
        
        if None in [f, b, l, r]: 
            return

        # Calculate errors (positive error_x means robot is to the right of center, positive error_y means robot is in front of center) 
        error_x = (f - b) / 2.0
        error_y = (l - r) / 2.0
        dist_to_center = math.hypot(error_x, error_y)
        
        cmd = Twist()

        # Move towards center until within arrival tolerance, then switch to alignment mode
        if self.state == 0:
            if dist_to_center < self.arrival_tol:
                rospy.loginfo("[GoHome] Center reached. Switching to align mode.")
                self.stop_robot()
                rospy.sleep(0.6) 
                self.state = 1
                return
            
            # Proportional control with minimum speed to ensure movement, but only if we have a significant error (to avoid oscillations near the target)
            kp_lin = 1.3
            vx = error_x * kp_lin
            vy = error_y * kp_lin
            
            # ensure minimum speed to prevent stalling, but only if we have a significant error (to avoid oscillations near the target)
            if abs(vx) < self.min_v: vx = self.min_v if vx > 0 else -self.min_v
            if abs(vy) < self.min_v: vy = self.min_v if vy > 0 else -self.min_v
            
            cmd.linear.x = np.clip(vx, -self.max_speed, self.max_speed)
            cmd.linear.y = np.clip(vy, -self.max_speed, self.max_speed)

        # In alignment mode, we only rotate to minimize heading error. We rely on the fact that if we are close to the center, small rotations will not cause us to drift away significantly. The LLS-based heading error should guide us to align with the walls.
        elif self.state == 1:
            h_err = self.get_heading_error(ranges)
            
            # If the heading error is small enough, we consider ourselves aligned and stop the robot
            if abs(h_err) < self.angle_tol:
                rospy.loginfo(f"[GoHome] Alignment complete (Error: {h_err:.2f} deg)")
                self._finish()
                return

            # Proportional control for angular velocity with a minimum threshold to ensure we actually rotate, but only if we have a significant error (to avoid oscillations near the target)
            kp_ang = 0.06
            v_ang = h_err * kp_ang
            
            if abs(v_ang) < self.min_ang_v:
                v_ang = self.min_ang_v if h_err > 0 else -self.min_ang_v
                
            cmd.angular.z = np.clip(v_ang, -0.5, 0.5)

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def _finish(self):
        self.enabled = False
        self.state = 0
        self.publish_status()
        self.stop_robot()
        rospy.loginfo("[GoHome] Task finished successfully.")

if __name__ == "__main__":
    rospy.init_node("go_home_node")
    try:
        node = GoHomeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass