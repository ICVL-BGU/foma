#!/usr/bin/env python3
import rospy
import math
import numpy as np
import cv2
from abstract_node import AbstractNode
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse
from cv_bridge import CvBridge

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
        self.state = 0              # 0 = coarse center, 1 = align, 2 = fine center, 3 = verify

        # Fine-centering and verification tolerances
        self.fine_tol = 0.08
        self.verify_angle_tol = 4.0
        self.verify_dist_tol = 0.10
        
        # Publishers
        self.cmd_pub = rospy.Publisher('motor_control/twist', Twist, queue_size=1, tcp_nodelay=True)
        self.pub_enabled = rospy.Publisher("go_home/enabled", Bool, queue_size=1, latch=True)
        self.viz_pub = rospy.Publisher('go_home/room_viz', Image, queue_size=1)
        self.cv_bridge = CvBridge()
        
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
        """ Calculate heading error via LLS; also return fitted wall lines for visualization """
        wall_lines = []
        try:
            angles_rad = np.deg2rad(np.arange(len(ranges)))
            x_all = ranges * np.cos(angles_rad)
            y_all = ranges * np.sin(angles_rad)
            
            errors = []
            # Sample around 4 main directions with ±25 deg window for stable wall angles
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
                b = (np.sum(y) - m * np.sum(x)) / n
                wall_angle = np.rad2deg(np.arctan(m))
                
                # Normalize error: wall surface is perpendicular to scan direction
                errors.append((wall_angle - base - 45) % 90 - 45)
                wall_lines.append((m, b, base))
            
            heading_err = np.mean(errors) if errors else 0
            return heading_err, wall_lines
        except Exception as e:
            rospy.logerr(f"LLS Error: {e}")
            return 0, wall_lines

    def _publish_room_viz(self, ranges, wall_lines):
        """ Render top-down LiDAR scan with LLS wall lines into an Image msg """
        viz_size = 400
        scale = viz_size / 24.0  # ~12m max range mapped to half the image
        center = viz_size // 2
        canvas = np.zeros((viz_size, viz_size, 3), dtype=np.uint8)

        # Draw LiDAR points
        angles_rad = np.deg2rad(np.arange(len(ranges)))
        xs = ranges * np.cos(angles_rad)
        ys = ranges * np.sin(angles_rad)
        valid = (ranges > 0.15) & (ranges < 12.0)
        for i in np.where(valid)[0]:
            px = int(center + xs[i] * scale)
            py = int(center - ys[i] * scale)
            if 0 <= px < viz_size and 0 <= py < viz_size:
                cv2.circle(canvas, (px, py), 1, (200, 200, 200), -1)

        # Draw fitted wall lines
        for m, b, base_angle in wall_lines:
            # Sample two far-apart x values to draw the line
            x1, x2 = -12.0, 12.0
            y1_w, y2_w = m * x1 + b, m * x2 + b
            p1 = (int(center + x1 * scale), int(center - y1_w * scale))
            p2 = (int(center + x2 * scale), int(center - y2_w * scale))
            cv2.line(canvas, p1, p2, (0, 255, 0), 1)

        # Draw robot at center
        cv2.circle(canvas, (center, center), 4, (0, 0, 255), -1)

        try:
            img_msg = self.cv_bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
            self.viz_pub.publish(img_msg)
        except Exception as e:
            rospy.logerr(f"Viz publish error: {e}")

    def on_lidar(self, msg):
        # Convert ranges to numpy array and scale to meters
        ranges = np.array(msg.ranges) / 1000.0

        # Always compute wall lines and publish visualization
        h_err, wall_lines = self.get_heading_error(ranges)
        self._publish_room_viz(ranges, wall_lines)

        if not self.enabled:
            return

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

        # --- State 0: Coarse centering ---
        if self.state == 0:
            if dist_to_center < self.arrival_tol:
                rospy.loginfo("[GoHome] Coarse center reached. Switching to align mode.")
                self.stop_robot()
                rospy.sleep(0.6) 
                self.state = 1
                return
            
            # Proportional control with minimum speed to ensure movement
            kp_lin = 1.3
            vx = error_x * kp_lin
            vy = error_y * kp_lin
            
            # Ensure minimum speed to prevent stalling
            if abs(vx) < self.min_v: vx = self.min_v if vx > 0 else -self.min_v
            if abs(vy) < self.min_v: vy = self.min_v if vy > 0 else -self.min_v
            
            cmd.linear.x = np.clip(vx, -self.max_speed, self.max_speed)
            cmd.linear.y = np.clip(vy, -self.max_speed, self.max_speed)

        # --- State 1: Wall alignment via LLS heading error ---
        elif self.state == 1:
            if abs(h_err) < self.angle_tol:
                rospy.loginfo(f"[GoHome] Alignment complete (Error: {h_err:.2f} deg). Switching to fine center.")
                self.stop_robot()
                rospy.sleep(0.6)
                self.state = 2
                return

            # Proportional angular control with minimum threshold
            kp_ang = 0.06
            v_ang = h_err * kp_ang
            
            if abs(v_ang) < self.min_ang_v:
                v_ang = self.min_ang_v if h_err > 0 else -self.min_ang_v
                
            cmd.angular.z = np.clip(v_ang, -0.5, 0.5)

        # --- State 2: Fine centering after alignment ---
        elif self.state == 2:
            if dist_to_center < self.fine_tol:
                rospy.loginfo("[GoHome] Fine center reached. Switching to verify.")
                self.stop_robot()
                rospy.sleep(0.6)
                self.state = 3
                return

            kp_lin = 1.3
            vx = error_x * kp_lin
            vy = error_y * kp_lin

            if abs(vx) < self.min_v: vx = self.min_v if vx > 0 else -self.min_v
            if abs(vy) < self.min_v: vy = self.min_v if vy > 0 else -self.min_v

            cmd.linear.x = np.clip(vx, -self.max_speed, self.max_speed)
            cmd.linear.y = np.clip(vy, -self.max_speed, self.max_speed)

        # --- State 3: Verify both alignment and centering ---
        elif self.state == 3:
            heading_ok = abs(h_err) < self.verify_angle_tol
            center_ok = dist_to_center < self.verify_dist_tol

            if heading_ok and center_ok:
                rospy.loginfo(f"[GoHome] Verified (h_err={h_err:.2f} deg, dist={dist_to_center:.3f} m)")
                self._finish()
                return

            # Re-enter alignment if heading drifted
            if not heading_ok:
                rospy.loginfo(f"[GoHome] Heading drifted ({h_err:.2f} deg). Re-aligning.")
                self.state = 1
                return

            # Re-enter fine centering if position drifted
            if not center_ok:
                rospy.loginfo(f"[GoHome] Position drifted ({dist_to_center:.3f} m). Re-centering.")
                self.state = 2
                return

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