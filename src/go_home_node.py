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

        # Debug viz state
        self._dist_to_center = 0.0
        self._h_err = 0.0
        
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
        """ Calculate heading error via PCA wall fitting; return wall lines for visualization """
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

                # PCA wall direction (robust to any orientation, unlike OLS y=mx+b)
                pts = np.column_stack([x, y])
                centroid = pts.mean(axis=0)
                centered = pts - centroid
                cov = np.cov(centered.T)
                eigvals, eigvecs = np.linalg.eigh(cov)
                # Largest eigenvector = wall direction
                wall_dir = eigvecs[:, 1]
                wall_angle = np.rad2deg(np.arctan2(wall_dir[1], wall_dir[0]))

                # Normalize error to [-45, 45]; handles PCA sign ambiguity via mod 90
                errors.append((wall_angle - base - 45) % 90 - 45)
                wall_lines.append((wall_dir, centroid, base))
            
            heading_err = np.mean(errors) if errors else 0
            return heading_err, wall_lines
        except Exception as e:
            rospy.logerr(f"LLS Error: {e}")
            return 0, wall_lines

    def _publish_room_viz(self, ranges, wall_lines):
        """ Render top-down LiDAR scan with wall lines; show as pop-up and publish to topic """
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

        # Draw fitted wall lines from PCA direction + centroid
        for wall_dir, centroid, base_angle in wall_lines:
            t = 12.0
            p1 = centroid - t * wall_dir
            p2 = centroid + t * wall_dir
            pt1 = (int(center + p1[0] * scale), int(center - p1[1] * scale))
            pt2 = (int(center + p2[0] * scale), int(center - p2[1] * scale))
            cv2.line(canvas, pt1, pt2, (0, 255, 0), 1)

        # Draw robot at center
        cv2.circle(canvas, (center, center), 4, (0, 0, 255), -1)

        # Debug overlay: state, heading error, distance
        state_names = {0: "COARSE_CENTER", 1: "ALIGN", 2: "FINE_CENTER", 3: "VERIFY"}
        info = [
            f"State: {self.state} ({state_names.get(self.state, '?')})",
            f"Enabled: {self.enabled}",
            f"H_err: {self._h_err:.2f} deg",
            f"Dist: {self._dist_to_center:.3f} m",
            f"Walls: {len(wall_lines)}/4",
        ]
        for i, txt in enumerate(info):
            cv2.putText(canvas, txt, (5, 15 + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

        # Independent pop-up debug window
        cv2.imshow("GoHome Debug", canvas)
        cv2.waitKey(1)

        # Also publish to ROS topic
        try:
            img_msg = self.cv_bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
            self.viz_pub.publish(img_msg)
        except Exception as e:
            rospy.logerr(f"Viz publish error: {e}")

    def _drive_toward_center(self, error_x, error_y, dist_to_center, cmd):
        """ Vector-based proportional drive with min speed as magnitude, not per-axis """
        kp_lin = 1.3
        vx = error_x * kp_lin
        vy = error_y * kp_lin
        speed = math.hypot(vx, vy)

        # Apply minimum speed as vector magnitude to overcome friction
        if speed < self.min_v and dist_to_center > 0.01:
            factor = self.min_v / max(speed, 1e-6)
            vx *= factor
            vy *= factor
            speed = self.min_v

        # Clamp to max speed while preserving direction
        if speed > self.max_speed:
            factor = self.max_speed / speed
            vx *= factor
            vy *= factor

        cmd.linear.x = vx
        cmd.linear.y = vy

    def on_lidar(self, msg):
        # Convert ranges to numpy array and scale to meters
        ranges = np.array(msg.ranges) / 1000.0

        # Always compute wall lines and publish visualization
        h_err, wall_lines = self.get_heading_error(ranges)
        self._h_err = h_err
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
        self._dist_to_center = dist_to_center
        
        cmd = Twist()

        # --- State 0: Coarse centering ---
        if self.state == 0:
            if dist_to_center < self.arrival_tol:
                rospy.loginfo("[GoHome] Coarse center reached. Switching to align mode.")
                self.stop_robot()
                rospy.sleep(0.6) 
                self.state = 1
                return
            
            self._drive_toward_center(error_x, error_y, dist_to_center, cmd)

        # --- State 1: Wall alignment via PCA heading error ---
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

            self._drive_toward_center(error_x, error_y, dist_to_center, cmd)

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