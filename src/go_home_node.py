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

        # Control parameters
        self.arrival_tol = 0.22             
        self.angle_tol = 2.5                # Parallel alignment target
        self.enabled = False
        self.max_speed = 0.35
        self.min_v = 0.18                   # Overcomes floor friction smoothly
        self.min_ang_v = 0.14               # Soft rotation minimum to absolutely eliminate overshoots
        self.state = 0                      # 0 = coarse center, 1 = align, 2 = fine center, 3 = verify

        # Fine-centering and verification tolerances
        self.fine_tol = 0.08
        self.verify_angle_tol = 3.5
        self.verify_dist_tol = 0.10

        # RANSAC/LLS parameters
        self.wall_inlier_tol = 0.05
        self.wall_min_inliers = 35
        self.wall_ransac_iters = 120

        # Debug/Estimation tracking state
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
        self._h_err = 0.0
        self.publish_status()
        if not self.enabled:
            self.stop_robot()
        return SetBoolResponse(success=True, message=f"GoHome status: {self.enabled}")

    def _fit_pca_line(self, pts):
        """Fit one 2D line using PCA (Linear Least Squares)."""
        centroid = pts.mean(axis=0)
        centered = pts - centroid
        cov = np.cov(centered.T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        wall_dir = eigvecs[:, int(np.argmax(eigvals))]
        wall_dir = wall_dir / max(np.linalg.norm(wall_dir), 1e-9)
        return wall_dir, centroid

    def _line_distances(self, pts, wall_dir, centroid):
        """Perpendicular distances from points to a line."""
        rel = pts - centroid
        return np.abs(rel[:, 0] * wall_dir[1] - rel[:, 1] * wall_dir[0])

    def _extract_wall_lines(self, pts):
        """Extract dominant wall lines from the full scan using RANSAC."""
        wall_lines = []
        remaining = pts.copy()

        for _ in range(4):
            if len(remaining) < self.wall_min_inliers:
                break

            best_idx = None
            best_count = 0

            for _ in range(self.wall_ransac_iters):
                ids = np.random.choice(len(remaining), 2, replace=False)
                p1, p2 = remaining[ids[0]], remaining[ids[1]]
                direction = p2 - p1
                norm = np.linalg.norm(direction)

                if norm < 1e-4:
                    continue

                direction = direction / norm
                dists = self._line_distances(remaining, direction, p1)
                inliers = np.where(dists < self.wall_inlier_tol)[0]

                if len(inliers) > best_count:
                    best_count = len(inliers)
                    best_idx = inliers

            if best_idx is None or best_count < self.wall_min_inliers:
                break

            inlier_pts = remaining[best_idx]
            wall_dir, centroid = self._fit_pca_line(inlier_pts)

            refined_dists = self._line_distances(remaining, wall_dir, centroid)
            refined_idx = np.where(refined_dists < self.wall_inlier_tol)[0]

            if len(refined_idx) < self.wall_min_inliers:
                break

            inlier_pts = remaining[refined_idx]
            wall_dir, centroid = self._fit_pca_line(inlier_pts)
            
            # Save line properties along with its inlier strength for structural anchoring
            wall_lines.append((wall_dir, centroid, len(refined_idx)))

            keep = np.ones(len(remaining), dtype=bool)
            keep[refined_idx] = False
            remaining = remaining[keep]

        return wall_lines

    def update_room_modeling(self, ranges):
        """
        Uses an anchor-based continuous tracking design to calculate heading error.
        Locks onto the strongest physical wall to prevent the 45 to -45 degree bouncing trap.
        """
        try:
            angles_rad = np.deg2rad(np.arange(len(ranges)))
            valid = (ranges > 0.15) & (ranges < 12.0) & np.isfinite(ranges)

            if np.sum(valid) < self.wall_min_inliers:
                return self._h_err, []

            x_rob = ranges[valid] * np.cos(angles_rad[valid])
            y_rob = ranges[valid] * np.sin(angles_rad[valid])
            pts_rob = np.column_stack([x_rob, y_rob])

            wall_lines = self._extract_wall_lines(pts_rob)

            if not wall_lines:
                return self._h_err, []

            # Find the Anchor Wall: the absolute most solid/longest line detected by RANSAC
            best_wall = max(wall_lines, key=lambda w: w[2])
            anchor_dir = best_wall[0]
            
            # Continuous absolute angle of our structural anchor line
            anchor_angle = np.rad2deg(np.arctan2(anchor_dir[1], anchor_dir[0])) % 360.0

            # Find orientation deviation to the nearest true clean global cardinal axis (0, 90, 180, 270)
            # This logic avoids the discontinuous modulo 'jump zone' entirely!
            cardinals = np.array([0.0, 90.0, 180.0, 270.0, 360.0])
            diffs = anchor_angle - cardinals
            closest_cardinal_idx = np.argmin(np.abs(diffs))
            heading_err = float(diffs[closest_cardinal_idx])

            # Clamp minor edge cases where it could still try to sit exactly on 45
            if abs(abs(heading_err) - 45.0) < 1.5:
                heading_err = 45.0  # Asserts definitive rotation direction to break symmetry instantly

            return heading_err, wall_lines

        except Exception as e:
            rospy.logerr(f"Modeling error: {e}")
            return self._h_err, []

    def _publish_room_viz(self, ranges, heading_error, wall_lines):
        """Renders the top-down environment ensuring walls lock to the room, not the robot."""
        viz_size = 400
        scale = viz_size / 24.0
        center = viz_size // 2
        canvas = np.zeros((viz_size, viz_size, 3), dtype=np.uint8)

        # Draw LiDAR points
        angles_rad = np.deg2rad(np.arange(len(ranges)))
        xs = ranges * np.cos(angles_rad)
        ys = ranges * np.sin(angles_rad)
        valid = (ranges > 0.15) & (ranges < 12.0) & np.isfinite(ranges)

        for i in np.where(valid)[0]:
            px = int(center + xs[i] * scale)
            py = int(center - ys[i] * scale)
            if 0 <= px < viz_size and 0 <= py < viz_size:
                canvas[py, px] = (130, 130, 130)

        # Draw fitted wall lines 
        for wall_dir, centroid, _ in wall_lines:
            t = 12.0
            p1 = centroid - t * wall_dir
            p2 = centroid + t * wall_dir

            pt1 = (int(center + p1[0] * scale), int(center - p1[1] * scale))
            pt2 = (int(center + p2[0] * scale), int(center - p2[1] * scale))
            cv2.line(canvas, pt1, pt2, (0, 255, 0), 1, cv2.LINE_AA)

        # Draw square robot boundary 
        r_w = int(0.35 * scale)
        cv2.rectangle(canvas, (center - r_w, center - r_w), (center + r_w, center + r_w), (255, 255, 255), 1)
        cv2.circle(canvas, (center, center), 4, (0, 0, 255), -1)

        state_names = {0: "COARSE_CENTER", 1: "ALIGN", 2: "FINE_CENTER", 3: "VERIFY"}
        info = [
            f"State: {self.state} ({state_names.get(self.state, '?')})",
            f"Enabled: {self.enabled}",
            f"H_err: {heading_error:.2f} deg",
            f"Dist: {self._dist_to_center:.3f} m",
            f"Walls Modeled: {len(wall_lines)}/4"
        ]

        for i, txt in enumerate(info):
            cv2.putText(canvas, txt, (5, 15 + i * 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow("GoHome Debug", canvas)
        cv2.waitKey(1)

        try:
            img_msg = self.cv_bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
            self.viz_pub.publish(img_msg)
        except Exception as e:
            rospy.logerr(f"Viz publish error: {e}")

    def _drive_toward_center(self, error_x, error_y, dist_to_center, cmd):
        """Vector proportional drive using safe linear velocity limits."""
        if dist_to_center < 1e-4:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            return

        kp_lin = 1.3
        target_speed = dist_to_center * kp_lin
        
        if target_speed < self.min_v:
            target_speed = self.min_v

        target_speed = min(target_speed, self.max_speed)

        cmd.linear.x = (error_x / dist_to_center) * target_speed
        cmd.linear.y = (error_y / dist_to_center) * target_speed

    def on_lidar(self, msg):
        # Convert ranges to meters
        ranges = np.array(msg.ranges, dtype=np.float32) / 1000.0

        # Real-time room modeling via continuous Anchor-RANSAC
        h_err, wall_lines = self.update_room_modeling(ranges)
        self._h_err = h_err
        self._publish_room_viz(ranges, h_err, wall_lines)

        if not self.enabled:
            return

        def get_dist(angle_deg):
            idx = [(int(angle_deg) + i) % len(ranges) for i in range(-10, 10)]
            vals = ranges[idx]
            valid = vals[(vals > 0.1) & (vals < 15.0) & np.isfinite(vals)]
            return np.median(valid) if len(valid) > 0 else None

        f = get_dist(270)
        b = get_dist(90)
        l = get_dist(0)
        r = get_dist(180)

        if None in [f, b, l, r]:
            return

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
                rospy.sleep(0.5)
                self.state = 1
                return

            self._drive_toward_center(error_x, error_y, dist_to_center, cmd)

        # --- State 1: Wall alignment ---
        elif self.state == 1:
            # Active breakout for the 45-degree deadlock
            if abs(h_err) == 45.0:
                rospy.logwarn("[GoHome] Absolute symmetric lock broken! Enforcing spin breakout.")
                cmd.angular.z = self.min_ang_v * 1.5
                self.cmd_pub.publish(cmd)
                return

            if abs(h_err) < self.angle_tol:
                rospy.loginfo(f"[GoHome] Alignment complete (Error: {h_err:.2f} deg). Switching to fine center.")
                self.stop_robot()
                rospy.sleep(0.5)
                self.state = 2
                return

            # Proportional controller with a lowered gain to ensure smooth approaching without oscillation
            kp_ang = 0.045
            v_ang = h_err * kp_ang

            if abs(v_ang) < self.min_ang_v:
                v_ang = self.min_ang_v if h_err > 0 else -self.min_ang_v

            cmd.angular.z = np.clip(v_ang, -0.25, 0.25)

        # --- State 2: Fine centering ---
        elif self.state == 2:
            if abs(h_err) > (self.angle_tol + 5.0):
                rospy.loginfo(f"[GoHome] Structural drift detected ({h_err:.2f} deg). Resetting to align state.")
                self.stop_robot()
                self.state = 1
                return

            if dist_to_center < self.fine_tol:
                rospy.loginfo("[GoHome] Fine center reached. Switching to verify.")
                self.stop_robot()
                rospy.sleep(0.5)
                self.state = 3
                return

            self._drive_toward_center(error_x, error_y, dist_to_center, cmd)

        # --- State 3: Verify pose ---
        elif self.state == 3:
            heading_ok = abs(h_err) < self.verify_angle_tol
            center_ok = dist_to_center < self.verify_dist_tol

            if heading_ok and center_ok:
                rospy.loginfo(f"[GoHome] Success! Pose completely modeled and verified (h_err={h_err:.2f} deg)")
                self._finish()
                return

            if not heading_ok:
                self.state = 1
                return
            if not center_ok:
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