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
        self.arrival_tol = 0.20
        self.angle_tol = 3.0
        self.enabled = False
        self.max_speed = 0.5
        self.min_v = 0.25
        self.min_ang_v = 0.16
        self.state = 0              # 0 = coarse center, 1 = align, 2 = fine center, 3 = verify

        # Fine-centering and verification tolerances
        self.fine_tol = 0.08
        self.verify_angle_tol = 4.0
        self.verify_dist_tol = 0.10

        # LLS/RANSAC parameters
        self.wall_inlier_tol = 0.07
        self.wall_min_inliers = 35
        self.wall_ransac_iters = 140

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

    def _fit_pca_line(self, pts):
        """Fit one 2D line using PCA."""
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
        """Extract dominant wall lines from the full scan."""
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
            wall_lines.append((wall_dir, centroid, None))

            keep = np.ones(len(remaining), dtype=bool)
            keep[refined_idx] = False
            remaining = remaining[keep]

        return wall_lines

    def get_heading_error(self, ranges):
        """Calculate heading error from dominant scan-wide wall lines."""
        wall_lines = []

        try:
            angles_rad = np.deg2rad(np.arange(len(ranges)))
            valid = (ranges > 0.15) & (ranges < 12.0) & np.isfinite(ranges)

            if np.sum(valid) < self.wall_min_inliers:
                return 0.0, wall_lines

            x_all = ranges[valid] * np.cos(angles_rad[valid])
            y_all = ranges[valid] * np.sin(angles_rad[valid])
            pts = np.column_stack([x_all, y_all])

            wall_lines = self._extract_wall_lines(pts)

            if not wall_lines:
                return 0.0, wall_lines

            errors = []

            for wall_dir, centroid, base in wall_lines:
                wall_angle = np.rad2deg(np.arctan2(wall_dir[1], wall_dir[0]))

                # Error to nearest room axis.
                err = (wall_angle + 45.0) % 90.0 - 45.0
                errors.append(err)

            if not errors:
                return 0.0, wall_lines

            heading_err = float(np.median(errors))
            return heading_err, wall_lines

        except Exception as e:
            rospy.logerr(f"LLS Error: {e}")
            return 0.0, wall_lines

    def _publish_room_viz(self, ranges, wall_lines):
        """Render top-down LiDAR scan with fitted walls."""
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
                cv2.circle(canvas, (px, py), 1, (200, 200, 200), -1)

        # Draw fitted wall lines
        for wall_dir, centroid, base_angle in wall_lines:
            t = 12.0
            p1 = centroid - t * wall_dir
            p2 = centroid + t * wall_dir

            pt1 = (int(center + p1[0] * scale), int(center - p1[1] * scale))
            pt2 = (int(center + p2[0] * scale), int(center - p2[1] * scale))

            cv2.line(canvas, pt1, pt2, (0, 255, 0), 1)

        # Draw robot at center
        cv2.circle(canvas, (center, center), 4, (0, 0, 255), -1)

        state_names = {
            0: "COARSE_CENTER",
            1: "ALIGN",
            2: "FINE_CENTER",
            3: "VERIFY",
        }

        info = [
            f"State: {self.state} ({state_names.get(self.state, '?')})",
            f"Enabled: {self.enabled}",
            f"H_err: {self._h_err:.2f} deg",
            f"Dist: {self._dist_to_center:.3f} m",
            f"Walls: {len(wall_lines)}/4",
        ]

        for i, txt in enumerate(info):
            cv2.putText(
                canvas,
                txt,
                (5, 15 + i * 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )

        cv2.imshow("GoHome Debug", canvas)
        cv2.waitKey(1)

        try:
            img_msg = self.cv_bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
            self.viz_pub.publish(img_msg)
        except Exception as e:
            rospy.logerr(f"Viz publish error: {e}")

    def _apply_axis_deadband(self, v):
        """Compensate motor deadband per axis."""
        if abs(v) < 1e-6:
            return 0.0

        if abs(v) < self.min_v:
            return math.copysign(self.min_v, v)

        return v

    def _drive_toward_center(self, error_x, error_y, dist_to_center, cmd):
        """Proportional drive with strict per-axis minimum."""
        kp_lin = 1.3

        vx = error_x * kp_lin
        vy = error_y * kp_lin

        vx = self._apply_axis_deadband(vx)
        vy = self._apply_axis_deadband(vy)

        speed = math.hypot(vx, vy)

        if speed > self.max_speed:
            factor = self.max_speed / speed
            vx *= factor
            vy *= factor

        cmd.linear.x = vx
        cmd.linear.y = vy

    def on_lidar(self, msg):
        # Convert ranges to meters
        ranges = np.array(msg.ranges, dtype=np.float32) / 1000.0

        h_err, wall_lines = self.get_heading_error(ranges)
        self._h_err = h_err
        self._publish_room_viz(ranges, wall_lines)

        if not self.enabled:
            return

        def get_dist(angle_deg):
            # Sample a small sector around the target angle.
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
                rospy.sleep(0.6)
                self.state = 1
                return

            self._drive_toward_center(error_x, error_y, dist_to_center, cmd)

        # --- State 1: Wall alignment ---
        elif self.state == 1:
            if abs(h_err) < self.angle_tol:
                rospy.loginfo(
                    f"[GoHome] Alignment complete (Error: {h_err:.2f} deg). Switching to fine center."
                )
                self.stop_robot()
                rospy.sleep(0.6)
                self.state = 2
                return

            kp_ang = 0.06
            v_ang = h_err * kp_ang

            if abs(v_ang) < self.min_ang_v:
                v_ang = self.min_ang_v if h_err > 0 else -self.min_ang_v

            cmd.angular.z = np.clip(v_ang, -0.5, 0.5)

        # --- State 2: Fine centering ---
        elif self.state == 2:
            if dist_to_center < self.fine_tol:
                rospy.loginfo("[GoHome] Fine center reached. Switching to verify.")
                self.stop_robot()
                rospy.sleep(0.6)
                self.state = 3
                return

            self._drive_toward_center(error_x, error_y, dist_to_center, cmd)

        # --- State 3: Verify pose ---
        elif self.state == 3:
            heading_ok = abs(h_err) < self.verify_angle_tol
            center_ok = dist_to_center < self.verify_dist_tol

            if heading_ok and center_ok:
                rospy.loginfo(
                    f"[GoHome] Verified (h_err={h_err:.2f} deg, dist={dist_to_center:.3f} m)"
                )
                self._finish()
                return

            if not heading_ok:
                rospy.loginfo(f"[GoHome] Heading drifted ({h_err:.2f} deg). Re-aligning.")
                self.state = 1
                return

            if not center_ok:
                rospy.loginfo(
                    f"[GoHome] Position drifted ({dist_to_center:.3f} m). Re-centering."
                )
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