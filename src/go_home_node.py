#!/usr/bin/env python3
"""GoHomeNode -- single-wall commitment with pulse-based alignment.

Operates inside a square / symmetric room.  Uses RANSAC + PCA to detect
wall lines from a 360-degree LiDAR scan, then locks onto ONE specific
physical wall as the anchor.  The robot aligns to that anchor, moves to
the room centre, verifies pose, and stops.

Key design -- strict commitment
---------------------------------
1.  During ST_ACQUIRE, choose ONE physical wall and lock ONE target axis.
2.  Parallel pairs are used only as evidence for room orientation;
    the tracked anchor is always a single physical wall.
3.  The target axis is NEVER recomputed after acquisition.
4.  Near 45 degrees a deterministic rule picks a fixed axis to avoid
    noisy nearest-axis selection.
5.  Anchor matching uses angle + signed_distance + centroid with hard
    reject thresholds so the robot never silently switches walls.

Key design -- three angular zones
-----------------------------------
1.  ALIGNED (abs(f_err) <= angle_tol): stop, confirm N frames.
2.  SOFT (angle_tol < abs(f_err) <= soft_angle_zone): pulse above
    static friction threshold, then pause and re-measure.
3.  FAR (abs(f_err) > soft_angle_zone): P controller + friction floor.
"""
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


# ====================================================================
#  Angle helpers (undirected wall orientations, period 180)
# ====================================================================

def _line_angle_deg(wall_dir):
    """PCA direction vector -> undirected line angle in [0, 180)."""
    return np.rad2deg(np.arctan2(wall_dir[1], wall_dir[0])) % 180.0


def _angle_diff_mod180(a, b):
    """Signed angular difference (a - b) on the mod-180 circle.
    Result in (-90, +90]."""
    return (a - b + 90.0) % 180.0 - 90.0


def _heading_error_to_locked_axis(anchor_angle, target_axis):
    """Smallest signed rotation from anchor_angle to target_axis.
    Both are undirected line orientations (mod 180).  Result in (-90, +90]."""
    return _angle_diff_mod180(anchor_angle, target_axis)


# ====================================================================

class GoHomeNode(AbstractNode):
    """ROS node: detect room walls, lock single anchor, align, centre, stop.

    Publishers   motor_control/twist, go_home/enabled, go_home/room_viz
    Subscriber   lidar/scans
    Service      /go_home/enable  (SetBool)
    """

    # -- State machine -----------------------------------------------
    ST_ACQUIRE = 0
    ST_ALIGN   = 1
    ST_CENTER  = 2
    ST_VERIFY  = 3

    # -- Confirmation ------------------------------------------------
    TRACKING_LOST_LIMIT   = 6
    ALIGN_CONFIRM_FRAMES  = 5
    CENTER_CONFIRM_FRAMES = 3

    def __init__(self):
        super().__init__('go_home', 'Go Home')

        # -- Motion limits -------------------------------------------
        self.max_speed = 0.60
        self.max_ang_v = 0.45
        self.min_v     = 0.35
        self.min_ang_v = 0.24

        # -- Alignment -----------------------------------------------
        self.angle_tol       = 3.5
        self.soft_angle_zone = 8.0
        self.kp_ang          = 0.06
        self.angular_sign    = 1.0       # flip to -1.0 if h_err grows

        # -- Soft-zone pulse -----------------------------------------
        self.soft_pulse_ang_v  = 0.20
        self.soft_pulse_frames = 2
        self.soft_pause_frames = 3

        # -- Centering -----------------------------------------------
        # NOTE: This centering uses cmd.linear.x AND cmd.linear.y
        # (holonomic / omnidirectional drive).  If your robot is
        # differential-drive and does not support linear.y, you must
        # replace this with rotate-then-drive centering.
        self.center_tol      = 0.12
        self.verify_dist_tol = 0.16
        self.verify_angle_tol = 5.0
        self.kp_lin          = 2.5

        # -- RANSAC / PCA --------------------------------------------
        self.wall_inlier_tol   = 0.05
        self.wall_min_inliers  = 35
        self.wall_ransac_iters = 120

        # -- Anchor tracking weights ---------------------------------
        self.anchor_angle_weight    = 2.0
        self.anchor_distance_weight = 8.0
        self.anchor_centroid_weight = 1.5
        self.anchor_inlier_weight   = 0.01
        self.anchor_angle_reject    = 20.0    # deg
        self.anchor_dist_reject     = 0.8     # m

        # -- Anchor EMA ----------------------------------------------
        self._anchor_ema_alpha = 0.4

        # -- Heading filter ------------------------------------------
        self._h_err_filter_beta = 0.45

        # -- Diagonal 45-deg deterministic rule ----------------------
        self.diagonal_band_deg = 8.0
        self.preferred_diagonal_target_axis = 0.0

        # -- Runtime -------------------------------------------------
        self.enabled = False
        self._init_runtime_state()

        # -- ROS I/O -------------------------------------------------
        self.cmd_pub     = rospy.Publisher('motor_control/twist', Twist,
                                          queue_size=1, tcp_nodelay=True)
        self.pub_enabled = rospy.Publisher('go_home/enabled', Bool,
                                          queue_size=1, latch=True)
        self.viz_pub     = rospy.Publisher('go_home/room_viz', Image,
                                          queue_size=1)
        self.cv_bridge   = CvBridge()

        rospy.Subscriber('lidar/scans', LaserScan, self.on_lidar,
                         queue_size=1, tcp_nodelay=True)

        self.srv = rospy.Service('/go_home/enable', SetBool, self.on_enable)

        self.publish_status()
        rospy.loginfo("[GoHome] Node started (single-wall commitment).")

    # ================================================================
    #  Runtime state
    # ================================================================

    def _init_runtime_state(self):
        self.state = self.ST_ACQUIRE

        # -- Locked anchor (ONE physical wall) -----------------------
        self._anchor_initialized     = False
        self._anchor_line_angle      = None      # [0, 180)
        self._anchor_target_axis     = None      # locked 0 or 90
        self._anchor_centroid        = np.zeros(2)
        self._anchor_signed_dist     = 0.0
        self._anchor_n_inliers       = 0
        self._anchor_lost_count      = 0
        self._anchor_match_score     = 999.0
        self._anchor_diagonal_used   = False
        self._last_matched_wall_angle = None

        # -- Heading error -------------------------------------------
        self._h_err          = 0.0
        self._filtered_h_err = None

        # -- Soft-zone pulse state -----------------------------------
        self._soft_phase         = 'idle'
        self._soft_pulse_counter = 0
        self._soft_pause_counter = 0
        self._soft_pulse_sign    = 1

        # -- Anti-oscillation (FAR zone) -----------------------------
        self._prev_cmd_sign   = 0
        self._sign_flip_count = 0

        # -- Stuck detection -----------------------------------------
        self._prev_abs_h_err          = None
        self._align_no_improve_count  = 0
        self._prev_dist_to_center     = None
        self._center_no_improve_count = 0

        # -- Sign divergence -----------------------------------------
        self._sign_diverge_count     = 0
        self._prev_filtered_for_sign = None

        # -- Confirmation counters -----------------------------------
        self._aligned_count  = 0
        self._centered_count = 0

        # -- Display bookkeeping -------------------------------------
        self._dist_to_center = 0.0
        self._last_ang_cmd   = 0.0
        self._last_lin_cmd   = 0.0
        self._raw_wall_angles = []
        self._is_aligned     = False
        self._is_centered    = False
        self._angular_zone   = "N/A"

    # ================================================================
    #  Service / status
    # ================================================================

    def publish_status(self):
        msg = Bool()
        msg.data = self.enabled
        self.pub_enabled.publish(msg)

    def on_enable(self, req):
        self.enabled = bool(req.data)
        self._init_runtime_state()
        self.publish_status()
        if not self.enabled:
            self._hard_stop()
        rospy.loginfo("[GoHome] enable=%s", self.enabled)
        return SetBoolResponse(success=True,
                               message="GoHome enabled: %s" % self.enabled)

    # ================================================================
    #  PCA / RANSAC wall extraction
    # ================================================================

    def _fit_pca_line(self, pts):
        centroid = pts.mean(axis=0)
        centered = pts - centroid
        cov = np.cov(centered.T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        wall_dir = eigvecs[:, int(np.argmax(eigvals))]
        wall_dir /= max(np.linalg.norm(wall_dir), 1e-9)
        return wall_dir, centroid

    def _line_distances(self, pts, wall_dir, centroid):
        rel = pts - centroid
        return np.abs(rel[:, 0] * wall_dir[1] - rel[:, 1] * wall_dir[0])

    def _extract_wall_lines(self, pts):
        """Extract up to 4 wall lines.  Each dict has:
        dir, centroid, n_inliers, line_angle, normal, signed_dist."""
        walls = []
        remaining = pts.copy()

        for _ in range(4):
            if len(remaining) < self.wall_min_inliers:
                break

            best_idx   = None
            best_count = 0

            for _ in range(self.wall_ransac_iters):
                ids = np.random.choice(len(remaining), 2, replace=False)
                p1, p2 = remaining[ids[0]], remaining[ids[1]]
                d = p2 - p1
                n = np.linalg.norm(d)
                if n < 1e-4:
                    continue
                d /= n
                dists   = self._line_distances(remaining, d, p1)
                inliers = np.where(dists < self.wall_inlier_tol)[0]
                if len(inliers) > best_count:
                    best_count = len(inliers)
                    best_idx   = inliers

            if best_idx is None or best_count < self.wall_min_inliers:
                break

            inlier_pts = remaining[best_idx]
            wall_dir, centroid = self._fit_pca_line(inlier_pts)

            refined_dists = self._line_distances(remaining, wall_dir, centroid)
            refined_idx   = np.where(refined_dists < self.wall_inlier_tol)[0]
            if len(refined_idx) < self.wall_min_inliers:
                break

            inlier_pts = remaining[refined_idx]
            wall_dir, centroid = self._fit_pca_line(inlier_pts)

            # Normal pointing away from robot origin
            nx, ny = -wall_dir[1], wall_dir[0]
            dot = nx * centroid[0] + ny * centroid[1]
            if dot < 0:
                nx, ny = -nx, -ny
                dot = -dot

            walls.append({
                'dir':         wall_dir,
                'centroid':    centroid,
                'n_inliers':   len(refined_idx),
                'line_angle':  _line_angle_deg(wall_dir),
                'normal':      np.array([nx, ny]),
                'signed_dist': dot,
            })

            keep = np.ones(len(remaining), dtype=bool)
            keep[refined_idx] = False
            remaining = remaining[keep]

        return walls

    def _process_lidar_walls(self, ranges):
        n_rays     = len(ranges)
        angles_rad = np.deg2rad(np.arange(n_rays))
        valid      = (ranges > 0.15) & (ranges < 12.0) & np.isfinite(ranges)
        if np.sum(valid) < self.wall_min_inliers:
            return []
        x = ranges[valid] * np.cos(angles_rad[valid])
        y = ranges[valid] * np.sin(angles_rad[valid])
        return self._extract_wall_lines(np.column_stack([x, y]))

    # ================================================================
    #  Deterministic target-axis selection (called ONCE)
    # ================================================================

    def _select_target_axis(self, line_angle):
        """Choose 0 or 90 for the given undirected wall angle.

        Near the diagonal (within diagonal_band_deg of 45 mod 90),
        use a fixed deterministic choice to avoid noisy flip-flopping.
        Returns (target_axis, diagonal_rule_was_used).
        """
        a_mod90 = line_angle % 90.0
        if abs(a_mod90 - 45.0) <= self.diagonal_band_deg:
            return self.preferred_diagonal_target_axis, True

        dist_to_0  = min(line_angle, 180.0 - line_angle)
        dist_to_90 = abs(line_angle - 90.0)
        if dist_to_0 <= dist_to_90:
            return 0.0, False
        else:
            return 90.0, False

    # ================================================================
    #  Anchor acquisition (ST_ACQUIRE -- called once)
    # ================================================================

    def _acquire_anchor(self, walls):
        """Choose ONE physical wall, lock its target axis.

        If a parallel pair is detected it is used only to confirm room
        orientation; the better of the two walls becomes the tracked
        anchor.  Returns True on success.
        """
        if not walls:
            return False

        # ----- find the best single wall ----------------------------
        #  Try to use a parallel pair for orientation evidence, but
        #  always select the stronger wall from the pair.
        chosen = None
        pair_used = False

        # Look for a parallel pair (optional evidence)
        for i in range(len(walls)):
            for j in range(i + 1, len(walls)):
                ad = abs(_angle_diff_mod180(
                    walls[i]['line_angle'], walls[j]['line_angle']))
                if ad > 10.0:
                    continue
                # Perpendicular separation
                avg_rad  = np.deg2rad(
                    (walls[i]['line_angle'] + walls[j]['line_angle']) / 2.0)
                perp_dir = np.array([-np.sin(avg_rad), np.cos(avg_rad)])
                cdiff    = walls[j]['centroid'] - walls[i]['centroid']
                perp_dist = abs(np.dot(cdiff, perp_dir))
                if 0.5 < perp_dist < 12.0:
                    # Valid pair -- choose the wall with more inliers
                    chosen = (walls[i] if walls[i]['n_inliers']
                              >= walls[j]['n_inliers'] else walls[j])
                    pair_used = True
                    break
            if chosen is not None:
                break

        # No pair found -- pick the strongest single wall
        if chosen is None:
            chosen = max(walls, key=lambda w: w['n_inliers'])

        # ----- lock anchor state ------------------------------------
        self._anchor_line_angle  = chosen['line_angle']
        self._anchor_centroid    = chosen['centroid'].copy()
        self._anchor_signed_dist = chosen['signed_dist']
        self._anchor_n_inliers   = chosen['n_inliers']

        # ----- lock target axis (ONCE) with diagonal rule -----------
        axis, diag = self._select_target_axis(self._anchor_line_angle)
        self._anchor_target_axis   = axis
        self._anchor_diagonal_used = diag

        # ----- initial heading error --------------------------------
        raw_err = _heading_error_to_locked_axis(
            self._anchor_line_angle, self._anchor_target_axis)
        self._h_err          = raw_err
        self._filtered_h_err = raw_err

        self._anchor_initialized = True
        self._anchor_lost_count  = 0
        self._anchor_match_score = 0.0

        rospy.loginfo(
            "[GoHome] Anchor LOCKED  angle=%.1f  target=%d  "
            "diag_rule=%s  pair_evidence=%s  inliers=%d  "
            "signed_dist=%.2f  init_h_err=%.1f",
            self._anchor_line_angle, int(self._anchor_target_axis),
            diag, pair_used, self._anchor_n_inliers,
            self._anchor_signed_dist, raw_err)
        return True

    # ================================================================
    #  Physical anchor matching (called every frame after acquisition)
    # ================================================================

    def _match_anchor_in_frame(self, walls):
        """Match detected walls to the stored physical anchor.

        Uses weighted score of angle, signed_distance, and centroid
        proximity, with hard reject thresholds.  A small inlier bonus
        breaks ties.

        Returns (matched_wall, score) or (None, 999).
        """
        if not walls or not self._anchor_initialized:
            return None, 999.0

        best_wall  = None
        best_score = 999.0

        for w in walls:
            ae = abs(_angle_diff_mod180(
                w['line_angle'], self._anchor_line_angle))
            if ae > self.anchor_angle_reject:
                continue

            de = abs(w['signed_dist'] - self._anchor_signed_dist)
            if de > self.anchor_dist_reject:
                continue

            ce = np.linalg.norm(w['centroid'] - self._anchor_centroid)

            score = (self.anchor_angle_weight    * ae
                     + self.anchor_distance_weight * de
                     + self.anchor_centroid_weight * ce
                     - self.anchor_inlier_weight   * w['n_inliers'])

            if score < best_score:
                best_score = score
                best_wall  = w

        if best_wall is None:
            return None, 999.0
        return best_wall, best_score

    # ================================================================
    #  Anchor update (EMA, target axis NEVER changed)
    # ================================================================

    def _update_anchor_from_match(self, matched):
        alpha = self._anchor_ema_alpha

        # EMA on angle (mod-180 aware)
        delta = _angle_diff_mod180(
            matched['line_angle'], self._anchor_line_angle)
        self._anchor_line_angle += alpha * delta
        self._anchor_line_angle %= 180.0

        # EMA on centroid
        self._anchor_centroid += alpha * (
            matched['centroid'] - self._anchor_centroid)

        # EMA on signed distance
        self._anchor_signed_dist += alpha * (
            matched['signed_dist'] - self._anchor_signed_dist)

        self._anchor_n_inliers = matched['n_inliers']

        # -- heading error (direct, no unwrapping) -------------------
        raw_h_err = _heading_error_to_locked_axis(
            self._anchor_line_angle, self._anchor_target_axis)
        self._h_err = raw_h_err

        beta = self._h_err_filter_beta
        if self._filtered_h_err is None:
            self._filtered_h_err = raw_h_err
        else:
            self._filtered_h_err += beta * (raw_h_err - self._filtered_h_err)

        # Sanity
        if abs(self._h_err) > 70.0:
            rospy.logwarn("[GoHome] h_err=%.1f sanity fail. -> ACQUIRE",
                          self._h_err)
            self._anchor_initialized = False
            self._h_err = 0.0
            self._filtered_h_err = None
            return

        self._anchor_lost_count       = 0
        self._anchor_match_score      = abs(delta)
        self._last_matched_wall_angle = matched['line_angle']

    # ================================================================
    #  Visualisation
    # ================================================================

    def _publish_room_viz(self, ranges, walls, matched_idx):
        viz_size = 480
        scale    = viz_size / 24.0
        cx       = viz_size // 2
        canvas   = np.zeros((viz_size, viz_size, 3), dtype=np.uint8)

        # LiDAR points
        n = len(ranges)
        angles_rad = np.deg2rad(np.arange(n))
        xs = ranges * np.cos(angles_rad)
        ys = ranges * np.sin(angles_rad)
        valid = (ranges > 0.15) & (ranges < 12.0) & np.isfinite(ranges)
        for i in np.where(valid)[0]:
            px = int(cx + xs[i] * scale)
            py = int(cx - ys[i] * scale)
            if 0 <= px < viz_size and 0 <= py < viz_size:
                canvas[py, px] = (130, 130, 130)

        # Fitted walls
        for idx, w in enumerate(walls):
            wd, cen = w['dir'], w['centroid']
            p1 = cen - 12.0 * wd
            p2 = cen + 12.0 * wd
            pt1 = (int(cx + p1[0]*scale), int(cx - p1[1]*scale))
            pt2 = (int(cx + p2[0]*scale), int(cx - p2[1]*scale))
            col = (255, 255, 0) if idx == matched_idx else (0, 255, 0)
            cv2.line(canvas, pt1, pt2, col,
                     2 if idx == matched_idx else 1, cv2.LINE_AA)

        # Anchor direction (orange)
        if self._anchor_initialized and self._anchor_line_angle is not None:
            a_rad = np.deg2rad(self._anchor_line_angle)
            dx, dy = np.cos(a_rad), np.sin(a_rad)
            L = 90
            cv2.line(canvas,
                     (int(cx - dx*L), int(cx + dy*L)),
                     (int(cx + dx*L), int(cx - dy*L)),
                     (0, 140, 255), 2, cv2.LINE_AA)

        # Robot
        rw = int(0.35 * scale)
        cv2.rectangle(canvas, (cx-rw, cx-rw), (cx+rw, cx+rw),
                      (255, 255, 255), 1)
        cv2.circle(canvas, (cx, cx), 4, (0, 0, 255), -1)

        # Text overlay
        _sn = {0: "ACQUIRE", 1: "ALIGN", 2: "CENTER", 3: "VERIFY"}
        anc  = "%.1f" % self._anchor_line_angle \
               if self._anchor_line_angle is not None else "N/A"
        tgt  = "%d" % int(self._anchor_target_axis) \
               if self._anchor_target_axis is not None else "N/A"
        fe   = "%.2f" % self._filtered_h_err \
               if self._filtered_h_err is not None else "N/A"
        mw   = "%.1f" % self._last_matched_wall_angle \
               if self._last_matched_wall_angle is not None else "N/A"

        lines = [
            "State: %s" % _sn.get(self.state, '?'),
            "raw_h: %.2f  filt_h: %s" % (self._h_err, fe),
            "tol: %.1f  zone: %s" % (self.angle_tol, self._angular_zone),
            "ALIGNED: %s (%d/%d)" % (self._is_aligned,
                self._aligned_count, self.ALIGN_CONFIRM_FRAMES),
            "Pulse: %d  Pause: %d" % (self._soft_pulse_counter,
                                        self._soft_pause_counter),
            "Anchor: %s -> tgt %s" % (anc, tgt),
            "DiagRule: %s" % self._anchor_diagonal_used,
            "Matched: %s  Score: %.1f" % (mw, self._anchor_match_score),
            "Inl: %d  Lost: %d" % (self._anchor_n_inliers,
                                     self._anchor_lost_count),
            "ang.z: %.3f  angSign: %.0f" % (self._last_ang_cmd,
                                              self.angular_sign),
            "Dist: %.3f  tol: %.2f" % (self._dist_to_center,
                                         self.center_tol),
            "CENTRED: %s (%d/%d)" % (self._is_centered,
                self._centered_count, self.CENTER_CONFIRM_FRAMES),
            "lin: %.3f" % self._last_lin_cmd,
            "StuckA: %d  StuckC: %d" % (self._align_no_improve_count,
                                         self._center_no_improve_count),
            "SignDiv: %d" % self._sign_diverge_count,
        ]
        for i, txt in enumerate(lines):
            cv2.putText(canvas, txt, (5, 14 + i*14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.34,
                        (0, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow("GoHome Debug", canvas)
        cv2.waitKey(1)
        try:
            self.viz_pub.publish(
                self.cv_bridge.cv2_to_imgmsg(canvas, encoding="bgr8"))
        except Exception as e:
            rospy.logerr("[GoHome] Viz error: %s", e)

    # ================================================================
    #  Distance helper
    # ================================================================

    def _get_dist(self, ranges, angle_deg):
        idx  = [(int(angle_deg) + i) % len(ranges) for i in range(-10, 10)]
        vals = ranges[idx]
        good = vals[(vals > 0.1) & (vals < 15.0) & np.isfinite(vals)]
        return np.median(good) if len(good) > 0 else None

    # ================================================================
    #  Main LiDAR callback
    # ================================================================

    def on_lidar(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32) / 1000.0

        walls = self._process_lidar_walls(ranges)
        self._raw_wall_angles = [w['line_angle'] for w in walls]

        # Viz highlight
        matched_idx = -1
        if self._anchor_initialized and walls:
            for idx, w in enumerate(walls):
                if abs(_angle_diff_mod180(
                        w['line_angle'],
                        self._anchor_line_angle)) < 15.0:
                    matched_idx = idx
                    break

        self._publish_room_viz(ranges, walls, matched_idx)

        if not self.enabled:
            return

        cmd = Twist()

        # ============================================================
        # ST_ACQUIRE
        # ============================================================
        if self.state == self.ST_ACQUIRE:
            self._angular_zone = "N/A"
            if self._acquire_anchor(walls):
                rospy.loginfo("[GoHome] -> ALIGN")
                self.state = self.ST_ALIGN
            self.cmd_pub.publish(cmd)
            return

        # ============================================================
        # Anchor tracking  (ALIGN / CENTER / VERIFY)
        # ============================================================
        if self._anchor_initialized:
            matched, score = self._match_anchor_in_frame(walls)
            self._anchor_match_score = score

            if matched is not None:
                self._update_anchor_from_match(matched)
            else:
                self._anchor_lost_count += 1
                rospy.logwarn("[GoHome] Anchor lost %d/%d",
                              self._anchor_lost_count,
                              self.TRACKING_LOST_LIMIT)

            if (not self._anchor_initialized
                    or self._anchor_lost_count >= self.TRACKING_LOST_LIMIT):
                rospy.logwarn("[GoHome] Anchor lost! -> ACQUIRE")
                self._hard_stop()
                self._init_runtime_state()
                self.cmd_pub.publish(Twist())
                return

        f_err = (self._filtered_h_err
                 if self._filtered_h_err is not None
                 else self._h_err)

        # ============================================================
        # ST_ALIGN  (three zones: ALIGNED / SOFT / FAR)
        # ============================================================
        if self.state == self.ST_ALIGN:

            # -- stuck detection ------------------------------------
            ca = abs(f_err)
            if (self._prev_abs_h_err is not None
                    and abs(self._last_ang_cmd) > 0.05):
                if ca >= self._prev_abs_h_err - 0.3:
                    self._align_no_improve_count += 1
                else:
                    self._align_no_improve_count = 0
            else:
                self._align_no_improve_count = 0
            self._prev_abs_h_err = ca

            if self._align_no_improve_count > 10:
                rospy.logwarn(
                    "[GoHome] ALIGN stuck %d frames! filt=%.2f "
                    "ang.z=%.3f. angular_sign=%.1f may be wrong.",
                    self._align_no_improve_count, f_err,
                    self._last_ang_cmd, self.angular_sign)

            # -- sign divergence ------------------------------------
            if (abs(self._last_ang_cmd) > 0.05
                    and self._prev_filtered_for_sign is not None):
                if abs(f_err) > abs(self._prev_filtered_for_sign) + 0.3:
                    self._sign_diverge_count += 1
                else:
                    self._sign_diverge_count = 0
                if self._sign_diverge_count > 8:
                    rospy.logwarn(
                        "[GoHome] h_err GROWING while rotating! "
                        "Try angular_sign = %.1f",
                        -self.angular_sign)
                    self._sign_diverge_count = 0
            self._prev_filtered_for_sign = f_err

            # ---- ZONE 1: ALIGNED ----------------------------------
            if abs(f_err) <= self.angle_tol:
                self._angular_zone   = "ALIGNED"
                self._is_aligned     = True
                self._aligned_count += 1
                self._soft_phase      = 'idle'
                self._soft_pulse_counter = 0
                self._soft_pause_counter = 0
                self._prev_cmd_sign   = 0
                self._sign_flip_count = 0
                cmd.angular.z  = 0.0
                self._last_ang_cmd = 0.0

                if self._aligned_count >= self.ALIGN_CONFIRM_FRAMES:
                    rospy.loginfo(
                        "[GoHome] Aligned (f_err=%.2f, %d fr). -> CENTER",
                        f_err, self._aligned_count)
                    self._hard_stop()
                    self._aligned_count          = 0
                    self._is_aligned             = False
                    self._prev_abs_h_err         = None
                    self._align_no_improve_count = 0
                    rospy.sleep(0.3)
                    self.state = self.ST_CENTER
                self.cmd_pub.publish(cmd)
                return

            # ---- ZONE 2: SOFT (pulse / pause) ---------------------
            elif abs(f_err) <= self.soft_angle_zone:
                self._aligned_count = 0
                self._is_aligned    = False
                self._prev_cmd_sign   = 0
                self._sign_flip_count = 0

                correction_sign = 1 if f_err > 0 else -1

                if self._soft_phase == 'idle':
                    self._soft_phase = 'pulse'
                    self._soft_pulse_counter = 0
                    self._soft_pulse_sign = correction_sign

                if self._soft_phase == 'pulse':
                    self._angular_zone = "SOFT_PULSE"
                    cmd.angular.z = float(
                        self._soft_pulse_sign
                        * self.soft_pulse_ang_v
                        * self.angular_sign)
                    self._last_ang_cmd = cmd.angular.z
                    self._soft_pulse_counter += 1
                    if self._soft_pulse_counter >= self.soft_pulse_frames:
                        self._soft_phase = 'pause'
                        self._soft_pause_counter = 0

                elif self._soft_phase == 'pause':
                    self._angular_zone = "SOFT_PAUSE"
                    cmd.angular.z  = 0.0
                    self._last_ang_cmd = 0.0
                    self._soft_pause_counter += 1
                    if self._soft_pause_counter >= self.soft_pause_frames:
                        self._soft_phase = 'idle'

                self.cmd_pub.publish(cmd)
                return

            # ---- ZONE 3: FAR (P-controller + friction floor) ------
            else:
                self._angular_zone   = "FAR"
                self._aligned_count  = 0
                self._is_aligned     = False
                self._soft_phase         = 'idle'
                self._soft_pulse_counter = 0
                self._soft_pause_counter = 0

                v_ang = self.angular_sign * f_err * self.kp_ang

                if abs(v_ang) < self.min_ang_v:
                    v_ang = self.min_ang_v if v_ang >= 0 else -self.min_ang_v

                new_sign = 1 if v_ang > 0 else -1
                if (self._prev_cmd_sign != 0
                        and new_sign != self._prev_cmd_sign):
                    self._sign_flip_count += 1
                    if self._sign_flip_count < 4:
                        v_ang = self._prev_cmd_sign * abs(v_ang)
                    else:
                        self._sign_flip_count = 0
                else:
                    self._sign_flip_count = 0
                self._prev_cmd_sign = new_sign \
                    if self._sign_flip_count == 0 else self._prev_cmd_sign

                cmd.angular.z = float(
                    np.clip(v_ang, -self.max_ang_v, self.max_ang_v))
                self._last_ang_cmd = cmd.angular.z
                self.cmd_pub.publish(cmd)
                return

        # ============================================================
        # LiDAR distance measurements for CENTER / VERIFY
        # ============================================================
        self._angular_zone = "N/A"

        fd = self._get_dist(ranges, 270)
        bd = self._get_dist(ranges, 90)
        ld = self._get_dist(ranges, 0)
        rd = self._get_dist(ranges, 180)

        if None in (fd, bd, ld, rd):
            self.cmd_pub.publish(cmd)
            return

        error_x        = (fd - bd) / 2.0
        error_y        = (ld - rd) / 2.0
        dist_to_center = math.hypot(error_x, error_y)
        self._dist_to_center = dist_to_center

        # ============================================================
        # ST_CENTER
        # ============================================================
        if self.state == self.ST_CENTER:

            # Heading drift guard (return to ALIGN, NOT re-acquire)
            if abs(f_err) > (self.verify_angle_tol + 3.0):
                rospy.loginfo("[GoHome] Heading drift %.2f. -> ALIGN", f_err)
                self._hard_stop()
                self._aligned_count           = 0
                self._centered_count          = 0
                self._center_no_improve_count = 0
                self._prev_dist_to_center     = None
                self.state = self.ST_ALIGN
                return

            # Stuck detection
            if (self._prev_dist_to_center is not None
                    and self._last_lin_cmd > 0.05):
                if dist_to_center >= self._prev_dist_to_center - 0.005:
                    self._center_no_improve_count += 1
                else:
                    self._center_no_improve_count = 0
            self._prev_dist_to_center = dist_to_center

            if self._center_no_improve_count > 10:
                rospy.logwarn(
                    "[GoHome] CENTER stuck %d frames! dist=%.3f. Boosting.",
                    self._center_no_improve_count, dist_to_center)

            # -- Inside tolerance: stop, confirm ---------------------
            if dist_to_center <= self.center_tol:
                self._centered_count += 1
                self._is_centered = True
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                self._last_lin_cmd = 0.0

                if self._centered_count >= self.CENTER_CONFIRM_FRAMES:
                    rospy.loginfo(
                        "[GoHome] Centred (%.3f m, %d fr). -> VERIFY",
                        dist_to_center, self._centered_count)
                    self._hard_stop()
                    self._centered_count          = 0
                    self._is_centered             = False
                    self._center_no_improve_count = 0
                    self._prev_dist_to_center     = None
                    rospy.sleep(0.3)
                    self.state = self.ST_VERIFY
                self.cmd_pub.publish(cmd)
                return

            # -- Outside tolerance: drive ----------------------------
            self._centered_count = 0
            self._is_centered    = False

            target_speed = self.kp_lin * dist_to_center
            target_speed = max(target_speed, self.min_v)

            if self._center_no_improve_count > 10:
                target_speed = max(target_speed, self.max_speed * 0.8)

            target_speed = min(target_speed, self.max_speed)

            cmd.linear.x = (error_x / dist_to_center) * target_speed
            cmd.linear.y = (error_y / dist_to_center) * target_speed
            self._last_lin_cmd = target_speed
            self.cmd_pub.publish(cmd)
            return

        # ============================================================
        # ST_VERIFY
        # ============================================================
        elif self.state == self.ST_VERIFY:
            heading_ok = abs(f_err)     < self.verify_angle_tol
            center_ok  = dist_to_center < self.verify_dist_tol

            if heading_ok and center_ok:
                rospy.loginfo("[GoHome] SUCCESS f_err=%.2f dist=%.3f",
                              f_err, dist_to_center)
                self._finish()
                return

            if not heading_ok:
                rospy.loginfo(
                    "[GoHome] Verify FAIL heading (%.2f). -> ALIGN", f_err)
                self._aligned_count = 0
                self.state = self.ST_ALIGN
                self.cmd_pub.publish(Twist())
                return

            if not center_ok:
                rospy.loginfo(
                    "[GoHome] Verify FAIL center (%.3f). -> CENTER",
                    dist_to_center)
                self._centered_count          = 0
                self._center_no_improve_count = 0
                self._prev_dist_to_center     = None
                self.state = self.ST_CENTER
                self.cmd_pub.publish(Twist())
                return

        self.cmd_pub.publish(cmd)

    # ================================================================
    #  Stop / finish
    # ================================================================

    def _hard_stop(self):
        zero = Twist()
        for _ in range(3):
            self.cmd_pub.publish(zero)
            rospy.sleep(0.05)
        self._last_ang_cmd = 0.0
        self._last_lin_cmd = 0.0

    def stop_robot(self):
        self.cmd_pub.publish(Twist())
        self._last_ang_cmd = 0.0
        self._last_lin_cmd = 0.0

    def _finish(self):
        self._hard_stop()
        self.enabled = False
        self._init_runtime_state()
        self.publish_status()
        rospy.loginfo("[GoHome] Task finished -- robot stopped.")


# ====================================================================
if __name__ == "__main__":
    rospy.init_node("go_home_node")
    try:
        node = GoHomeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass