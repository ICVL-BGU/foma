#!/usr/bin/env python3
"""
Robot Rotation Node for FOMA.

Provides discrete step rotations (90, 180, 270, 360 degrees / quarter, half, 3/4, full turn)
combining open-loop course rotation with fine LIDAR-based wall-axis snapping.

Exposes clean ROS Services for easy GUI button integration.
"""

import math
import threading
import time
from collections import namedtuple

import cv2
import numpy as np
import rospy
from abstract_node import AbstractNode
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger, TriggerResponse


class Status:
    SUCCESS   = 'success'
    TIMEOUT   = 'timeout'
    CANCELLED = 'cancelled'
    NO_SCAN   = 'no_scan'


class PhaseResult(namedtuple('PhaseResult', 'status error elapsed')):
    """Outcome of a rotation execution."""

    __slots__ = ()

    @property
    def ok(self):
        return self.status == Status.SUCCESS

    def __bool__(self):
        return self.ok

    def __str__(self):
        return "%s (err=%.4f, %.1fs)" % (self.status, self.error, self.elapsed)


class RobotRotationNode(AbstractNode):
    """
    ROS Node that executes precise relative rotations (90, 180, 270, 360 deg)
    and exposes ROS services for GUI integration.
    """

    # Default parameters (can be overridden via ROS parameter server)
    MAX_ANG_V = 0.45          # Normalized motor speed [-1, 1]
    MIN_ANG_V = 0.20
    KP_ANG    = 30.0          # P-gain for fine wall alignment
    ALIGN_TOL = 0.005         # Beam symmetry tolerance threshold
    ALIGN_SETTLE_S = 0.3      # Time error must remain inside tolerance
   
    # Nominal angular speed (rad/s) at MAX_ANG_V for coarse rotation timing
    NOMINAL_ANG_SPEED_RAD_S = 1.0

    # LIDAR beam probing parameters
    BEAM_HALF_WIN = 10
    RANGE_MIN_M   = 0.10
    RANGE_MAX_M   = 12.0
    CARDINALS     = [0, 90, 180, 270]
    PROBE_ALPHAS  = [30, 45]

    CONTROL_HZ    = 100.0
    GLOBAL_TIMEOUT_S = 30.0

    def __init__(self):
        super().__init__('robot_rotation', 'Robot Rotation')

        # Load parameters from ROS Param Server
        g = rospy.get_param
        self.max_ang_v = float(g('~max_ang_v', self.MAX_ANG_V))
        self.min_ang_v = float(g('~min_ang_v', self.MIN_ANG_V))
        self.kp_ang    = float(g('~kp_ang', self.KP_ANG))
        self.align_tol = float(g('~align_tol', self.ALIGN_TOL))
        self.align_settle = float(g('~align_settle', self.ALIGN_SETTLE_S))
        self.nominal_ang_speed = float(g('~nominal_ang_speed', self.NOMINAL_ANG_SPEED_RAD_S))
        self.control_hz = float(g('~control_hz', self.CONTROL_HZ))
        self.timeout_s  = float(g('~timeout', self.GLOBAL_TIMEOUT_S))

        # Runtime state
        self._latest_scan = None
        self._cancel = threading.Event()
        self._lock = threading.Lock()
        self._thread = None
        self._is_busy = False

        # ROS Publishers / Subscribers
        self.cmd_pub = rospy.Publisher('motor_control/twist', Twist, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('lidar/scans', LaserScan, self._on_lidar, queue_size=1, tcp_nodelay=True)

        # Services for GUI Integration
        self.srv_rotate_90  = rospy.Service('/rotate/quarter', Trigger, lambda req: self._handle_service_rotate(90))
        self.srv_rotate_180 = rospy.Service('/rotate/half', Trigger, lambda req: self._handle_service_rotate(180))
        self.srv_rotate_270 = rospy.Service('/rotate/three_quarters', Trigger, lambda req: self._handle_service_rotate(270))
        self.srv_rotate_360 = rospy.Service('/rotate/full', Trigger, lambda req: self._handle_service_rotate(360))
        self.srv_cancel     = rospy.Service('/rotate/cancel', Trigger, self._handle_service_cancel)

        rospy.on_shutdown(self.cancel)
        rospy.loginfo("[RobotRotation] Node initialized. Services ready for GUI button calls.")

    # -------------------------------------------------------------------------
    # ROS Service Callbacks
    # -------------------------------------------------------------------------

    def _handle_service_rotate(self, target_deg):
        """Generic handler for rotation service requests."""
        if self._is_busy:
            return TriggerResponse(success=False, message="Robot is currently executing another rotation.")

        started = self.start_rotation(target_deg)
        if started:
            return TriggerResponse(success=True, message=f"Started rotation of {target_deg} degrees.")
        else:
            return TriggerResponse(success=False, message="Failed to start rotation.")

    def _handle_service_cancel(self, req):
        """Service callback to stop and cancel rotation."""
        self.cancel()
        return TriggerResponse(success=True, message="Rotation cancelled and robot stopped.")

    # -------------------------------------------------------------------------
    # Thread & Control Lifecycle
    # -------------------------------------------------------------------------

    def start_rotation(self, target_deg, direction=1):
        """
        Kicks off rotation in a worker thread. Non-blocking call.
        direction: +1 for Counter-Clockwise (CCW), -1 for Clockwise (CW).
        """
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return False
            self._cancel.clear()
            self._is_busy = True
            self._thread = threading.Thread(
                target=self._rotation_worker,
                args=(target_deg, direction),
                name='rotation_worker',
                daemon=True
            )
            self._thread.start()
            return True

    def cancel(self, join_timeout=1.0):
        """Stop rotation and hard-stop motors."""
        self._cancel.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=join_timeout)
        self._hard_stop()
        self._is_busy = False

    def _rotation_worker(self, target_deg, direction):
        try:
            result = self.execute_rotation(target_deg, direction)
            rospy.loginfo("[RobotRotation] Rotation of %d deg finished with result: %s", target_deg, result)
        finally:
            self._hard_stop()
            self._is_busy = False

    # -------------------------------------------------------------------------
    # LIDAR Processing & Beam Symmetry
    # -------------------------------------------------------------------------

    def _on_lidar(self, msg):
        """Callback receiving scan data."""
        ranges_m = np.array(msg.ranges, dtype=np.float64) / 1000.0
        self._latest_scan = ranges_m

    def _get_dist(self, ranges_m, angle_deg):
        n = len(ranges_m)
        indices = [(int(round(angle_deg)) + i) % n for i in range(-self.BEAM_HALF_WIN, self.BEAM_HALF_WIN + 1)]
        vals = ranges_m[indices]
        good = vals[(vals > self.RANGE_MIN_M) & (vals < self.RANGE_MAX_M) & np.isfinite(vals)]
        if len(good) < 3:
            return None
        return float(np.median(good))

    def _compute_alignment_error(self, ranges_m):
        """Calculates beam symmetry error relative to nearest cardinal axis."""
        accum = 0.0
        count = 0
        for c in self.CARDINALS:
            for alpha in self.PROBE_ALPHAS:
                d_plus  = self._get_dist(ranges_m, (c + alpha) % 360)
                d_minus = self._get_dist(ranges_m, (c - alpha) % 360)
                if d_plus is None or d_minus is None:
                    continue
                s = d_plus + d_minus
                if s < 0.01:
                    continue
                accum += (d_plus - d_minus) / s
                count += 1

        if count == 0:
            return 0.0, False
        return accum / count, True

    # -------------------------------------------------------------------------
    # Rotation Execution Core
    # -------------------------------------------------------------------------

    def execute_rotation(self, target_deg, direction=1):
        """
        Executes relative rotation:
          Phase 1: Coarse turn (target - 15 degrees).
          Phase 2: Fine alignment to wall axis using LIDAR symmetry.
        """
        started = time.monotonic()
        rate = rospy.Rate(self.control_hz)

        # Calculate coarse angle duration (leave 15 deg for fine snapping)
        coarse_deg = max(0, target_deg - 15)
        coarse_rad = math.radians(coarse_deg)
        coarse_duration = coarse_rad / self.nominal_ang_speed if self.nominal_ang_speed > 0 else 0.0

        rospy.loginfo(f"[RobotRotation] Phase 1: Coarse turn {coarse_deg} deg (~{coarse_duration:.2f}s)")

        coarse_start = time.monotonic()
        while not rospy.is_shutdown() and (time.monotonic() - coarse_start < coarse_duration):
            if self._cancel.is_set():
                self._hard_stop()
                return PhaseResult(Status.CANCELLED, 0.0, time.monotonic() - started)

            cmd = Twist()
            cmd.angular.z = direction * self.max_ang_v
            self.cmd_pub.publish(cmd)
            rate.sleep()

        self._publish_stop()
        rospy.sleep(0.1)

        # Phase 2: Fine Wall Alignment
        rospy.loginfo("[RobotRotation] Phase 2: Fine LIDAR axis snapping")
        inside_since = None
        deadline = started + self.timeout_s
        residual = float('inf')

        while not rospy.is_shutdown():
            now = time.monotonic()

            if self._cancel.is_set():
                self._hard_stop()
                return PhaseResult(Status.CANCELLED, residual, now - started)

            if now > deadline:
                self._hard_stop()
                return PhaseResult(Status.TIMEOUT, residual, now - started)

            ranges_m = self._latest_scan
            if ranges_m is None:
                self._publish_stop()
                rate.sleep()
                continue

            ang_err, valid = self._compute_alignment_error(ranges_m)
            if not valid:
                self._publish_stop()
                rate.sleep()
                continue

            residual = abs(ang_err)

            if residual <= self.align_tol:
                self._publish_stop()
                if inside_since is None:
                    inside_since = now
                elif now - inside_since >= self.align_settle:
                    self._hard_stop()
                    return PhaseResult(Status.SUCCESS, residual, now - started)
            else:
                inside_since = None
                cmd = Twist()
                v_ang = self.kp_ang * ang_err
                if 0 < abs(v_ang) < self.min_ang_v:
                    v_ang = math.copysign(self.min_ang_v, v_ang)
                cmd.angular.z = max(-self.max_ang_v, min(self.max_ang_v, v_ang))
                self.cmd_pub.publish(cmd)

            rate.sleep()

        self._hard_stop()
        return PhaseResult(Status.CANCELLED, residual, time.monotonic() - started)

    # -------------------------------------------------------------------------
    # Helper Functions
    # -------------------------------------------------------------------------

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _hard_stop(self):
        zero = Twist()
        for _ in range(3):
            self.cmd_pub.publish(zero)
            rospy.sleep(0.03)


if __name__ == '__main__':
    try:
        node = RobotRotationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass