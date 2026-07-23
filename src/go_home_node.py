#!/usr/bin/env python3
"""Homing node: centre the robot in the room, align it to the walls, and
perform precise closed-loop rotations.

Overhauled to keep the three concerns fully **decoupled and reusable**:
  * centering (translate to the geometric centre),
  * alignment (yaw correction to the walls), and
  * precise rotation (spin-in-place via LiDAR feedback)
are each a one-line call into the hardware-independent helpers in
``etc.homing`` (unit-tested in ``test/test_homing.py``). Either controller can
be reused on its own elsewhere (e.g. continuous yaw correction while driving a
corridor) without touching this node.

The precise-rotation capability (previously a separate ``rotation_node``) is
merged here to keep the ROS graph clean: one node, one LiDAR subscription, one
Twist publisher.

Key improvements over the previous version:
  * The wall-orientation estimate no longer fits fixed cardinal sectors (which
    straddle a corner and lose accuracy at ~45 degrees). ``homing`` estimates a
    single room orientation from every wall segment and folds it to
    [-45, 45), so any yaw - including 45 degrees - is handled gracefully.
  * Friction / dead-band compensation lives in one reusable helper
    (``homing.apply_deadband``), applied to every motion channel.
  * Concurrent yaw correction during centering: small angular.z is applied
    alongside the translation commands so the robot is mostly aligned by the
    time it reaches the centre.
  * Consecutive-arrival counter: the node waits for N successive "arrived"
    readings before switching from centering to alignment, preventing
    premature transitions from a single lucky reading while oscillating.
  * While active, the node renders and publishes a top-down "room model as the
    robot sees it" (LiDAR dots on the modelled walls, robot pose, offset
    vector) on ``go_home/room_view`` so the GUI can show it during centering,
    alignment, and rotation.

Rotation interface (non-blocking, so the GUI never freezes):
  * Service ``/rotation/spin`` (foma/Float): request ``data`` = signed degrees
    (positive = CCW, negative = CW). Starts the spin and returns immediately.
  * Service ``/rotation/stop`` (std_srvs/SetBool): abort the current spin.
  * Topic ``/rotation/active`` (Bool, latched): True while a spin is running,
    False when it finishes or is aborted. The GUI/protocol watches this to
    chain the next step.
"""
import rospy
import numpy as np

from abstract_node import AbstractNode
from etc import homing
from sensor_msgs.msg import LaserScan, CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse
from foma.srv import Float, FloatResponse


class GoHomeNode(AbstractNode):
    STATE_CENTER = 0
    STATE_ALIGN = 1
    STATE_SPINNING = 2

    # Number of consecutive "arrived" LiDAR frames required before
    # transitioning from centering to alignment.
    ARRIVAL_CONFIRM_COUNT = 5

    def __init__(self):
        super().__init__('go_home', 'Go Home')

        # Decoupled controller parameter blocks (kept at the original tuning).
        self.center_params = homing.CenteringParams(
            kp=1.3, min_v=0.18, max_speed=0.55, arrival_tol=0.15,
            settle_zone=0.30)
        self.align_params = homing.AlignParams(
            kp=0.06, min_ang_v=0.16, max_ang_v=0.5, angle_tol=3.0)

        # Centering parameters for concurrent yaw correction.
        self._center_yaw_kp = 0.03          # weaker than full alignment
        self._center_yaw_max = 0.25         # cap angular.z while translating

        self.enabled = False
        self.state = self.STATE_CENTER
        self._arrival_counter = 0

        # --- Precise rotation (merged from rotation_node) --- #
        self._rot_tol = rospy.get_param('~rot_tol_deg', 2.0)
        self._rot_slow_zone = rospy.get_param('~rot_slow_zone_deg', 15.0)
        self._spin_cmd = rospy.get_param('~spin_cmd', 0.5)
        self._pulse_period = rospy.get_param('~pulse_period', 4)

        self._rot_controller = homing.RotationController(
            tol=self._rot_tol, slow_zone=self._rot_slow_zone)
        self._rot_tracker = homing.RotationTracker()

        self._spin_active = False
        self._spin_target = 0.0
        self._spin_direction = 0
        self._pulse_tick = 0

        # Safety: maximum allowed spin duration (seconds) and stall detection.
        self._spin_timeout = rospy.get_param('~spin_timeout', 30.0)
        self._spin_start_time = None
        self._spin_stall_limit = rospy.get_param('~spin_stall_limit', 50)
        self._spin_stall_counter = 0
        self._spin_last_travelled = 0.0

        # Room-view rendering throttle (Hz).
        self._view_period = rospy.Duration(1.0 / 10.0)
        self._last_view = rospy.Time(0)

        # Publishers
        self.cmd_pub = rospy.Publisher('motor_control/twist', Twist,
                                       queue_size=1, tcp_nodelay=True)
        self.pub_enabled = rospy.Publisher('go_home/enabled', Bool,
                                           queue_size=1, latch=True)
        self.view_pub = rospy.Publisher('go_home/room_view', CompressedImage,
                                        queue_size=1, tcp_nodelay=True)
        self.spin_active_pub = rospy.Publisher('rotation/active', Bool,
                                               queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber('lidar/scans', LaserScan, self.on_lidar,
                         queue_size=1, tcp_nodelay=True)

        # Services — homing
        rospy.Service('go_home/enable', SetBool, self.on_enable)

        # Services — precise rotation
        rospy.Service('rotation/spin', Float, self.on_spin)
        rospy.Service('rotation/stop', SetBool, self.on_spin_stop)

        self.publish_status()
        self._publish_spin_active()

        # SAFETY: ensure motors are stopped on node shutdown/crash/kill.
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo('[GoHome] Node Started Successfully')

    # ------------------------------------------------------------------ #
    #  Status helpers                                                      #
    # ------------------------------------------------------------------ #
    def publish_status(self):
        self.pub_enabled.publish(Bool(data=self.enabled))

    def _publish_spin_active(self):
        self.spin_active_pub.publish(Bool(data=self._spin_active))

    # ------------------------------------------------------------------ #
    #  Homing enable / disable                                             #
    # ------------------------------------------------------------------ #
    def on_enable(self, req):
        self.enabled = bool(req.data)
        self.state = self.STATE_CENTER
        self._arrival_counter = 0
        self.publish_status()
        if not self.enabled:
            self.stop_robot()
        return SetBoolResponse(success=True,
                               message=f'GoHome status: {self.enabled}')

    # ------------------------------------------------------------------ #
    #  Precise rotation services                                           #
    # ------------------------------------------------------------------ #
    def on_spin(self, req):
        """Begin a precise spin of ``req.data`` degrees (signed)."""
        degrees = float(req.data)
        if abs(degrees) <= self._rot_tol:
            return FloatResponse(result=True)

        # Rotation takes over — disable centering/alignment if active.
        if self.enabled:
            self.enabled = False
            self.publish_status()

        self._spin_target = degrees
        self._spin_direction = 1 if degrees > 0 else -1
        self._rot_tracker.reset()
        self._pulse_tick = 0
        self._spin_active = True
        self._spin_start_time = rospy.Time.now()
        self._spin_stall_counter = 0
        self._spin_last_travelled = 0.0
        self.state = self.STATE_SPINNING
        self._publish_spin_active()
        self.loginfo(f'Spinning {degrees:+.1f} deg (LiDAR closed loop)')
        return FloatResponse(result=True)

    def on_spin_stop(self, req):
        self._finish_spin(aborted=True)
        return SetBoolResponse(success=True, message='Rotation stopped')

    def _finish_spin(self, aborted=False):
        self._spin_active = False
        self._spin_direction = 0
        self.state = self.STATE_CENTER
        self.stop_robot()
        self._publish_spin_active()
        if aborted:
            self.logwarn('Rotation aborted.')

    # ------------------------------------------------------------------ #
    #  LiDAR callback — main control loop                                  #
    # ------------------------------------------------------------------ #
    def on_lidar(self, msg):
        ranges = np.array(msg.ranges) / 1000.0  # mm -> m

        # Always compute room geometry for the view — even when idle.
        cardinals = homing.cardinal_distances(ranges)
        heading_err = homing.wall_heading_error(ranges)

        # Publish room view whenever homing or spinning is active.
        if self.enabled or self._spin_active:
            self._publish_room_view(ranges, heading_err, cardinals)

        # --- Precise rotation (STATE_SPINNING) --- #
        if self._spin_active:
            # Safety timeout: abort if we've been spinning too long.
            if self._spin_start_time is not None:
                elapsed = (rospy.Time.now() - self._spin_start_time).to_sec()
                if elapsed > self._spin_timeout:
                    self.logwarn(f'Rotation TIMEOUT after {elapsed:.1f}s '
                                 f'(travelled {self._rot_tracker.total:+.1f} deg '
                                 f'of {self._spin_target:+.1f} deg). Aborting.')
                    self._finish_spin(aborted=True)
                    return

            # wall_heading_error returns exactly 0.0 when it cannot compute a
            # reliable heading (too few wall segments visible — common during
            # fast rotation).  Feeding that into the tracker corrupts the
            # accumulated angle, so we skip it and just keep the motors
            # running with the last command.
            if heading_err is None or heading_err == 0.0:
                # Still publish the spin command so motors keep turning.
                cmd = Twist()
                cmd.angular.z = self._spin_direction * self._spin_cmd
                self.cmd_pub.publish(cmd)
                return

            folded = heading_err  # wall_heading_error already gives folded angle
            travelled = self._rot_tracker.update(folded)
            remaining = self._spin_target - travelled

            # Stall detection: if travelled hasn't changed for too many frames
            # despite receiving valid headings, something is wrong.
            if abs(travelled - self._spin_last_travelled) < 0.05:
                self._spin_stall_counter += 1
            else:
                self._spin_stall_counter = 0
                self._spin_last_travelled = travelled

            if self._spin_stall_counter >= self._spin_stall_limit:
                self.logwarn(f'Rotation STALLED ({self._spin_stall_limit} frames '
                             f'with no progress at {travelled:+.1f} deg). Aborting.')
                self._finish_spin(aborted=True)
                return

            mode, sign = self._rot_controller.classify(remaining)
            if mode == homing.RotationController.STOP:
                self.loginfo(f'Rotation complete (travelled {travelled:+.1f} deg)')
                self._finish_spin()
                return

            cmd = Twist()
            if mode == homing.RotationController.FULL:
                cmd.angular.z = sign * self._spin_cmd
            else:  # SLOW -> pulse the command to creep the last few degrees
                self._pulse_tick = (self._pulse_tick + 1) % self._pulse_period
                cmd.angular.z = sign * self._spin_cmd if self._pulse_tick == 0 else 0.0
            self.cmd_pub.publish(cmd)
            return

        # --- Homing (centering + alignment) --- #
        if not self.enabled:
            return

        if cardinals is None:
            return

        cmd = Twist()
        if self.state == self.STATE_CENTER:
            vx, vy, dist, arrived = homing.centering_command(
                cardinals, self.center_params)

            if arrived:
                self._arrival_counter += 1
                if self._arrival_counter >= self.ARRIVAL_CONFIRM_COUNT:
                    rospy.loginfo('[GoHome] Center reached. Switching to align mode.')
                    self.stop_robot()
                    rospy.sleep(0.6)
                    self.state = self.STATE_ALIGN
                    return
                # Not enough consecutive arrivals yet — hold still.
                self.stop_robot()
                return
            else:
                self._arrival_counter = 0

            cmd.linear.x = vx
            cmd.linear.y = vy

            # Concurrent yaw correction: apply gentle angular.z during centering
            # so the robot is already mostly aligned when it reaches the centre.
            if heading_err is not None and abs(heading_err) > self.align_params.angle_tol:
                raw_ang = heading_err * self._center_yaw_kp
                # Clip to a gentle max so rotation doesn't interfere with driving.
                ang = max(-self._center_yaw_max, min(self._center_yaw_max, raw_ang))
                cmd.angular.z = ang

        elif self.state == self.STATE_ALIGN:
            if heading_err is None:
                # No valid heading — hold still until we get a reading.
                self.stop_robot()
                return
            v_ang, aligned = homing.alignment_command(
                heading_err, self.align_params)
            if aligned:
                rospy.loginfo(
                    f'[GoHome] Alignment complete (Error: {heading_err:.2f} deg)')
                self._finish()
                return
            cmd.angular.z = v_ang

        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------ #
    def _publish_room_view(self, ranges, heading_err, cardinals):
        """Render + publish the top-down room model (throttled)."""
        now = rospy.Time.now()
        if now - self._last_view < self._view_period:
            return
        self._last_view = now
        try:
            import cv2
            img = homing.render_room_view(ranges, heading_err, cardinals)
            ok, buf = cv2.imencode('.jpg', img)
            if not ok:
                return
            out = CompressedImage()
            out.header.stamp = now
            out.format = 'jpeg'
            out.data = buf.tobytes()
            self.view_pub.publish(out)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f'[GoHome] room-view render failed: {e}')

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def _finish(self):
        self.enabled = False
        self.state = self.STATE_CENTER
        self._arrival_counter = 0
        self.publish_status()
        self.stop_robot()
        rospy.loginfo('[GoHome] Task finished successfully.')

    def _on_shutdown(self):
        """SAFETY: called on node kill/crash/Ctrl-C. Publishes zero-velocity
        commands multiple times to maximise the chance the motor controller
        receives at least one before this process exits."""
        self.logwarn('Shutdown requested — sending emergency stop.')
        self.enabled = False
        self._spin_active = False
        stop = Twist()
        # Publish several times with small delays: the subscriber may need
        # more than one message to overcome its ramp, and a single publish
        # might be lost if the transport is shutting down simultaneously.
        for _ in range(5):
            try:
                self.cmd_pub.publish(stop)
                rospy.sleep(0.02)
            except Exception:
                pass

if __name__ == '__main__':
    rospy.init_node('go_home_node')
    try:
        GoHomeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass