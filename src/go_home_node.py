#!/usr/bin/env python3
"""Homing node: square the robot up to the walls, centre it in the room, and
perform precise closed-loop relative rotations.

A thin ROS shell — all control logic lives in the hardware-independent,
unit-tested components in ``etc/`` (alignment, centering, homing). Alignment
and centering are independent, so they are exposed separately as well as
chained:

    go_home/enable  (SetBool)  align -> settle -> centre -> verify
    go_home/align   (SetBool)  align only
    go_home/center  (SetBool)  centre only
    rotation/spin   (Float)    signed degrees, + = CCW; non-blocking
    rotation/stop   (SetBool)  abort the current rotation

Safety: motion is commanded only while a controller says it is unfinished,
motor_control/twist has a subscriber, LiDAR scans are arriving, the requesting
client is still alive (gui/heartbeat), and the operation timeout has not
expired. Any of those failing calls _hard_stop(). The timeouts live in the
20 Hz publish timer, not the LiDAR callback, so they fire even when the LiDAR
is the thing that died.
"""
import numpy as np
import rospy

from abstract_node import AbstractNode
from etc import scan_geometry as geom
from etc.alignment import AlignmentController, AlignParams
from etc.centering import CenteringController, CenteringParams
from etc.homing import HomingRoutine, PreciseRotator, RotationParams
from foma.srv import Float, FloatResponse
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse


class GoHomeNode(AbstractNode):

    # A single dropped message must never leave the robot driving.
    HARD_STOP_REPEATS = 5
    STOP_TAIL_S = 0.6       # seconds of zero velocity published after a stop

    def __init__(self):
        super().__init__('go_home', 'Go Home')

        g = rospy.get_param

        # --- controllers (each independently constructed and reusable) --- #
        align_params = AlignParams(
            kp=g('~align_kp', 0.018),
            kd=g('~align_kd', 0.004),
            min_ang_v=g('~align_min_v', 0.13),
            max_ang_v=g('~align_max_v', 0.40),
            angle_tol=g('~align_tol_deg', 2.5),
            filter_alpha=g('~align_filter_alpha', 0.35),
            slow_zone=g('~align_slow_zone_deg', 12.0),
            max_slew=g('~align_max_slew', 0.05))

        center_params = CenteringParams(
            kp=g('~center_kp', 3.0),
            min_v=g('~center_min_v', 0.35),
            max_speed=g('~center_max_v', 0.85),
            arrival_tol=g('~center_tol_m', 0.10),
            settle_zone=g('~center_settle_zone_m', 0.30),
            alpha=g('~center_filter_alpha', 0.5),
            stall_time=g('~center_stall_time', 1.2),
            stall_boost=g('~center_stall_boost', 1.4))

        self._align_ctrl = AlignmentController(
            params=align_params, confirm_frames=g('~align_confirm_frames', 5))
        self._center_ctrl = CenteringController(
            params=center_params,
            confirm_frames=g('~center_confirm_frames', 5),
            drift_tol=g('~center_drift_tol_deg', 8.0))
        self._routine = HomingRoutine(
            mode=HomingRoutine.FULL,
            align_ctrl=self._align_ctrl,
            center_ctrl=self._center_ctrl,
            settle_frames=g('~settle_frames', 6))

        self._rotator = PreciseRotator(RotationParams(
            tol=g('~rot_tol_deg', 2.0),
            slow_zone=g('~rot_slow_zone_deg', 20.0),
            cmd=g('~spin_cmd', 0.35),
            min_cmd=g('~spin_min_cmd', 0.18),
            timeout=g('~spin_timeout', 25.0),
            stall_time=g('~spin_stall_time', 2.5),
            wrong_way_grace=g('~spin_wrong_way_grace', 2.0),
            blind_time=g('~spin_blind_time', 1.5)))

        # --- runtime state --- #
        self.enabled = False              # homing routine running
        self._homing_timeout = g('~homing_timeout', 60.0)
        self._homing_start = None

        self._spin_start = None
        self._spin_target = 0.0
        self._spin_watch_clients = False  # arm the client-drop check per spin

        self._last_cmd = Twist()
        self._last_lidar = rospy.Time(0)
        self._lidar_timeout = g('~lidar_timeout', 0.5)
        self._stop_until = rospy.Time(0)  # keep publishing zeros until this

        # Armed only once a beat has been seen, so a headless launch (joystick,
        # rosservice call) isn't crippled by a watchdog for a client that never
        # existed. 2 s = 10 missed beats at the GUI's 5 Hz.
        self._heartbeat_timeout = g('~heartbeat_timeout', 2.0)
        self._last_heartbeat = None

        self._view_period = rospy.Duration(1.0 / 10.0)
        self._last_view = rospy.Time(0)

        # --- ROS I/O --- #
        self.cmd_pub = rospy.Publisher('motor_control/twist', Twist,
                                       queue_size=1, tcp_nodelay=True)
        self.pub_enabled = rospy.Publisher('go_home/enabled', Bool,
                                           queue_size=1, latch=True)
        self.spin_active_pub = rospy.Publisher('rotation/active', Bool,
                                               queue_size=1, latch=True)
        self.view_pub = rospy.Publisher('go_home/room_view', CompressedImage,
                                        queue_size=1, tcp_nodelay=True)

        rospy.Subscriber('lidar/scans', LaserScan, self.on_lidar,
                         queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('gui/heartbeat', Bool, self.on_heartbeat,
                         queue_size=1, tcp_nodelay=True)

        rospy.Service('go_home/enable', SetBool, self.on_enable)
        rospy.Service('go_home/align', SetBool, self.on_align_only)
        rospy.Service('go_home/center', SetBool, self.on_center_only)
        rospy.Service('rotation/spin', Float, self.on_spin)
        rospy.Service('rotation/stop', SetBool, self.on_spin_stop)

        # 20 Hz command pump + safety supervisor.
        self._cmd_timer = rospy.Timer(rospy.Duration(0.05), self._publish_loop)

        self.publish_status()
        self._publish_spin_active()
        rospy.on_shutdown(self._on_shutdown)
        self.loginfo('Node started (align / centre / precise rotation).')

    # ------------------------------------------------------------------ #
    #  Status helpers                                                      #
    # ------------------------------------------------------------------ #
    def publish_status(self):
        self.pub_enabled.publish(Bool(data=self.enabled))

    def _publish_spin_active(self):
        self.spin_active_pub.publish(Bool(data=self._rotator.active))

    @property
    def _busy(self):
        return self.enabled or self._rotator.active

    def stop_robot(self):
        """Publish one zero command and latch it as what the 20 Hz pump repeats."""
        self._last_cmd = Twist()
        self.cmd_pub.publish(self._last_cmd)

    def _hard_stop(self, reason='hard stop', warn=True):
        """The single funnel every stop path goes through — completion,
        timeout, stall, LiDAR loss, motor-node loss, client disconnect.

        Clears the activity flags first so no callback can re-arm a command
        behind our back, then publishes zero repeatedly.
        """
        was_busy = self._busy
        was_spinning = self._rotator.active

        self.enabled = False
        self._rotator.active = False
        self._homing_start = None
        self._spin_start = None
        self._spin_watch_clients = False
        self._last_cmd = Twist()

        zero = Twist()
        for _ in range(self.HARD_STOP_REPEATS):
            try:
                self.cmd_pub.publish(zero)
            except Exception:
                break
        self._stop_until = rospy.Time.now() + rospy.Duration(self.STOP_TAIL_S)

        self.publish_status()
        if was_spinning:
            self._publish_spin_active()
        if was_busy:
            if warn:
                self.logwarn(f'HARD STOP — {reason}')
            else:
                self.loginfo(f'Stopped — {reason}')

    def _start_routine(self, mode, label):
        self._routine.reset(mode=mode)
        self.enabled = True
        self._homing_start = rospy.Time.now()
        self._last_cmd = Twist()
        self.publish_status()
        self.loginfo(f'{label} started.')
        return SetBoolResponse(success=True, message=f'{label} started')

    def _handle_enable(self, req, mode, label):
        # A new request always supersedes whatever was running: a rotation and
        # a homing routine must never drive the motors at the same time.
        if self._rotator.active:
            self._rotator.cancel('superseded by homing request')
            self._publish_spin_active()
        if not bool(req.data):
            self._hard_stop(f'{label} disabled by client')
            return SetBoolResponse(success=True, message=f'{label} stopped')
        return self._start_routine(mode, label)

    def on_enable(self, req):
        return self._handle_enable(req, HomingRoutine.FULL, 'GoHome')

    def on_align_only(self, req):
        return self._handle_enable(req, HomingRoutine.ALIGN_ONLY, 'Align-only')

    def on_center_only(self, req):
        return self._handle_enable(req, HomingRoutine.CENTER_ONLY,
                                   'Centre-only')

    def on_spin(self, req):
        """Spin ``req.data`` degrees (signed, + = CCW). Non-blocking: returns
        immediately, ``rotation/active`` reports completion."""
        degrees = float(req.data)

        if abs(degrees) <= self._rotator.params.tol:
            self._hard_stop('spin request below tolerance')
            return FloatResponse(result=True)

        # Rotation takes over from homing rather than fighting it. The homing
        # deadline has to be cleared with it, or the (now irrelevant) homing
        # timeout would fire mid-rotation and abort a perfectly good spin.
        if self.enabled:
            self.enabled = False
            self._homing_start = None
            self.publish_status()

        now = rospy.Time.now()
        self._spin_target = degrees
        self._spin_start = now
        self._rotator.start(degrees, now.to_sec())
        # Arm the client-drop watchdog only if somebody is actually listening
        # for the result, so a headless `rosservice call` is not aborted.
        self._spin_watch_clients = self.spin_active_pub.get_num_connections() > 0
        self._last_cmd = Twist()
        self._publish_spin_active()
        self.loginfo(f'Spinning {degrees:+.1f} deg (LiDAR closed loop).')
        return FloatResponse(result=True)

    def on_spin_stop(self, req):
        self._hard_stop('rotation stopped by client')
        return SetBoolResponse(success=True, message='Rotation stopped')

    def on_heartbeat(self, msg):
        self._last_heartbeat = rospy.Time.now()

    # ---------------- 20 Hz command pump + safety supervisor ------------- #
    def _publish_loop(self, event):
        now = rospy.Time.now()

        if not self._busy:
            # Keep asserting zero briefly after a stop, so a dropped message
            # cannot leave the motor node ramping.
            if now < self._stop_until:
                self.cmd_pub.publish(Twist())
            return

        if self._safety_check(now):
            return

        self.cmd_pub.publish(self._last_cmd)

    def _safety_check(self, now):
        """Run every watchdog. Returns True if the robot was stopped."""
        # Nobody is listening to our commands.
        if self.cmd_pub.get_num_connections() == 0:
            self._hard_stop('no subscriber on motor_control/twist '
                            '(motor_control_node down?)')
            return True

        # The client that asked for this motion disappeared.
        if self._last_heartbeat is not None:
            age = (now - self._last_heartbeat).to_sec()
            if age > self._heartbeat_timeout:
                self._hard_stop(f'client heartbeat lost ({age:.1f}s) — '
                                f'GUI closed or disconnected')
                return True
        if self._spin_watch_clients and self._rotator.active and \
                self.spin_active_pub.get_num_connections() == 0:
            self._hard_stop('rotation client disconnected')
            return True

        # LiDAR died: we are flying blind.
        if (now - self._last_lidar).to_sec() > self._lidar_timeout:
            self._hard_stop('no LiDAR scans')
            return True

        # Operation timeouts live here, not in the LiDAR callback, so they fire
        # even when scans have stopped arriving.
        if self._spin_start is not None:
            elapsed = (now - self._spin_start).to_sec()
            if elapsed > self._rotator.params.timeout:
                self._hard_stop(f'rotation timeout after {elapsed:.1f}s '
                                f'(travelled {self._rotator.tracker.total:+.1f} '
                                f'of {self._spin_target:+.1f} deg)')
                return True
        if self._homing_start is not None:
            elapsed = (now - self._homing_start).to_sec()
            if elapsed > self._homing_timeout:
                self._hard_stop(f'homing timeout after {elapsed:.1f}s '
                                f'(state={self._routine.state})')
                return True
        return False

    # ---------------- LiDAR callback: the main control loop -------------- #
    def on_lidar(self, msg):
        now = rospy.Time.now()
        self._last_lidar = now

        # lidar/scans republishes at 100 Hz while the sensor only completes a
        # revolution at ~5 Hz, and the wall-angle estimator is a 360-beam
        # Python loop — derive the geometry once, and only when it's needed.
        view_due = (now - self._last_view) >= self._view_period
        if not (self._busy or view_due):
            return

        ranges = np.asarray(msg.ranges, dtype=float) / 1000.0  # mm -> m
        heading_err = geom.wall_heading_error(ranges)
        cardinals = geom.cardinal_distances(ranges)

        if view_due:
            self._publish_room_view(now, ranges, heading_err, cardinals)

        if self._rotator.active:
            self._step_rotation(ranges, heading_err)
            return

        if self.enabled:
            self._step_homing(ranges, heading_err, cardinals)

    def _step_rotation(self, ranges, heading_err):
        """PreciseRotator owns every termination condition; all this has to
        guarantee is that ``done`` results in a hard stop."""
        status = self._rotator.update(heading_err, rospy.Time.now().to_sec(),
                                      ranges=ranges)

        if status.done:
            if status.success:
                self.loginfo(f'Rotation complete: travelled '
                             f'{status.travelled:+.1f} of '
                             f'{self._spin_target:+.1f} deg ({status.reason}).')
            else:
                self.logwarn(f'Rotation aborted ({status.reason}) after '
                             f'{status.travelled:+.1f} of '
                             f'{self._spin_target:+.1f} deg.')
            self._hard_stop(f'rotation finished: {status.reason}',
                            warn=not status.success)
            return

        cmd = Twist()
        cmd.angular.z = status.angular_z
        self._last_cmd = cmd
        rospy.loginfo_throttle(
            1.0, f'[GoHome] SPIN travelled={status.travelled:+.1f} '
                 f'remaining={status.remaining:+.1f} cmd={status.angular_z:+.3f}')

    def _step_homing(self, ranges, heading_err, cardinals):
        """One align/centre step, driven entirely by the routine state machine."""
        result = self._routine.step(ranges, now=rospy.Time.now().to_sec(),
                                    heading_err=heading_err,
                                    cardinals=cardinals)

        if result.done:
            self.loginfo(f'Homing finished ({result.reason}).')
            self._hard_stop(f'homing finished: {result.reason}', warn=False)
            return

        # Translation and rotation are mutually exclusive at the hardware, so
        # the routine only ever produces one of them at a time.
        cmd = Twist()
        cmd.linear.x = result.vx
        cmd.linear.y = result.vy
        cmd.angular.z = result.wz
        self._last_cmd = cmd

        heading = ('n/a' if result.heading_err is None
                   else f'{result.heading_err:+.2f} deg')
        dist = 'n/a' if result.dist is None else f'{result.dist:.3f} m'
        rospy.loginfo_throttle(
            2.0, f'[GoHome] {result.state} ({result.reason}) '
                 f'heading={heading} dist={dist} '
                 f'cmd=({result.vx:+.2f},{result.vy:+.2f},{result.wz:+.2f})')

    def _publish_room_view(self, now, ranges, heading_err, cardinals):
        """Render + publish the top-down room model (throttled in on_lidar)."""
        self._last_view = now
        try:
            import cv2
            img = geom.render_room_view(ranges, heading_err, cardinals)
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

    def _on_shutdown(self):
        """Node kill / crash / Ctrl-C: leave the motors stopped."""
        self.enabled = False
        self._rotator.active = False
        stop = Twist()
        for _ in range(self.HARD_STOP_REPEATS):
            try:
                self.cmd_pub.publish(stop)
                rospy.sleep(0.02)
            except Exception:
                break


if __name__ == '__main__':
    rospy.init_node('go_home_node')
    try:
        GoHomeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
