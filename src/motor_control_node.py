#!/usr/bin/env python3

from pickletools import dis
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from abstract_node import AbstractNode
from etc.MotorControl import *
from etc import scan_geometry as geom
from etc.heading_hold import HeadingHold, HeadingHoldParams
from etc.rotation import Rotator
from gpiozero import BadPinFactory
import numpy as np
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from foma.srv import Float, FloatRequest, FloatResponse, String
import math

class MotorControlNode(AbstractNode):
    def __init__(self):
        super().__init__('motor_control', 'Motor control')

        # Quarter turns closed on the LIDAR (service rotate/quarter_turns).
        # While one runs it owns the motors: a real command from the joystick
        # or the GUI cancels it, zeros and the fish are ignored. Created
        # before the subscribers, which feed it.
        self.__rotator = Rotator(self.__rotator_command)

        self.__speed_publisher = rospy.Publisher('motor_control/speed', TwistStamped, queue_size=10, tcp_nodelay=True)
        
        rospy.Subscriber('fish_detection/state', TwistStamped, self.__handle_fish_direction, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('lidar/scans', LaserScan, self.__update_lidar, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('motor_control/angle', Float32, self.__handle_angle, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('motor_control/vector',  Vector3,  self.__handle_vector, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('motor_control/twist',   Twist,    self.__handle_twist, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('motor_control/rotate',  Float32,  self.__handle_rotate, queue_size=1, tcp_nodelay=True)

        rospy.Service('motor_control/bypass_lidar', SetBool, self.__bypass_lidar)
        rospy.Service('motor_control/set_speed', Float, self.__set_speed)
        rospy.Service('motor_control/heading_hold', SetBool, self.__set_heading_hold)
        self.__heading_pub = rospy.Publisher('motor_control/heading', Vector3, queue_size=1)
        # x = drift across the line (m, + = right of it), y = correction duty
        self.__lane_pub = rospy.Publisher('motor_control/lane', Vector3, queue_size=1)

        g = rospy.get_param
        self.__foma_camera_frame_shape = tuple(g('/FOMA_CAMERA_FRAME_SHAPE'))
        self.__direction_epsilon = g('/DIRECTION_EPSILON')
        _safety_distance = g('/SAFETY_DISTANCE')
        self.__safety_distance_vector = _safety_distance / np.cos(np.abs(np.arange(-45, 45) * np.pi / 180))

        try:
            self.__motor_control = MotorControl(resetPins = (g('/MOTOR_TOP_BOTTOM_RESET'), g('/MOTOR_RIGHT_LEFT_RESET'))
                                            ,encoderChannels = ((g('/MOTOR_TOP_CHA'),    g('/MOTOR_TOP_CHB')),
                                                               (g('/MOTOR_BOTTOM_CHA'), g('/MOTOR_BOTTOM_CHB')),
                                                               (g('/MOTOR_LEFT_CHA'),   g('/MOTOR_LEFT_CHB')),
                                                               (g('/MOTOR_RIGHT_CHA'),  g('/MOTOR_RIGHT_CHB')))
                                            ,accl = 5
                                            ,brake = 0
                                            ,port = g('/MOTOR_PORT')
                                            ,speed = g('/MOTOR_SPEED'))
        except BadPinFactory as e:
            self.logerr("MotorControlNode: "+e.msg)

        self.__scans = None
        self.__movement = 'linear'  # or 'rotate'
        self._mode = 'idle'

        self.__lidar_bypassed = False
        self.__speed = 1.0

        self.__desired_h = 0.0
        self.__desired_v = 0.0
        self.__desired_rotate = 0.0

        self.__current_h = 0.0
        self.__current_v = 0.0
        self.__current_rotate = 0.0

        self.__rate_hz       = 50
        self.__max_ramp_time = 2    # → takes 2 s to go from –1 to +1; adjust X here
        self.__accel_step    = 0.03#self.__max_ramp_time / self.__steps

        self.__last_cmd_time = rospy.Time(0)

        self.__fresh_threshold = 1.0  # seconds, how long to keep the last command fresh

        # Heading hold: keep square to the room while translating.
        hh = lambda k, d: g('~heading_hold/' + k, d)
        self.__hold_enabled = bool(hh('enabled', True))
        self.__hold_min_conf = float(hh('min_conf', 0.5))
        self.__hold_offset = float(g('/HEADING_OFFSET_DEG', 0.0))   # shared with go_home and rotate
        self.__hold = HeadingHold(HeadingHoldParams(
            kp=float(hh('kp', 0.060)), ki=float(hh('ki', 0.030)),
            max_corr=float(hh('max_corr', 0.30)),
            deadband=float(hh('deadband_deg', 0.1)),
            filter_alpha=float(hh('filter_alpha', 0.5)),
            i_limit=float(hh('i_limit', 0.15)),
            runaway_deg=float(hh('runaway_deg', 6.0)),
            sign=1 if int(g('/ALIGN_SIGN', 1)) >= 0 else -1))
        self.__hold_engaged = False
        self.__hold_last_t = None

        # Lane hold: driving along a room axis, keep the side-wall offset it
        # started with, so sideways wheel slip cannot add up to a diagonal.
        lh = lambda k, d: g('~lane_hold/' + k, d)
        self.__lane_enabled = bool(lh('enabled', True))
        self.__lane_kp = float(lh('kp', 3.0))              # duty per metre of drift
        self.__lane_max = float(lh('max_corr', 0.35))
        # Sideways correction lands on the wheels the drive leaves idle; they
        # are standing still, so anything under static friction does nothing.
        self.__lane_min = float(lh('min_corr', 0.24))
        self.__lane_deadband = float(lh('deadband_m', 0.010))
        self.__lane_jump = float(lh('jump_m', 0.25))       # bigger = an obstacle, re-anchor
        self.__lane_dir_tol = float(lh('dir_tol_deg', 5.0))  # command turn that starts a new line
        self.__lane_dir = None
        self.__lane_ref = None
        # Joystick: within this many degrees of an axis counts as on it.
        self.__axis_snap_deg = float(g('~axis_snap_deg', 12.0))

        rospy.on_shutdown(self.__on_shutdown)

    def run(self):
        rate = rospy.Rate(self.__rate_hz)  # 10 Hz
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt  = (now - self.__last_cmd_time).to_sec()

            if dt > self.__fresh_threshold:
                self.__desired_h = 0.0
                self.__desired_v = 0.0
                self.__desired_rotate = 0.0

            # command expired → ramp back to zero
            self.__current_h      = self.__ramp(self.__current_h,      self.__desired_h)
            self.__current_v      = self.__ramp(self.__current_v,      self.__desired_v)
            self.__current_rotate = self.__ramp(self.__current_rotate, self.__desired_rotate)

            if abs(self.__current_rotate) > 1e-6:
                self.__release_hold()       # an explicit turn always wins
                self.__rotate_motor(self.__current_rotate)
            else:
                self.__current_h, self.__current_v = self.__check_blocking(self.__current_h, self.__current_v)
                h, v = self.__lane_correction(self.__current_h, self.__current_v)
                h, v = self.__check_blocking(h, v)
                corr = self.__heading_correction(h, v)
                if corr is None:
                    self.__move_by_components(h, v)
                else:
                    self.__move_with_rotation(h, v, corr)

                self.__speed_publisher.publish(
                    TwistStamped(
                        twist = Twist(linear = Vector3(self.__current_v * self.__speed, self.__current_h * self.__speed, 0.0),
                                    angular = Vector3(0.0, 0.0, self.__current_rotate * self.__speed)),
                        header = Header(stamp = rospy.Time.now())
                    )
                )
            rate.sleep()

    def __ramp(self, current, target):
        '''
        Ramp the current value towards the target value by a fixed step.
        If the target is within the step range, it will return the target value.
        '''
        delta = target - current
        if abs(delta) <= self.__accel_step:
            return target
        return current + self.__accel_step * np.sign(delta)
    
    def __bypass_lidar(self, request:SetBoolRequest):
        self.__lidar_bypassed = request.data
        return SetBoolResponse(success = True, message = f"LIDAR {'' if self.__lidar_bypassed else 'not'} bypassed.")

    def __set_speed(self, request:FloatRequest):
        self.__speed = request.data
        return FloatResponse(result = True)

    def __set_heading_hold(self, request:SetBoolRequest):
        self.__hold_enabled = bool(request.data)
        self.__hold.tripped = False
        self.__release_hold()
        return SetBoolResponse(success = True, message = f"Heading hold {'on' if self.__hold_enabled else 'off'}.")

    def __release_hold(self):
        if self.__hold_engaged:
            self.__hold.reset()
        self.__hold_engaged = False
        self.__hold_last_t = None

    def __snap_to_axis(self, h, v):
        '''
            A stick pushed "straight" is never exactly straight; a few degrees
            off drives a diagonal. Near an axis, drive exactly along it.
        '''
        ang = math.degrees(math.atan2(v, h))
        nearest = round(ang / 90.0) * 90.0
        if abs(ang - nearest) <= self.__axis_snap_deg:
            r = math.radians(nearest)
            return float(round(math.cos(r))), float(round(math.sin(r)))
        return h, v

    def __wall_fit(self):
        '''One wall-line fit per scan, shared by the heading and lane holds.'''
        if getattr(self, '_MotorControlNode__fit', None) is None:
            self.__fit = geom.wall_fit(self.__scans / 1000.0)
        return self.__fit

    def __lane_position(self):
        '''(forward, right) position in the room, metres, from the
        perpendicular distances to the fitted walls. A range read along a
        fixed beam would tilt with the lidar's mounting and turn a straight
        drive into a diagonal; a perpendicular distance does not. None if a
        wall is not visible.'''
        _, _, d = self.__wall_fit()
        if None in d:
            return None
        fwd, left, back, right = d
        return (back - fwd) / 2.0, (left - right) / 2.0

    def __lane_correction(self, h, v):
        '''
            Keep a straight drive on its line, in any direction (axis or an
            explicit diagonal, joystick or fish). Forward = h, Right = v (see
            __check_blocking). Only the drift *across* the commanded
            direction is corrected; a new direction starts a new line.
        '''
        dh, dv = self.__desired_h, self.__desired_v
        dmag = math.hypot(dh, dv)
        if not (self.__lane_enabled and self.__hold_enabled and not self.__hold.tripped
                and self.__scans is not None and dmag > 1e-6 and math.hypot(h, v) > 0.05):
            self.__lane_ref = self.__lane_dir = None
            return h, v

        pos = self.__lane_position()
        if pos is None:
            return h, v
        uh, uv = dh / dmag, dv / dmag
        new_dir = (self.__lane_dir is None or
                   math.degrees(math.acos(max(-1.0, min(1.0,
                       uh * self.__lane_dir[0] + uv * self.__lane_dir[1]))))
                   > self.__lane_dir_tol)
        if new_dir or self.__lane_ref is None:
            self.__lane_ref, self.__lane_dir = pos, (uh, uv)
            return h, v

        # Signed drift across the line (normal = direction turned to the right).
        nh, nv = -uv, uh
        e = (pos[0] - self.__lane_ref[0]) * nh + (pos[1] - self.__lane_ref[1]) * nv
        if abs(e) > self.__lane_jump:          # an obstacle, not drift: new line
            self.__lane_ref = pos
            return h, v
        c = 0.0
        if abs(e) >= self.__lane_deadband:
            c = math.copysign(min(max(self.__lane_kp * abs(e), self.__lane_min),
                                  self.__lane_max), e)
        self.__lane_pub.publish(Vector3(e, c, 0.0))
        return h - c * nh, v - c * nv

    def __heading_correction(self, h_component, v_component):
        '''
            Rotation to mix into a translation, or None to drive as before
            (not translating, hold off or tripped). Heading is the wall
            direction folded to [-45, 45), so the target is always the
            nearest room axis.
        '''
        if not self.__hold_enabled or self.__hold.tripped or self.__scans is None \
                or math.hypot(h_component, v_component) < 0.05:
            self.__release_hold()
            return None

        now = rospy.get_time()
        dt = 0.0 if self.__hold_last_t is None else now - self.__hold_last_t
        self.__hold_last_t = now
        self.__hold_engaged = True

        heading, conf, _ = self.__wall_fit()
        if heading is not None and conf >= self.__hold_min_conf:
            heading = geom.fold_angle(heading - self.__hold_offset)
        else:
            heading = None
        corr = self.__hold.update(heading, dt, now)
        if self.__hold.tripped:
            self.logwarn("Heading hold disabled: error kept growing while "
                         "correcting (ALIGN_SIGN wrong?). Re-enable with "
                         "motor_control/heading_hold.")
            return None
        self.__heading_pub.publish(Vector3(
            float('nan') if heading is None else heading, conf, corr))
        return corr

    def __move_with_rotation(self, h_component, v_component, rotation):
        if self.__movement != 'linear':
            self.__movement = 'linear'
            self.__motor_control.reset_encoders()
        if self.__motor_control:
            self.__motor_control.move(h_component * self.__speed,
                                      v_component * self.__speed,
                                      rotation)

    def __set_mode(self, request:String):
        self._mode = request.data
        return String(result = f"Mode set to {self._mode}")

    def __move_by_components(self, h_component, v_component):
        if self.__movement != 'linear':
            self.__movement = 'linear'
            self.__motor_control.reset_encoders()

        # Send final components to motor
        if self.__motor_control:
            self.__motor_control.move_by_components(
                h_component * self.__speed, v_component * self.__speed
            )

    def __check_blocking(self, h_component, v_component):
        '''
            Forward = 0
            Left = 90
            Backward = 180
            Right = 270
        '''
        if not self.__lidar_bypassed and self.__scans is not None:
            if h_component > 0 and self.__is_sector_blocked(range(-43, 43)):
                h_component = 0
            elif h_component < 0 and self.__is_sector_blocked(range(137, 223)):
                h_component = 0
            if v_component > 0 and self.__is_sector_blocked(range(227, 313)):
                v_component = 0
            elif v_component < 0 and self.__is_sector_blocked(range(47, 133)):
                v_component = 0

        return h_component, v_component

    def __turn_owns_motors(self, magnitude, source):
        '''True if a running turn keeps the motors and this command is dropped.
        A real command (not a zero) cancels the turn and goes through.'''
        if not self.__rotator.active:
            return False
        if magnitude > 0.05:
            self.__rotator.request_cancel(f"{source} command")
            return False
        return True

    def __rotator_command(self, duty):
        self.__desired_rotate = duty
        self.__desired_h = 0.0
        self.__desired_v = 0.0
        self.__last_cmd_time = rospy.Time.now()

    def __handle_fish_direction(self, msg: TwistStamped):
        if self._mode != 'trial' or self.__rotator.active:
            return
        
        # Get fish location
        fish_x = msg.twist.linear.x
        fish_y = msg.twist.linear.y

        # Calculate the vector from the image center to the fish location
        height, width = self.__foma_camera_frame_shape
        center_x = width // 2
        center_y = height // 2
        # 1. compute the pixel‐offset with y inverted
        vector_x = fish_x - center_x
        vector_y = center_y - fish_y   # <— flip the sign here!

        # 2. get the two angles
        center_to_fish_angle = math.degrees(math.atan2(vector_y, vector_x))

        direction_angle = math.degrees(
            math.atan2(msg.twist.angular.y,   # note: angular.y is the ‘up/down’ component
                    msg.twist.angular.x)   #        angular.x is the ‘left/right’ component
        )

        diff = (direction_angle - center_to_fish_angle + 180) % 360 - 180

        if abs(diff) <= self.__direction_epsilon:
            self.__handle_angle(Float32(direction_angle))
        else:
            self.__handle_vector(Vector3(0, 0, 0))

    def __handle_angle(self, msg: Float32):
        if self.__turn_owns_motors(1.0, 'motor_control/angle'):
            return
        angle = msg.data % 360
        h, v = self.__split_components(angle_deg=angle)

        # Store as the new “desired” motion
        self.__desired_h = h
        self.__desired_v = v
        self.__desired_rotate = 0.0  # cancel any pending rotation
        self.__last_cmd_time = rospy.Time.now()

    def __handle_vector(self, msg: Vector3):
        x, y = msg.x, msg.y
        mag = np.hypot(x, y)
        if self.__turn_owns_motors(mag, 'motor_control/vector'):
            return
        if mag < 1e-6:
            # Zero‐motion request
            self.__desired_h = 0.0
            self.__desired_v = 0.0
        else:
            self.__desired_h, self.__desired_v = self.__snap_to_axis(x / mag, y / mag)

        self.__desired_rotate = 0.0
        self.__last_cmd_time = rospy.Time.now()

    def __handle_twist(self, msg: Twist):
        mag = max(math.hypot(msg.linear.x, msg.linear.y), abs(msg.angular.z))
        if self.__turn_owns_motors(mag, 'motor_control/twist'):
            return
        self.__desired_h = msg.linear.y
        self.__desired_v = msg.linear.x
        self.__desired_rotate = msg.angular.z
        self.__last_cmd_time = rospy.Time.now()

    def __handle_rotate(self, msg: Float32):
        if self.__turn_owns_motors(abs(msg.data), 'motor_control/rotate'):
            return
        self.__desired_rotate = msg.data
        self.__desired_h = 0.0
        self.__desired_v = 0.0
        self.__last_cmd_time = rospy.Time.now()

    def __update_lidar(self, scans: LaserScan):
        self.__scans = np.array(scans.ranges)
        self.__fit = None                   # recomputed on demand, once per scan
        self.__rotator.on_scan(scans.ranges)

    def __is_sector_blocked(self, sector):
        filtered_scans = np.array(self.__scans)[sector] 
        distance_checks = filtered_scans < self.__safety_distance_vector[45-len(sector)//2:45+len(sector)//2]
        return np.any(distance_checks)
    
    def __is_direction_blocked(self, heading_deg: float) -> bool:
        half = 576.0/2.0
        corners = np.array([
            [ +half, +half],  # TL
            [ +half, -half],  # TR
            [ -half, -half],  # BR
            [ -half, +half],  # BL
        ])
        θ = np.deg2rad(heading_deg % 360)
        offset = 100.0 * np.array([np.cos(θ), np.sin(θ)])
        trans = corners + offset

        inside = np.all(np.abs(trans) <= half, axis=1)  # which corners stayed inside

        # build only the two endpoint spokes + all adj‑adj transform edges
        segments = []
        N = 4
        for i in range(N):
            # neighbors in the 0→1→2→3 cycle
            prev_i = (i - 1) % N
            next_i = (i + 1) % N

            # 1) spoke corner→transformed only if exactly one neighbor is inside
            if (not inside[i]) and (inside[prev_i] ^ inside[next_i]):
                segments.append((corners[i], trans[i]))

            # 2) edge between two adjacent transformed corners, if both moved outside
            if (not inside[i]) and (not inside[next_i]):
                segments.append((trans[i], trans[next_i]))

        # now compute per‑beam thresholds exactly as before…
        beam_thresh = {}
        for p, q in segments:
            a = (np.rad2deg(np.arctan2(p[1], p[0])) ) % 360
            b = (np.rad2deg(np.arctan2(q[1], q[0])) ) % 360
            diff = (b - a + 360) % 360
            if diff <= 180:
                span, step = int(diff), +1
            else:
                span, step = int(360 - diff), -1

            for k in range(span + 1):
                ang = int((a + step * k) % 360)
                if ang in beam_thresh:
                    continue

                d = np.array([np.cos(np.deg2rad(ang)),
                            np.sin(np.deg2rad(ang))])
                A = np.column_stack((q - p, -d))
                bvec = -p
                try:
                    u, t = np.linalg.solve(A, bvec)
                except np.linalg.LinAlgError:
                    continue
                if 0.0 <= u <= 1.0 and t >= 0.0:
                    beam_thresh[ang] = t

        if not beam_thresh:
            return False

        # collect a continuous beam range
        angles = sorted(beam_thresh.keys())
        lo, hi = angles[0], angles[-1]
        if hi >= lo:
            rng = np.arange(lo, hi + 1, dtype=int)
        else:
            rng = np.arange(lo, hi + 361, dtype=int) % 360

        # fill a threshold array
        thr = np.full(rng.shape, np.inf, dtype=float)
        for ang, t in beam_thresh.items():
            idx = np.where(rng == ang)[0][0]
            thr[idx] = t

        # vectorized compare
        scans = np.array(self.__scans)[rng]
        return np.any(scans < thr)

    def __split_components(self, angle_deg: float):
        rad = angle_deg * np.pi/180.0
        return np.sin(rad), np.cos(rad)
    
    def __rotate_motor(self, z_value):
        """
        Rotation command (unchanged, but only called from main loop).
        """
        if self.__movement != 'rotate':
            self.__movement = 'rotate'
            self.__motor_control.reset_encoders()

        if self.__motor_control:
            self.__motor_control.rotate(z_value * self.__speed)
    
    def __stop_motor(self):
        """
        A simple “stop everything” call. You can decide what “stop” means:
        here, we send zero velocity both to translation and rotation.
        """
        if self.__motor_control:
            self.__motor_control.move_by_components(0.0, 0.0)
            self.__motor_control.rotate(0.0)
    
    def __on_shutdown(self):
        self.__rotator.shutdown()
        if self.__motor_control:
            self.loginfo("Stopping motors.")
            self.__motor_control.shutdown()

if __name__ == "__main__":
    rospy.init_node("motor_control_node")
    rospy.loginfo("Motor Control Node: node created.")
    MotorControlNode().run()