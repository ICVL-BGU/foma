#!/usr/bin/env python3

from pickletools import dis
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Int16MultiArray
from geometry_msgs.msg import Twist, Vector3
from abstract_node import AbstractNode
from etc.settings import *
from etc.MotorControl import *
from gpiozero import BadPinFactory
import numpy as np
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

class MotorControlNode(AbstractNode):
    safety_distance_vector = SAFETY_DISTANCE / np.cos(np.abs(np.arange(-45,45) * np.pi/180 ))
    def __init__(self):
        super().__init__('motor_control', 'Motor control')

        self.__blocked_publisher = rospy.Publisher('motor_control/blocked', Int16MultiArray, queue_size=10)
        self.__speed_publisher = rospy.Publisher('motor_control/speed', Twist, queue_size=10)
        
        rospy.Subscriber('lidar/scans', LaserScan, self.__update_lidar, queue_size=10)
        rospy.Subscriber('motor_control/angle', Float32, self.__handle_angle) 
        rospy.Subscriber('motor_control/vector',  Vector3,  self.__handle_vector)
        rospy.Subscriber('motor_control/twist',   Twist,    self.__handle_twist)
        rospy.Subscriber('motor_control/rotate',  Float32,  self.__handle_rotate)

        rospy.Service('motor_control/bypass_lidar', SetBool, self.__bypass_lidar)
        rospy.Subscriber('motor_control/set_speed', Float32, self.__set_speed)
        
        self.__scans = None

        try:
            self.__motor_control = MotorControl(resetPins = (MOTOR_TOP_BOTTOM_RESET, MOTOR_RIGHT_LEFT_RESET)
                                            ,encoderChannels = ((MOTOR_TOP_CHA, MOTOR_TOP_CHB),
                                                               (MOTOR_BOTTOM_CHA, MOTOR_BOTTOM_CHB),
                                                               (MOTOR_LEFT_CHA, MOTOR_LEFT_CHB),
                                                               (MOTOR_RIGHT_CHA, MOTOR_RIGHT_CHB))
                                            ,accl = 5
                                            ,brake = 0
                                            ,port = MOTOR_PORT
                                            ,speed = MOTOR_SPEED)
        except BadPinFactory as e:
            self.logerr("MotorControlNode: "+e.msg)

        self.__movement = 'linear'  # or 'rotate'

        self.__lidar_bypassed = False
        self.__speed = 0.5

        self.__desired_h = 0.0
        self.__desired_v = 0.0
        self.__desired_rotate = 0.0

        self.__current_h = 0.0
        self.__current_v = 0.0
        self.__current_rotate = 0.0

        self.__rate_hz       = 10
        self.__max_ramp_time = 1.5    # → takes 2 s to go from –1 to +1; adjust X here
        self.__accel_step    = 1.5/(self.__max_ramp_time*self.__rate_hz)

        self.__last_cmd_time = rospy.Time(0)

        self.__fresh_threshold = 1.0  # seconds, how long to keep the last command fresh

        # self.hComponent, self.vComponent = 0, 0
        rospy.on_shutdown(self.__on_shutdown)

    def run(self):
        rate = rospy.Rate(self.__rate_hz)  # 10 Hz
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt  = (now - self.__last_cmd_time).to_sec()

            if dt < self.__fresh_threshold:
                # 1) move current toward desired
                self.__current_h      = self.__ramp(self.__current_h,      self.__desired_h,      self.__accel_step)
                self.__current_v      = self.__ramp(self.__current_v,      self.__desired_v,      self.__accel_step)
                self.__current_rotate = self.__ramp(self.__current_rotate, self.__desired_rotate, self.__accel_step)

                # 2) issue command
                if abs(self.__current_rotate) > 1e-6:
                    self.__rotate_motor(self.__current_rotate)
                else:
                    self.__move_by_components(self.__current_h, self.__current_v)
            else:
                # command expired → ramp back to zero
                self.__current_h      = self.__ramp(self.__current_h,      0.0, self.__accel_step)
                self.__current_v      = self.__ramp(self.__current_v,      0.0, self.__accel_step)
                self.__current_rotate = self.__ramp(self.__current_rotate, 0.0, self.__accel_step)

                if abs(self.__current_rotate) > 1e-6:
                    self.__rotate_motor(self.__current_rotate)
                else:
                    self.__move_by_components(self.__current_h, self.__current_v)

            self.__speed_publisher.publish(Twist(
                linear = Vector3(self.__current_v * self.__speed, self.__current_h * self.__speed, 0.0),
                angular = Vector3(0.0, 0.0, self.__current_rotate * self.__speed)
            ))
            rate.sleep()

    def __ramp(self, current, target, step):
        '''
        Ramp the current value towards the target value by a fixed step.
        If the target is within the step range, it will return the target value.
        '''
        delta = target - current
        if abs(delta) <= step:
            return target
        return current + step * np.sign(delta)
    
    def __bypass_lidar(self, request:SetBoolRequest):
        self.__lidar_bypassed = request.data
        return SetBoolResponse(success = True, message = f"LIDAR {'' if self.__lidar_bypassed else 'not'} bypassed.")

    def __set_speed(self, request:Float32):
        self.__speed = request.data

    def __move_by_components(self, h_component, v_component):
        '''
            Forward = 0
            Left = 90
            Backward = 180
            Right = 270
        '''
        if self.__movement != 'linear':
            self.__movement = 'linear'
            self.__motor_control.reset_encoders()

        if not self.__lidar_bypassed and self.__scans is not None:
            # Check blocking logic exactly as before:
            if h_component != 0 and v_component != 0:
                # Quadrant logic
                if h_component > 0 and v_component > 0:
                    if self.__is_sector_blocked(range(-45, 45)):
                        h_component = 0
                    if self.__is_sector_blocked(range(225, 315)):
                        v_component = 0
                elif h_component < 0 and self.__is_sector_blocked(range(135, 225)):
                    h_component = 0
                if v_component > 0 and self.__is_sector_blocked(range(225, 315)):
                    v_component = 0
                elif v_component < 0 and self.__is_sector_blocked(range(45, 135)):
                    v_component = 0
            elif h_component != 0:
                if h_component > 0 and self.__is_sector_blocked(range(-36, 36)):
                    h_component = 0
                elif h_component < 0 and self.__is_sector_blocked(range(144, 216)):
                    h_component = 0
            elif v_component != 0:
                if v_component > 0 and self.__is_sector_blocked(range(234, 306)):
                    v_component = 0
                elif v_component < 0 and self.__is_sector_blocked(range(54, 126)):
                    v_component = 0

        # Send final components to motor
        if self.__motor_control:
            self.__motor_control.move_by_components(
                h_component * self.__speed, v_component * self.__speed
            )

    def __handle_angle(self, msg: Float32):
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
        if mag < 1e-6:
            # Zero‐motion request
            self.__desired_h = 0.0
            self.__desired_v = 0.0
        else:
            self.__desired_h = x / mag
            self.__desired_v = y / mag

        self.__desired_rotate = 0.0
        self.__last_cmd_time = rospy.Time.now()

    def __handle_twist(self, msg: Twist):
        self.__desired_h = msg.linear.y
        self.__desired_v = msg.linear.x
        self.__desired_rotate = 0.0
        self.__last_cmd_time = rospy.Time.now()

    def __handle_rotate(self, msg: Float32):
        self.__desired_rotate = msg.data
        self.__desired_h = 0.0
        self.__desired_v = 0.0
        self.__last_cmd_time = rospy.Time.now()

    def __update_lidar(self, scans: LaserScan):
        self.__scans = np.array(scans.ranges)

        # Reshape scans into (4, 90) to match safety_vec shape for elementwise comparison
        scans_reshaped = self.__scans.reshape(4, 90)
        distance_checks = (scans_reshaped < MotorControlNode.safety_distance_vector)
        distance_checks = distance_checks.reshape(-1)  # back to 360‐length

        blocked_indices = np.where(distance_checks)[0]
        blocked_array = Int16MultiArray()
        blocked_array.data = blocked_indices.tolist()
        self.__blocked_publisher.publish(blocked_array)

    def __is_sector_blocked(self, sector):
        filtered_scans = np.array(self.__scans)[sector] 
        distance_checks = filtered_scans < MotorControlNode.safety_distance_vector[45-len(sector)//2:45+len(sector)//2]
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
        if self.__motor_control:
            self.loginfo("Stopping motors.")
            self.__motor_control.shutdown()

if __name__ == "__main__":
    rospy.init_node("motor_control_node")
    rospy.loginfo("Motor Control Node: node created.")
    MotorControlNode().run()