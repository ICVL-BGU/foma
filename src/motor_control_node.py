#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3
from abstract_node import AbstractNode
from etc.settings import MOTOR_PORT, MOTOR_SPEED, MOTOR_TOP_BOTTOM_RESET, MOTOR_RIGHT_LEFT_RESET, SAFETY_DISTANCE
from etc.MotorControl import *
from gpiozero import BadPinFactory
import numpy as np
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

class MotorControlNode(AbstractNode):
    safety_distance_vector = SAFETY_DISTANCE / np.cos(np.abs(np.arange(-45,45) * np.pi/180 ))
    def __init__(self):
        super().__init__('motor_control', 'Motor control')
        rospy.Subscriber('lidar/scans', LaserScan, self.__update_lidar, queue_size=10)
        rospy.Subscriber('motor_control/angle', Float32, self.__handle_angle) 
        rospy.Subscriber('motor_control/vector',  Vector3,  self.__handle_vector)
        rospy.Subscriber('motor_control/twist',   Twist,    self.__handle_twist)
        rospy.Subscriber('motor_control/rotate',  Float32,  self.__handle_rotate)

        rospy.Service('motor_control/bypass_lidar', SetBool, self.__bypass_lidar)
        rospy.Subscriber('motor_control/set_speed', Float32, self.__set_speed)
        self.scans = None

        try:
            self.__motor_control = MotorControl(resetPins = (MOTOR_TOP_BOTTOM_RESET, MOTOR_RIGHT_LEFT_RESET)
                                            ,accl = 0
                                            ,brake = 0
                                            ,port = MOTOR_PORT
                                            ,speed = MOTOR_SPEED)
        except BadPinFactory as e:
            self.logerr("MotorControlNode: "+e.msg)

        self.__lidar_bypassed = False
        self.__speed = 0.5

        self.hComponent, self.vComponent = 0, 0
        rospy.on_shutdown(self.__on_shutdown)

    def __bypass_lidar(self, request:SetBoolRequest):
        self.__lidar_bypassed = request.data
        return SetBoolResponse(success = True, message = f"LIDAR {'' if self.__lidar_bypassed else 'not'} bypassed.")

    def __set_speed(self, request:Float32):
        self.__speed = request.data

    def __move_by_components(self, hComponent, vComponent):
        '''
            Forward = 0
            Left = 90
            Backward = 180
            Right = 270
        '''
        if not self.__lidar_bypassed:
            if hComponent != 0 and vComponent !=0:
                if hComponent > 0 and vComponent > 0:
                    if self.__is_sector_blocked(range(-45,45)):
                        hComponent = 0
                    if self.__is_sector_blocked(range(225,315)):
                        vComponent = 0
                elif hComponent < 0 and self.__is_sector_blocked(range(135,225)):
                    hComponent = 0
                if vComponent > 0 and self.__is_sector_blocked(range(225,315)):
                    vComponent = 0
                elif vComponent < 0 and self.__is_sector_blocked(range(45,135)):
                    vComponent = 0
            elif hComponent != 0:
                if hComponent > 0 and self.__is_sector_blocked(range(-36,36)):
                    hComponent = 0
                elif hComponent < 0 and self.__is_sector_blocked(range(144,216)):
                    hComponent = 0
            elif vComponent != 0:
                if vComponent > 0 and self.__is_sector_blocked(range(234,306)):
                    vComponent = 0
                elif vComponent < 0 and self.__is_sector_blocked(range(54,126)):
                    vComponent = 0
        # self.__motor_control.move_by_components(hComponent, vComponent)       
        self.__motor_control.move_by_components(hComponent*self.__speed, vComponent*self.__speed)              

    def __handle_angle(self, msg: Float32):
        angle = msg.data % 360
        h, v = self.__split_components(angle_deg=angle)
        self.__move_by_components(h, v)

    def __handle_vector(self, msg: Vector3):
        x, y = msg.x, msg.y
        mag = np.hypot(x, y)
        if mag < 1e-6:
            self.__move_by_components(0, 0)
            return
        self.__move_by_components(x/mag, y/mag)

    def __handle_twist(self, msg: Twist):
        self.__move_by_components(msg.linear.y, msg.linear.x)

    def __handle_rotate(self, msg: Float32):
        z = msg.data
        # if not self.__lidar_bypassed:
        #     if z > 0 and self.lBlocked:
        #         self.logwarn("Left blocked → rotation canceled.")
        #         return
        #     if z < 0 and self.rBlocked:
        #         self.logwarn("Right blocked → rotation canceled.")
        #         return
        self.__motor_control.rotate(z*self.__speed)

    def __update_lidar(self, scans:LaserScan):
        self.scans = np.array(scans.ranges)

    def __is_sector_blocked(self, sector):
        filtered_scans = np.array(self.scans)[sector] 
        distance_checks = filtered_scans < MotorControlNode.safety_distance_vector[45-len(sector)//2:45+len(sector)//2]
        if np.any(distance_checks):
            return True
        return False
    
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
        scans = np.array(self.scans)[rng]
        return np.any(scans < thr)

    def __split_components(self, angle_deg: float):
        rad = angle_deg * np.pi/180.0
        return np.sin(rad), np.cos(rad)
    
    def __on_shutdown(self):
        if self.__motor_control:
            self.logwarn("Stopping motors.")
            # self.hBlocked, self.vBlocked = True, True
            self.__motor_control.move_by_components(0, 0)

if __name__ == "__main__":
    rospy.init_node('motor_control_node')
    rospy.loginfo("Motor Control Node: node created.")
    
    MotorControlNode()
    
    rospy.spin()