#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
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
        # self.fish_dir_sub = rospy.Subscriber('fish_detection/state', FishState, self.update_direction)
        self.lidar_sub = rospy.Subscriber('lidar/scans', LaserScan, self.update_lidar, queue_size=10)
        self.manual_subscriber = rospy.Subscriber('gui/motor_control_twist', Twist, self.__manual_control) 
        rospy.Subscriber('gui/motor_control_dir', UInt16, self.update_direction) 
        self.manual_mode_service = rospy.Service('motor_control/motor_mode_control', SetBool, self.__manual_overide)
        self.bypass_lidar_service = rospy.Service('motor_control/bypass_lidar', SetBool, self.__bypass_lidar)
        self.__manual_mode = False
        self.direction = None
        self.scans = None
        try:
            self.motor_control = MotorControl(resetPins = (MOTOR_TOP_BOTTOM_RESET, MOTOR_RIGHT_LEFT_RESET)
                                            ,port = MOTOR_PORT
                                            ,speed = MOTOR_SPEED)
        except BadPinFactory as e:
            rospy.logerr("MotorControlNode: "+e.msg)

        self.__lidar_bypassed = False
        
        self.hComponent, self.vComponent = 0, 0
        self.fBlocked, self.lBlocked, self.bBlocked, self.rBlocked = True, True, True, True
        rospy.on_shutdown(self.__on_shutdown)

    def __bypass_lidar(self, request:SetBoolRequest):
        self.__lidar_bypassed = request.data
        return SetBoolResponse(success = True, message = f"LIDAR {'' if self.__lidar_bypassed else 'not'} bypassed.")

    def __manual_overide(self, request:SetBoolRequest):
        self.__manual_mode = request.data
        return SetBoolResponse(success = True, message = "Motor control turned {}.".format("manual" if request.data else "auto"))

    def run(self):
        while not rospy.is_shutdown():
            if not self.__manual_mode:
                self.move_by_components(self.hComponent, self.vComponent)
            rospy.sleep(0.005)

    def move_by_components(self, hComponent, vComponent):
        '''
            Forward = 0
            Left = 90
            Backward = 180
            Right = 270
        '''
        if not self.__lidar_bypassed:
            if hComponent != 0 and vComponent !=0:
                if hComponent > 0 and vComponent > 0:
                    if self.__check_angel_range(range(-45,45)):
                        hComponent = 0
                    if self.__check_angel_range(range(225,315)):
                        vComponent = 0
                elif hComponent < 0 and self.__check_angel_range(range(135,225)):
                    hComponent = 0
                if vComponent > 0 and self.__check_angel_range(range(225,315)):
                    vComponent = 0
                elif vComponent < 0 and self.__check_angel_range(range(45,135)):
                    vComponent = 0
            elif hComponent != 0:
                if hComponent > 0 and self.__check_angel_range(range(-36,36)):
                    hComponent = 0
                elif hComponent < 0 and self.__check_angel_range(range(144,216)):
                    hComponent = 0
            elif vComponent != 0:
                if vComponent > 0 and self.__check_angel_range(range(234,306)):
                    vComponent = 0
                elif vComponent < 0 and self.__check_angel_range(range(54,126)):
                    vComponent = 0
        self.motor_control.move_by_components(hComponent, vComponent)                

    def update_direction(self, direction: UInt16):
        # Process fish direction and adjust motors accordingly
        self.direction = direction.direction
        self.hComponent, self.vComponent = self.__split_components()

    def update_lidar(self, scans:LaserScan):
        # self.loginfo("Got here")
        # Process lidar data and adjust motors to avoid obstacles
        self.scans = scans.ranges
        # if self.direction is None:
        #     return
        self.fBlocked = self.__check_angel_range(range(-45,45))# or self.__check_angel_range(range(305,360))
        self.lBlocked = self.__check_angel_range(range(45,135))
        self.bBlocked = self.__check_angel_range(range(135,225))
        self.rBlocked = self.__check_angel_range(range(225,315))
        # low, high = self.direction - 90, self.direction + 90
        # if self.direction in range(0,90):
        #     self.vBlocked = self.__check_angel_range(range(0,45)) or self.__check_angel_range(range(305,360))
        #     self.hBlocked = self.__check_angel_range(range(45,135))
        
        # elif self.direction in range(90,180):
        #     self.vBlocked = self.__check_angel_range(range(135,225))
        #     self.hBlocked = self.__check_angel_range(range(45,135))
            
        # elif self.direction in range(180,270):
        #     self.vBlocked = self.__check_angel_range(range(135,225))
        #     self.hBlocked = self.__check_angel_range(range(225,315))
            
        # elif self.direction in range(270,360) :
        #     self.vBlocked = self.__check_angel_range(range(0,45)) or self.__check_angel_range(range(305,360))
        #     self.hBlocked = self.__check_angel_range(range(225,315))

        # else:
        #     rospy.logerr("MotorControlNode: Wrong direction, stopping motors.")
        #     self.hComponent, self.vComponent = 0,0
        #     return
        
        # Splits the speed components according to fish's direction
        # self.hComponent, self.vComponent = self.__split_components()


    def __check_angel_range(self, angle_range):
        filtered_scans = np.array(self.scans)[angle_range] 
        distance_checks = filtered_scans < MotorControlNode.safety_distance_vector
        if np.any(distance_checks):
            self.logwarn(np.where(distance_checks)[0] + angle_range[0])
            return True
        return False

    def __split_components(self, deg = True):
        if deg:
            factor = np.pi/180
        else:
            factor = 1
        horizontal_component = np.sin(self.direction * factor)
        vertical_component = np.cos(self.direction * factor)
        return (horizontal_component, vertical_component)

    def __manual_control(self, twist: Twist):
        """
        Callback function to handle incoming Twist messages from the cmd_vel topic.
        Maps linear and angular velocities to motor control.
        """
        # self.loginfo(f"System: {self._system_on}, Manual: {self.__manual_mode}")
        if self.__manual_mode:# and self._system_on:
            # Linear velocity controls forward/backward motion
            # print(twist)
            
            # self.loginfo("got here")
            vertical_component = twist.linear.x
            
            # Angular velocity controls left/right motion
            horizontal_component = twist.linear.y
            # print(vertical_component, horizontal_component)
            # Move motors based on components
            if twist.angular.z != 0 and twist.linear.x ==0 and twist.linear.y==0:
                self.motor_control.rotate(twist.angular.z)
            else:
                self.move_by_components(horizontal_component, vertical_component)

    
    def __on_shutdown(self):
        if self.motor_control:
            self.logwarn("MotorControlNode: Stopping motors.")
            # self.hBlocked, self.vBlocked = True, True
            self.motor_control.move_by_components(0, 0)

if __name__ == "__main__":
    rospy.init_node('motor_control_node')
    rospy.loginfo("Motor Control Node: node created.")
    # print("Started")
    motor_ctrl = MotorControlNode()
    # self.logwarn("Distances vector: {}".format(MotorControlNode.safety_distance_vector))
    motor_ctrl.run()
    
    rospy.spin()