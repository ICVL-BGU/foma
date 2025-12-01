#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, Vector3
from std_srvs.srv import Trigger

class Joystick_node:
    def __init__(self):
        rospy.init_node("joystick_node")
        rospy.loginfo("Joystick node started")

        # mode of control
        self.mode = "disabled"  # default mode

        # publishers to MotorControlNode
        self.pub_twist = rospy.Publisher("motor_control/twist", Twist, queue_size=10)
        self.pub_vector = rospy.Publisher("motor_control/vector", Vector3, queue_size=10)
        self.pub_rotate = rospy.Publisher("motor_control/rotate", Float32, queue_size=10)

        #services
        self.feed_srv = rospy.ServiceProxy('fish_feeder/feed', Trigger) 
        self.lidar_srv = rospy.ServiceProxy('motor_control/bypass_lidar', Trigger)

        #mode listener
        rospy.Subscriber('control/mode', String, self.set_mode)
        
        #main joystick
        rospy.Subscriber('/joy', Joy, self.on_joy)

    def set_mode(self, msg):
        self.mode = msg.data
        rospy.loginfo(f"Control mode set to: {self.mode}")

    
    def on_joy(self, msg):

        if self.mode != "joystick": 
            return

        #left stick
        lx = msg.axes[0]  # Left/Right
        ly = msg.axes[1]  # Forward/Backward    

        #right stick
        rx = msg.axes[3]  # rotation

        #Button mappings
        BTN_A = joy.buttons[0]
        BTN_B = joy.buttons[1]
        BTN_X = joy.buttons[2]
        BTN_Y = joy.buttons[3]

        #feeding fish
        if BTN_A == 1:
            try:
                self.feed_srv()
                rospy.loginfo("feeding triggered")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
        
        #bypass lidar
        if BTN_B:
            try:
                self.lidar_srv()
                rospy.loginfo("toggle LIDAR bypass")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

        # change mode
        if BTN_X:
            rospy.Publisher('control/mode', String, queue_size=1).publish("manual") 
        if BTN_Y:
            rospy.Publisher('control/mode', String, queue_size=1).publish("fish_feeder")
      
        #movment normalized
        #if rotating
        if abs(rx) > 0.2:
            self.pub_rotate(Float32(rx))
            return
        
        #if moving
        mag = np.hypot(lx, ly)
        if mag < 0.15:
            vec = Vector3(0, 0, 0)
        else:
            vec = Vector3(lx, ly, 0)

        self.pub_vector.publish(vec)
        

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    Joystick_node().run()