#!/usr/bin/env python3
import rospy
import numpy as np
from abstract_node import AbstractNode
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Vector3, Twist
from std_srvs.srv import Trigger, TriggerRequest, SetBool, SetBoolRequest
from foma.srv import Float

class JoystickNode(AbstractNode):
    def __init__(self):
        super().__init__('joystick', 'Joystick')

        # mode handling 
        self.mode = "joystick"
        rospy.Subscriber("control/mode", String, self.set_mode)
        self.mode_pub = rospy.Publisher("control/mode", String, queue_size=1)

        #  publishers to MotorControlNode 
        self.pub_vector = rospy.Publisher("motor_control/vector", Vector3, queue_size=1, tcp_nodelay=True)
        self.pub_rotate = rospy.Publisher("motor_control/rotate", Float32, queue_size=1, tcp_nodelay=True)
        self.pub_twist  = rospy.Publisher("motor_control/twist", Twist, queue_size=1, tcp_nodelay=True)

        #  services 
        # fish feeder 
        self.feed_srv  = rospy.ServiceProxy("fish_feeder/feed", Trigger)

        # lidar bypass
        self.lidar_srv = rospy.ServiceProxy("motor_control/bypass_lidar", SetBool)
        self.lidar_bypassed = False   

        # speed control (optional)
        self.speed_srv = rospy.ServiceProxy("motor_control/set_speed", Float)
        self.current_speed = 1.0

        # joystick subscriber
        rospy.Subscriber("/joy", Joy, self.on_joy, queue_size=1, tcp_nodelay=True)

        # for edge detection on buttons (so we don’t spam services)
        self.prev_buttons = []
   
    def set_mode(self, msg: String):
        self.mode = msg.data
        rospy.loginfo(f"[joystick_node] Control mode set to: {self.mode}")


    def on_joy(self, msg: Joy):
        # store buttons for rising edge checks
        if not self.prev_buttons:
            self.prev_buttons = [0] * len(msg.buttons)

        # only move the robot when in joystick mode
        if self.mode != "joystick":
            self.prev_buttons = msg.buttons[:]  # still update state
            return

        # AXES (check with `rostopic echo /joy` if needed)
        lx = msg.axes[0]  # left stick left/right
        ly = msg.axes[1]  # left stick up/down
        rx = msg.axes[3]  # right stick left/right for rotation

        # BUTTONS (Xbox layout)
        BTN_A = msg.buttons[0]
        BTN_B = msg.buttons[1]
        BTN_X = msg.buttons[2]
        BTN_Y = msg.buttons[3]
        BTN_LB = msg.buttons[4]
        BTN_RB = msg.buttons[5]

        #  A: feed fish 
        if BTN_A and not self.prev_buttons[0]:
            try:
                self.feed_srv(TriggerRequest())
                rospy.loginfo("[joystick_node] Feeding triggered")
            except rospy.ServiceException as e:
                rospy.logerr(f"[joystick_node] Feeder service call failed: {e}")

        #  B: toggle lidar bypass 
        if BTN_B and not self.prev_buttons[1]:
            self.lidar_bypassed = not self.lidar_bypassed
            req = SetBoolRequest(data=self.lidar_bypassed)
            try:
                resp = self.lidar_srv(req)
                rospy.loginfo(f"[joystick_node] LIDAR bypass set to {self.lidar_bypassed} ({resp.message})")
            except rospy.ServiceException as e:
                rospy.logerr(f"[joystick_node] LIDAR bypass service call failed: {e}")

        #  X/Y: change global mode if you want 
        if BTN_X and not self.prev_buttons[2]:
            self.mode_pub.publish("manual")
        if BTN_Y and not self.prev_buttons[3]:
            self.mode_pub.publish("fish_feeder")

        #  LB/RB: simple speed control example 
        if BTN_LB and not self.prev_buttons[4]:
            self.current_speed = max(0.2, self.current_speed - 0.2)
            self.speed_srv(self.current_speed)
            rospy.loginfo(f"[joystick_node] Speed down: {self.current_speed:.2f}")
        if BTN_RB and not self.prev_buttons[5]:
            self.current_speed = min(2.0, self.current_speed + 0.2)
            self.speed_srv(self.current_speed)
            rospy.loginfo(f"[joystick_node] Speed up: {self.current_speed:.2f}")

        #  movement: rotate or translate 

        # rotation wins if strong enough
        if abs(rx) > 0.2:
            self.pub_rotate.publish(Float32(rx))
        else:
            # left stick as 2D motion vector
            mag = np.hypot(lx, ly)
            if mag < 0.15:  # deadzone
                vec = Vector3(0.0, 0.0, 0.0)
            else:
                # send raw axes (MotorControlNode normalizes internally)
                vec = Vector3(lx, ly, 0.0)
            self.pub_vector.publish(vec)

        self.prev_buttons = msg.buttons[:]
 
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('joystick_node')
    node = JoystickNode()
    node.run()
