#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from etc.MotorControl import MotorControl  # Import your MotorControl class
from etc.settings import MOTOR_TOP_BOTTOM_RESET, MOTOR_RIGHT_LEFT_RESET, MOTOR_PORT, MOTOR_SPEED

class MotorControlNode:
    def __init__(self):
        
        # Initialize MotorControl with reset pins, port, and speed settings
        self.motor_control = MotorControl(
            resetPins=(MOTOR_TOP_BOTTOM_RESET, MOTOR_RIGHT_LEFT_RESET),
            port=MOTOR_PORT,
            speed=MOTOR_SPEED
        )
        
        # Subscriber to the 'cmd_vel' topic for velocity commands
        self.cmd_vel_subscriber = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)

        rospy.loginfo("MotorControlNode initialized and ready to receive velocity commands.")

    def cmd_vel_callback(self, twist: Twist):
        """
        Callback function to handle incoming Twist messages from the cmd_vel topic.
        Maps linear and angular velocities to motor control.
        """
        # Linear velocity controls forward/backward motion
        # print(twist)
        vertical_component = twist.linear.x
        
        # Angular velocity controls left/right motion
        horizontal_component = twist.linear.y
        # print(vertical_component, horizontal_component)
        # Move motors based on components
        if twist.angular.z != 0 and twist.linear.x ==0 and twist.linear.y==0:
            self.motor_control.rotate(twist.angular.z)
        else:
            self.motor_control.move_by_components(horizontal_component, vertical_component)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('manual_control_node')
    node = MotorControlNode()
    node.run()
