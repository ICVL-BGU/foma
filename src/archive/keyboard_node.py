#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import curses

def keyboard_control():
    rospy.init_node('keyboard_control_node')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Initialize the Twist message
    twist = Twist()

    # Initialize curses
    stdscr = curses.initscr()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)  # Non-blocking input

    try:
        while not rospy.is_shutdown():
            key = stdscr.getch()

            # Reset twist values
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            if key == curses.KEY_UP:
                twist.linear.x = 1.0  # Move forward
            elif key == curses.KEY_DOWN:
                twist.linear.x = -1.0  # Move backward
            elif key == curses.KEY_LEFT:
                twist.angular.z = 1.0  # Rotate counterclockwise (CCW)
            elif key == curses.KEY_RIGHT:
                twist.angular.z = -1.0  # Rotate clockwise (CW)
            elif key == ord('q'):
                twist.linear.x = 0.0
                twist.angular.z = 1.0  # Rotate CCW
            elif key == ord('e'):
                twist.linear.x = 0.0
                twist.angular.z = -1.0  # Rotate CW
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0  # Stop if no key is pressed

            pub.publish(twist)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
    finally:
        curses.endwin()

if __name__ == '__main__':
    try:
        keyboard_control()
    except rospy.ROSInterruptException:
        pass
