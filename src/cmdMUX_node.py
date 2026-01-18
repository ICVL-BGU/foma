#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class cmdMUX_node:
    def __init__(self):
        self.manual_topic = rospy.get_param("~manual_topic", "manual_control/twist")
        self.auto_topic   = rospy.get_param("~auto_topic",   "go_home/twist")
        self.auto_en_topic = rospy.get_param("~auto_enabled_topic", "go_home/enabled")
        self.out_topic    = rospy.get_param("~out_topic",    "motor_control/twist")

        self.auto_active = False
        self.last_manual = Twist()
        self.last_auto   = Twist()
        self.last_manual_time = rospy.Time(0)

        self.manual_timeout = rospy.get_param("~manual_timeout", 0.25)  # seconds
        self.rate_hz = rospy.get_param("~rate_hz", 20)

        self.pub = rospy.Publisher(self.out_topic, Twist, queue_size=10)
        rospy.Subscriber(self.manual_topic, Twist, self._on_manual, queue_size=10)
        rospy.Subscriber(self.auto_topic,   Twist, self._on_auto,   queue_size=10)
        rospy.Subscriber(self.auto_en_topic, Bool, self._on_auto_enabled, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._tick)

    def _on_auto_enabled(self, msg: Bool):
        self.auto_active = bool(msg.data)

    def _on_manual(self, msg: Twist):
        self.last_manual = msg
        self.last_manual_time = rospy.Time.now()

    def _on_auto(self, msg: Twist):
        self.last_auto = msg

    def _tick(self, _evt):
        now = rospy.Time.now()
        manual_fresh = (now - self.last_manual_time).to_sec() <= self.manual_timeout

        if manual_fresh:
            cmd = self.last_manual                     
        elif self.auto_active:
            cmd = self.last_auto                        
        else:
            cmd = self.last_manual                     

        self.pub.publish(cmd)

if __name__ == "__main__":
    rospy.init_node("cmd_mux_node")
    CmdMuxNode()
    rospy.spin()
