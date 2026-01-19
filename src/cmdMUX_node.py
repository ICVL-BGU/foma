#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

class CmdMuxNode:
    def __init__(self):
        self.manual_twist_topic = rospy.get_param("~manual_twist_topic", "manual_control/twist")
        self.auto_twist_topic   = rospy.get_param("~auto_twist_topic",   "go_home/twist")
        self.out_twist_topic    = rospy.get_param("~out_twist_topic",    "motor_control/twist")

        self.manual_rot_topic   = rospy.get_param("~manual_rot_topic",   "manual_control/rotate")
        self.auto_rot_topic     = rospy.get_param("~auto_rot_topic",     "go_home/rotate")
        self.out_rot_topic      = rospy.get_param("~out_rot_topic",      "motor_control/rotate")

        self.auto_enabled_topic = rospy.get_param("~auto_enabled_topic", "go_home/enabled")

        self.manual_timeout = rospy.get_param("~manual_timeout", 0.25)
        self.rate_hz = rospy.get_param("~rate_hz", 20)

        self.auto_active = False

        self.last_manual_twist = Twist()
        self.last_auto_twist = Twist()
        self.last_manual_twist_time = rospy.Time(0)

        self.last_manual_rot = Float32(0.0)
        self.last_auto_rot = Float32(0.0)
        self.last_manual_rot_time = rospy.Time(0)

        self.twist_pub = rospy.Publisher(self.out_twist_topic, Twist, queue_size=10)
        self.rot_pub = rospy.Publisher(self.out_rot_topic, Float32, queue_size=10)

        rospy.Subscriber(self.manual_twist_topic, Twist, self._on_manual_twist, queue_size=10)
        rospy.Subscriber(self.auto_twist_topic, Twist, self._on_auto_twist, queue_size=10)

        rospy.Subscriber(self.manual_rot_topic, Float32, self._on_manual_rot, queue_size=10)
        rospy.Subscriber(self.auto_rot_topic, Float32, self._on_auto_rot, queue_size=10)

        rospy.Subscriber(self.auto_enabled_topic, Bool, self._on_auto_enabled, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._tick)

    def _on_auto_enabled(self, msg: Bool):
        self.auto_active = bool(msg.data)

    def _on_manual_twist(self, msg: Twist):
        self.last_manual_twist = msg
        self.last_manual_twist_time = rospy.Time.now()

    def _on_auto_twist(self, msg: Twist):
        self.last_auto_twist = msg

    def _on_manual_rot(self, msg: Float32):
        self.last_manual_rot = msg
        self.last_manual_rot_time = rospy.Time.now()

    def _on_auto_rot(self, msg: Float32):
        self.last_auto_rot = msg

    def _tick(self, _evt):
        now = rospy.Time.now()

        manual_twist_fresh = (now - self.last_manual_twist_time).to_sec() <= self.manual_timeout
        manual_rot_fresh   = (now - self.last_manual_rot_time).to_sec() <= self.manual_timeout

        if manual_twist_fresh:
            out_twist = self.last_manual_twist
        elif self.auto_active:
            out_twist = self.last_auto_twist
        else:
            out_twist = Twist()

        if manual_rot_fresh:
            out_rot = self.last_manual_rot
        elif self.auto_active:
            out_rot = self.last_auto_rot
        else:
            out_rot = Float32(0.0)

        self.twist_pub.publish(out_twist)
        self.rot_pub.publish(out_rot)

if __name__ == "__main__":
    rospy.init_node("cmd_mux_node")
    CmdMuxNode()
    rospy.spin()
