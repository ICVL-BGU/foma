#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16MultiArray, Bool
from std_srvs.srv import SetBool, SetBoolResponse
from foma.msg import FomaLocation

class GoHomeNode:
    def __init__(self):
        rospy.init_node("go_home_node")

        self.enabled = False
        self.last_loc = None
        self.blocked_angles = set()

        self.goal_x = rospy.get_param("~goal_x", 0.5)
        self.goal_y = rospy.get_param("~goal_y", 0.5)
        self.arrival_tol = rospy.get_param("~arrival_tol", 0.03)
        self.max_cmd = rospy.get_param("~max_cmd", 0.8)
        self.publish_hz = rospy.get_param("~publish_hz", 20)

        self.pub_vec = rospy.Publisher("go_home/vector", Vector3, queue_size=10)
        self.pub_enabled = rospy.Publisher("go_home/enabled", Bool, queue_size=1, latch=True)

        rospy.Subscriber("localization/location", FomaLocation, self.on_location)
        rospy.Subscriber("motor_control/blocked", Int16MultiArray, self.on_blocked)

        self.srv = rospy.Service("go_home/enable", SetBool, self.on_enable)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_hz), self.on_timer)

        self.pub_enabled.publish(Bool(self.enabled))
        rospy.loginfo("GoHomeNode ready. Call service /go_home/enable (SetBool) to start/stop.")

    def on_enable(self, req):
        self.enabled = bool(req.data)
        self.pub_enabled.publish(Bool(self.enabled))

        if not self.enabled:
            self.pub_vec.publish(Vector3(0.0, 0.0, 0.0))

        return SetBoolResponse(success=True, message=f"go_home enabled={self.enabled}")

    def on_location(self, msg: FomaLocation):
        self.last_loc = msg

    def on_blocked(self, msg: Int16MultiArray):
        self.blocked_angles = set(int(a) for a in msg.data)

    def on_timer(self, _):
        if not self.enabled or self.last_loc is None:
            return

        x = float(self.last_loc.world.x)
        y = float(self.last_loc.world.y)

        dx = self.goal_x - x
        dy = self.goal_y - y
        dist = math.hypot(dx, dy)

        if dist < self.arrival_tol:
            self.pub_vec.publish(Vector3(0.0, 0.0, 0.0))
            self.enabled = False
            self.pub_enabled.publish(Bool(self.enabled))
            return

        ux = dx / dist
        uy = dy / dist

        speed = min(self.max_cmd, 1.5 * dist)
        h = ux * speed
        v = uy * speed

        self.pub_vec.publish(Vector3(h, v, 0.0))

if __name__ == "__main__":
    GoHomeNode()
    rospy.spin()
