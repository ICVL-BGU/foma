#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16MultiArray
from std_srvs.srv import SetBool, SetBoolResponse

from foma.msg import FomaLocation

class GoHomeNode:
    def __init__(self):
        rospy.init_node("go_home_node")

        self.enabled = False
        self.last_loc = None
        self.blocked_angles = set()

        # Tunables
        self.goal_x = 0.5
        self.goal_y = 0.5
        self.arrival_tol = rospy.get_param("~arrival_tol", 0.03)   # normalized units (~15cm if room=5m)
        self.max_cmd = rospy.get_param("~max_cmd", 0.8)            # 0..1 scaling
        self.publish_hz = rospy.get_param("~publish_hz", 20)

        # Pub/sub
        self.pub_vec = rospy.Publisher("motor_control/vector", Vector3, queue_size=10)
        rospy.Subscriber("localization/location", FomaLocation, self.on_location)
        rospy.Subscriber("motor_control/blocked", Int16MultiArray, self.on_blocked)

        # Enable/disable service (so you don't need GUI edits)
        self.srv = rospy.Service("go_home/enable", SetBool, self.on_enable)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_hz), self.on_timer)

        rospy.loginfo("GoHomeNode ready. Call service /go_home/enable (SetBool) to start/stop.")

    def on_enable(self, req):
        self.enabled = bool(req.data)
        if not self.enabled:
            self.pub_vec.publish(Vector3(0.0, 0.0, 0.0))
        return SetBoolResponse(success=True, message=f"go_home enabled={self.enabled}")

    def on_location(self, msg: FomaLocation):
        self.last_loc = msg

    def on_blocked(self, msg: Int16MultiArray):
        self.blocked_angles = set(int(a) for a in msg.data)

    def desired_angle_deg_from_vector(self, vx: float, vy: float) -> float:
        """
        Match your GUI convention:
        GUI builds vector from angle as:
          x = -sin(rad)
          y =  cos(rad)
        Invert it:
          angle = atan2(-x, y)  in degrees, mapped to [0,360)
        """
        ang = math.degrees(math.atan2(-vx, vy))
        ang = (ang + 360.0) % 360.0
        return ang

    def is_blocked(self, ang_deg: float) -> bool:
        tol = 2  #degrees
        for a in self.blocked_angles:
            if abs(((ang_deg - a + 180) % 360) - 180) <= tol:
                return True
        return False

    def on_timer(self, _):
        if not self.enabled or self.last_loc is None:
            return

        x = float(self.last_loc.world.x)
        y = float(self.last_loc.world.y)

        dx = self.goal_x - x
        dy = self.goal_y - y
        dist = math.hypot(dx, dy)

        #arrived
        if dist < self.arrival_tol:
            self.pub_vec.publish(Vector3(0.0, 0.0, 0.0))
            return

        #normalize
        ux = dx / dist
        uy = dy / dist

        #(dist normalized: 0..~0.7 max from corner to center)
        speed = min(self.max_cmd, 1.5 * dist)  # tune 1.0..2.0 multiplier if needed
        vx = ux * speed
        vy = uy * speed

        #if blocked, stop 
        ang = self.desired_angle_deg_from_vector(vx, vy)
        if self.is_blocked(ang):
            self.pub_vec.publish(Vector3(0.0, 0.0, 0.0))
            return

        self.pub_vec.publish(Vector3(vx, vy, 0.0))

if __name__ == "__main__":
    GoHomeNode()
    rospy.spin()
