#!/usr/bin/env python3
import rospy
import numpy as np

from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from geometry_msgs.msg import Vector3
from foma.msg import FomaLocation

from abstract_node import AbstractNode


class GoHomeNode(AbstractNode):
    """
    GoHome:
      - enable via service /go_home/enable (SetBool)
      - when enabled, uses /localization/location (FomaLocation) to drive towards (goal_x, goal_y)
      - publishes:
          go_home/enabled (Bool, latched)
          go_home/vector  (Vector3)  -> consumed by MotorControlNode
      - auto-disables when within arrival_tol for stable_ticks consecutive timer callbacks

    Params (ROS):
      ~goal_x (float, default 0.5)
      ~goal_y (float, default 0.5)
      ~arrival_tol (float, default 0.03)
      ~rate_hz (float, default 20.0)
      ~loc_max_age (float, default 0.8)  # seconds
      ~stable_ticks (int, default 6)

      # coordinate fix helpers:
      ~swap_xy (bool, default False)
      ~invert_x (bool, default False)
      ~invert_y (bool, default False)

    Notes:
      - MotorControlNode normalizes the vector you publish, so we only do "stop" vs "go" here.
    """

    def _init_(self):
        super()._init_('go_home', 'GoHome')

        # --- Params ---
        self.goal_x = float(rospy.get_param('~goal_x', 0.5))
        self.goal_y = float(rospy.get_param('~goal_y', 0.5))
        self.arrival_tol = float(rospy.get_param('~arrival_tol', 0.03))
        self.rate_hz = float(rospy.get_param('~rate_hz', 20.0))
        self.loc_max_age = float(rospy.get_param('~loc_max_age', 0.8))
        self.stable_ticks_required = int(rospy.get_param('~stable_ticks', 6))

        # axis correction toggles (useful when it goes to corners)
        self.swap_xy = bool(rospy.get_param('~swap_xy', False))
        self.invert_x = bool(rospy.get_param('~invert_x', False))
        self.invert_y = bool(rospy.get_param('~invert_y', False))

        # --- State (IMPORTANT: these must exist before timers) ---
        self.enabled = False
        self.last_loc = None               # <<< FIX: prevents AttributeError
        self.last_loc_time = rospy.Time(0)
        self._stable_ticks = 0
        self._last_warn = rospy.Time(0)

        # --- ROS I/O ---
        # Latch so MotorControl receives the last state even if it starts later
        self.pub_enabled = rospy.Publisher('go_home/enabled', Bool, queue_size=1, latch=True)
        self.pub_vector = rospy.Publisher('go_home/vector', Vector3, queue_size=1)

        self.sub_loc = rospy.Subscriber('localization/location', FomaLocation, self._on_location, queue_size=1)

        self.srv_enable = rospy.Service('go_home/enable', SetBool, self._srv_enable)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self._on_timer)

        # Publish initial disabled state (important for clean startup)
        self.pub_enabled.publish(Bool(data=False))
        self.pub_vector.publish(Vector3(0.0, 0.0, 0.0))

        self.loginfo("GoHomeNode ready. Call service /go_home/enable (SetBool) to start/stop.")

    def _on_location(self, msg: FomaLocation):
        # Store the last localization message
        self.last_loc = msg
        # Prefer header stamp if present, else now
        try:
            self.last_loc_time = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        except Exception:
            self.last_loc_time = rospy.Time.now()

    def _srv_enable(self, req: SetBoolRequest) -> SetBoolResponse:
        self.enabled = bool(req.data)
        self._stable_ticks = 0

        # Publish mode immediately (latched)
        self.pub_enabled.publish(Bool(data=self.enabled))

        # If disabling, also command stop once
        if not self.enabled:
            self.pub_vector.publish(Vector3(0.0, 0.0, 0.0))
            return SetBoolResponse(success=True, message="GoHome disabled (stop sent).")

        return SetBoolResponse(success=True, message="GoHome enabled.")

    def _warn_throttled(self, text: str, period_sec: float = 2.0):
        now = rospy.Time.now()
        if (now - self._last_warn).to_sec() >= period_sec:
            self._last_warn = now
            self.logwarn(text)

    def _on_timer(self, _evt):
        if not self.enabled:
            return

        # Need localization
        if self.last_loc is None:
            # This is what happens when md5 mismatch / no connection / localization dead
            self.pub_vector.publish(Vector3(0.0, 0.0, 0.0))
            self._warn_throttled("GoHome enabled but no localization received (last_loc is None).")
            return

        # Freshness check
        age = (rospy.Time.now() - self.last_loc_time).to_sec()
        if age > self.loc_max_age:
            self.pub_vector.publish(Vector3(0.0, 0.0, 0.0))
            self._warn_throttled(f"GoHome: localization too old ({age:.2f}s). Stopping until fresh.")
            return

        # Read normalized world coordinates (0..1 expected)
        x = float(self.last_loc.world.x)
        y = float(self.last_loc.world.y)

        # Vector to goal in "world" space
        dx = self.goal_x - x
        dy = self.goal_y - y
        dist = float(np.hypot(dx, dy))

        # Arrived: require stable for a few ticks to avoid flicker
        if dist <= self.arrival_tol:
            self._stable_ticks += 1
            self.pub_vector.publish(Vector3(0.0, 0.0, 0.0))
            if self._stable_ticks >= self.stable_ticks_required:
                self.enabled = False
                self.pub_enabled.publish(Bool(data=False))
                self.loginfo("GoHome: arrived at goal. Auto-disabled.")
            return
        else:
            self._stable_ticks = 0

        # Optional axis fixes
        vx, vy = dx, dy
        if self.swap_xy:
            vx, vy = vy, vx
        if self.invert_x:
            vx = -vx
        if self.invert_y:
            vy = -vy

        # Publish direction (MotorControl normalizes it anyway)
        self.pub_vector.publish(Vector3(vx, vy, 0.0))


if _name_ == "_main_":
    rospy.init_node("go_home_node")
    GoHomeNode()
    rospy.spin()