#!/usr/bin/env python3

import rospy
from abstract_node import AbstractNode
from fov.srv import Light, LightRequest, LightResponse
from gpiozero import PWMLED, BadPinFactory

class LightDimmerNode(AbstractNode):
    def __init__(self):
        super().__init__('light_dimmer', 'Light dimmer')
        self.fish_feed_sub = rospy.Service('light_dimmer/change', Light, self.feed)
        try:
            #TODO
            self.out = PWMLED(pin = 17, initial_value = 0.5, frequency = 700)
        except BadPinFactory as e:
            rospy.logwarn(e.msg)

    def dim(self, data: LightRequest):
        self.out.value = data.data / 255
        return LightResponse(success = True, message = "Light dimmer dimmed.")
    
if __name__ == "__main__":
    rospy.init_node('feeder_node')
    ceiling_cam_handler = LightDimmerNode()
    rospy.spin()
