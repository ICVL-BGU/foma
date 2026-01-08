#!/usr/bin/env python3

import rospy
from abstract_node import AbstractNode
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
import PololuStepper as PS
from etc.settings import FEEDER_DIR_PIN, FEEDER_STEP_PIN, FEEDER_ENABLE_PIN, FEEDER_STEPS_PER_REVOLUTION, FEEDER_MICROSTEPS, FEEDER_RPM, FEEDER_HOLE_OFFSET_ANGLE

class FeederNode(AbstractNode):
    def __init__(self):
        super().__init__('fish_feeder', 'Fish feeder')
        
        rospy.Service('fish_feeder/feed', Trigger, self.feed)
        rospy.Service('fish_feeder/empty', Trigger, self.empty)

        self.feeder = PS.DRV8834(FEEDER_STEPS_PER_REVOLUTION,
                                 FEEDER_DIR_PIN,
                                 FEEDER_STEP_PIN,
                                 FEEDER_ENABLE_PIN)
        
        self.feeder.begin(FEEDER_RPM, FEEDER_MICROSTEPS)

    def feed(self, data: TriggerRequest):
        self.feeder.rotate(FEEDER_HOLE_OFFSET_ANGLE)
        return TriggerResponse(success = True, message = "Fish feeder fed.")
    
    def empty(self, data: TriggerRequest):
        for _ in range(360 // FEEDER_HOLE_OFFSET_ANGLE):
            self.feeder.rotate(FEEDER_HOLE_OFFSET_ANGLE)
            rospy.sleep(0.1)
        return TriggerResponse(success = True, message = "Fish feeder emptied.")
    
if __name__ == "__main__":
    rospy.init_node('feeder_node')
    feeder_node = FeederNode()
    rospy.spin()
