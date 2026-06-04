#!/usr/bin/env python3

import rospy
from abstract_node import AbstractNode
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
import PololuStepper as PS
import threading
import numpy as np

class FeederNode(AbstractNode):
    def __init__(self):
        super().__init__('fish_feeder', 'Fish feeder')

        rospy.Service('fish_feeder/feed', Trigger, self.feed)
        rospy.Service('fish_feeder/empty', Trigger, self.empty)
        self.feed_inc_pub = rospy.Publisher('fish_feeder/feed_inc', TriggerResponse, queue_size=10)

        self.__hole_offset_angle = 360 / rospy.get_param('/FEEDER_HOLE_OFFSET_ANGLE_DIVISOR')

        self.feeder = PS.DRV8834(rospy.get_param('/FEEDER_STEPS_PER_REVOLUTION'),
                                 rospy.get_param('/FEEDER_DIR_PIN'),
                                 rospy.get_param('/FEEDER_STEP_PIN'),
                                 rospy.get_param('/FEEDER_ENABLE_PIN'))

        self.feeder.begin(rospy.get_param('/FEEDER_RPM'), rospy.get_param('/FEEDER_MICROSTEPS'))

    def feed(self, data: TriggerRequest):
        self.feeder.rotate(self.__hole_offset_angle)
        self.feed_inc_pub.publish(TriggerResponse(success = True, message = "Fish feeder fed."))
        return TriggerResponse(success = True, message = "Fish feeder fed.")

    def empty(self, data: TriggerRequest):
        threading.Thread(target=self.__empty).start()
        return TriggerResponse(success = True, message = "Fish feeder emptied.")

    def __empty(self):
        for _ in np.arange(0, 360, self.__hole_offset_angle):
            self.feeder.rotate(self.__hole_offset_angle)
            rospy.sleep(0.1)
    
if __name__ == "__main__":
    rospy.init_node('feeder_node')
    feeder_node = FeederNode()
    rospy.spin()
