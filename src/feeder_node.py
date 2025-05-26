#!/usr/bin/env python3

import rospy
from abstract_node import AbstractNode
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from gpiozero import PWMOutputDevice, BadPinFactory
import PololuStepper as PS

class FeederNode(AbstractNode):
    def __init__(self):
        super().__init__('fish_feeder', 'Fish feeder')
        self.fish_feed_sub = rospy.Service('fish_feeder/feed', Trigger, self.feed)
        
        steps_per_revolution = 200  # Number of steps per revolution for your stepper motor
        dir_pin = 18
        step_pin = 27
        enable_pin = 4 
        self.feeder = PS.DRV8834(steps_per_revolution, dir_pin, step_pin, enable_pin)
        
        microsteps = 4  # You can adjust this as needed
        
        # Initialize the driver with the current RPM and microsteps
        rpm = 30  # Rotations per minute
        self.feeder.begin(rpm, microsteps)
        try:
            self.out = PWMOutputDevice(pin = 17, initial_value = 0.5, frequency = 700)
        except BadPinFactory as e:
            self.logwarn(e.msg)

    def feed(self, data: TriggerRequest):
        self.feeder.rotate(12)
        return TriggerResponse(success = True, message = "Fish feeder fed.")
    
if __name__ == "__main__":
    rospy.init_node('feeder_node')
    feeder_node = FeederNode()
    rospy.spin()
