#!/usr/bin/env python3

import rospy
from abstract_node import AbstractNode
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
import RPi.GPIO as GPIO
from gpiozero import PWMOutputDevice, BadPinFactory

class FeederNode(AbstractNode):
    def __init__(self):
        super().__init__('fish_feeder', 'Fish feeder')
        self.fish_feed_sub = rospy.Service('fish_feeder/feed', Trigger, self.feed)
        # GPIO setup
        # GPIO.setmode(GPIO.BCM)  # use BCM pin numbering
        # GPIO.setwarnings(False)
        # GPIO.setup(17, GPIO.OUT, initial=GPIO.LOW)  # Set as output and initialize as LOW
        
        steps_per_revolution = 200  # Number of steps per revolution for your stepper motor
        dir_pin = 18
        step_pin = 27
        enable_pin = 4 
        self.feeder = DRV8834(steps_per_revolution, dir_pin, step_pin, enable_pin)
        
        microsteps = 4  # You can adjust this as needed
        
        # Initialize the driver with the current RPM and microsteps
        rpm = 30  # Rotations per minute
        feeder.begin(rpm, microsteps)
        try:
            self.out = PWMOutputDevice(pin = 17, initial_value = 0.5, frequency = 700)
        except BadPinFactory as e:
            rospy.logwarn(e.msg)

    def feed(self, data: TriggerRequest):
        self.feeder.rotate(12)
        # self.out.pulse(fade_in_time = 5, fade_out_time = 5, n = 1)
        return TriggerResponse(success = True, message = "Fish feeder fed.")
    
if __name__ == "__main__":
    rospy.init_node('feeder_node')
    ceiling_cam_handler = FeederNode()
    rospy.spin()
