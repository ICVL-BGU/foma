#!/usr/bin/env python3

import rospy
from abstract_node import AbstractNode
import numpy as np
from ina226 import INA226

class BatteryMonitorNode(AbstractNode):
    def __init__(self):
        super().__init__('battery_monitor', 'Battery monitor')

        self.battery_publisher = rospy.Publisher('/battery_level', rospy.Float32, queue_size=10)

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            battery_level = self.get_battery_level()
            self.battery_publisher.publish(battery_level)
            if battery_level < 20:
                rospy.logwarn("Battery level low! Please recharge soon.")
            rate.sleep()  # Check battery level every 60 seconds

    def get_battery_level(self):
        try:
            ina = INA226(busnum=1, address=0x40)
            ina.configure()
            voltage = ina.voltage()
            # Assuming 12V battery voltage range is 10.5V (0%) to 12.6V (100%) for typical lead-acid
            battery_level = ((voltage - 10.5) / (12.6 - 10.5)) * 100
            battery_level = np.clip(battery_level, 0, 100)
            return battery_level
        except Exception as e:
            rospy.logerr(f"Error reading battery level: {e}")
            return 0.0
    
if __name__ == "__main__":
    rospy.init_node('battery_monitor_node')
    battery_monitor_node = BatteryMonitorNode()
    battery_monitor_node.run()
    rospy.spin()