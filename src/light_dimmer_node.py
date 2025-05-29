#!/usr/bin/env python3

import rospy
import serial
from abstract_node import AbstractNode
from foma.srv import Light, LightRequest, LightResponse

class LightDimmerNode(AbstractNode):
    def __init__(self):
        super().__init__('light_dimmer', 'Light dimmer')
        self.dimmer_service = rospy.Service('light_dimmer/change', Light, self.dim)

        # Use the alias for the serial port
        self.serial_port_alias = '/dev/light_dimmer'  # Replace this with your configured alias
        self.serial_baud_rate = 9600
        self.serial_timeout = 1

        # Initialize serial port as None
        self.serial_port = None
        self.open_serial_port()
        rospy.on_shutdown(self.__on_shutdown)

    def open_serial_port(self):
        """Attempt to open the serial port."""
        try:
            self.serial_port = serial.Serial(
                self.serial_port_alias,
                self.serial_baud_rate,
                timeout=self.serial_timeout
            )
            self.loginfo(f"Serial port {self.serial_port_alias} opened at {self.serial_baud_rate} baud.")
        except serial.SerialException as e:
            self.logerr(f"Failed to open serial port {self.serial_port_alias}: {e}")
            self.serial_port = None

    def dim(self, data: LightRequest):
        """Handle the light dimming request."""
        # Attempt to open the serial port if not open
        if not self.serial_port or not self.serial_port.is_open:
            self.logwarn("Serial port is not open. Attempting to reopen...")
            self.open_serial_port()

        # If still not open, return an error response
        if not self.serial_port or not self.serial_port.is_open:
            return LightResponse(result=False)#, message="Serial port is not available.")

        try:
            # Ensure data is within the valid range (0-255)
            value = max(0, min(255, int(data.data)))

            # Send the value via serial
            self.serial_port.write(f"{value}".encode('utf-8'))
            self.loginfo(f"Sent value {value} to the light dimmer via serial.")
            return LightResponse(result=True)#, message=f"Light dimmer dimmed to {value}.")
        except Exception as e:
            rospy.logerr(f"Failed to send data via serial: {e}")
            return LightResponse(result=False)#, message="Failed to dim light.")
        
    def __on_shutdown(self):
        self.loginfo(f"Turning off the light dimmer.")
        if self.serial_port and self.serial_port.is_open:
            # Send a command to turn off the light dimmer
            self.serial_port.write(f"{0}".encode('utf-8'))
            self.serial_port.close()
        else:
            self.logwarn("Serial port was not open during shutdown.")

if __name__ == "__main__":
    rospy.init_node('light_dimmer_node')
    light_dimmer = LightDimmerNode()
    rospy.spin()