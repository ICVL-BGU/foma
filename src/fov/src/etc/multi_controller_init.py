# This code should only be executed once, since pololu controllers have non-volatile memory.

from serial import Serial
from time import sleep
from PololuQik import *
from ResetPin import *

# Settings
# Change according to your settings
SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200 
TIMEOUT = 0.02

# Could add more controllers if needed, should add their reset pins and new addresses.
# Addresses should be increased by 2, reset pins are by bcm order
CONTROLER_1_RESET_PIN = 23
CONTROLER_2_RESET_PIN = 24

CONTROLER_2_NEW_ADDRESS = 0x0C

ser = Serial(port = SERIAL_PORT, baudrate = BAUD_RATE, timeout = TIMEOUT)
cntrl1 = PololuQik2s15v9(serial = ser, resetPin = ResetPin(CONTROLER_1_RESET_PIN), multi_device = True)
cntrl2 = PololuQik2s15v9(serial = ser, resetPin = ResetPin(CONTROLER_2_RESET_PIN), multi_device = True)

# Controller instances created turned off, but as counter-measure
cntrl1.turnOff()
cntrl2.turnOff()

cntrl2.turnOn()
sleep(0.005)

cntrl2.setAddress(CONTROLER_2_NEW_ADDRESS)

ret_val = ser.read()

if ret_val == 0:
    print("Address set successfully.")
elif ret_val == 1:
    print("Bad parameter.") # Shouldn't happen unless signal corrupted.
elif ret_val == 2:
    print("Bad value.") # Bad new address, or if the parameter was corrupted, but to an allowed parameter, wrong value for said parameter.
