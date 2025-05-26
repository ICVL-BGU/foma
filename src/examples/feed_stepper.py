import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'etc'))

from PololuStepper.DRV8834 import *
from gpiozero import Button

# Initialize the stepper motor driver
steps_per_revolution = 200  # Number of steps per revolution for your stepper motor
dir_pin = 18
step_pin = 27
enable_pin = 4  # Assuming you don't have an enable pin connected
# m0_pin = 17
# m1_pin = 22

# Create an instance of the DRV8834 class
# feeder = DRV8834(steps_per_revolution, dir_pin, step_pin, enable_pin, m0_pin, m1_pin)
feeder = DRV8834(steps_per_revolution, dir_pin, step_pin, enable_pin)

# Set the number of microsteps
microsteps = 4  # You can adjust this as needed

# Initialize the driver with the current RPM and microsteps
rpm = 30  # Rotations per minute
feeder.begin(rpm, microsteps)
# feeder.enable()

# Function to move the motor one step (18 degrees)
def move_to_next_hole():
    feeder.rotate(45)
    print("Moved to next hole")

# Main loop to move continuously with a 0.5-second delay between moves
print("Running continuously. Press Ctrl+C to exit.")
try:
    # while input() != 'q':
    #     # feeder.enable()
    #     feeder.move(1)
    #     # feeder.rotate(18)
    #     print("Moved 1 step")
    #     # feeder.disable()

    while True:
        input()
        # feeder.enable()
        # feeder.move(10)
        move_to_next_hole()
        # feeder.disable()
        # sleep(0.5)  # Wait for 0.5 seconds before the next move
except KeyboardInterrupt:
    print("Exiting...")
finally:
    print("Disabling")
    feeder.disable()
    # No GPIO.cleanup() needed as gpiozero handles cleanup automatically
