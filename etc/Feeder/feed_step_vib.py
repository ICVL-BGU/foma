import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'etc'))

from PololuStepper.DRV8834 import *
import time

# Initialize the stepper motor driver
steps_per_revolution = 200  # Number of steps per revolution for your stepper motor
dir_pin = 18
step_pin = 27
enable_pin = 4  # Assuming you don't have an enable pin connected

# vib_pin = 17
# vib_motor = PWMOutputDevice(vib_pin)
# Create an instance of the DRV8834 class
feeder = DRV8834(steps_per_revolution, dir_pin, step_pin, enable_pin)

# Set the number of microsteps
microsteps = 4  # You can adjust this as needed

# Initialize the driver with the current RPM and microsteps
rpm = 30  # Rotations per minute
feeder.begin(rpm, microsteps)
# feeder.enable()

# Function to move the motor one step (18 degrees)
def move_to_next_hole():
    feeder.rotate(12)
    # feeder.rotate(55)
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
        # input()
        # feeder.enable()
        # feeder.move(10)
        # vib_motor.value = 1
        move_to_next_hole()
        time.sleep(1)
        # time.sleep(0.5)
        # vib_motor.value = 0
        # feeder.disable()
        # sleep(0.5)  # Wait for 0.5 seconds before the next move
except KeyboardInterrupt:
    print("Exiting...")
finally:
    print("Disabling")
    feeder.disable()
    # No GPIO.cleanup() needed as gpiozero handles cleanup automatically
