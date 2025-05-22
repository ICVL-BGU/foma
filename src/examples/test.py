from gpiozero import OutputDevice
from time import sleep

# Define pins using BCM numbering
dir_pin = 18
step_pin = 27
enable_pin = 4  # Enable pin might not be connected
m0_pin = 17
m1_pin = 22

# Initialize pins
direction = OutputDevice(dir_pin)
step = OutputDevice(step_pin)
enable = OutputDevice(enable_pin, initial_value=False)

# Example function to step the motor
def step_motor(steps, delay=0.01):
    enable.on()  # Enable the motor driver
    direction.on()  # Set direction (change to .off() for opposite direction)
    
    for _ in range(steps):
        step.on()
        sleep(delay)
        step.off()
        sleep(delay)
    
    enable.off()  # Disable the motor driver

# Test with 200 steps
step_motor(200)
