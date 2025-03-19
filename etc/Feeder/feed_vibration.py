import time
from gpiozero import PWMOutputDevice

# Initialize the stepper motor driver
# steps_per_revolution = 200  # Number of steps per revolution for your stepper motor
# dir_pin = 18
# step_pin = 27
# enable_pin = 4  # Assuming you don't have an enable pin connected
m0_pin = 17
# m1_pin = 22
motor = PWMOutputDevice(m0_pin)

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
        val = float(input())
        # feeder.enable()
        # feeder.move(10)
        motor.value=val
        # move_to_next_hole()
        # feeder.disable()
        # sleep(0.5)  # Wait for 0.5 seconds before the next move
except KeyboardInterrupt:
    print("Exiting...")
finally:
    print("Disabling")
    # feeder.disable()
    # No GPIO.cleanup() needed as gpiozero handles cleanup automatically
