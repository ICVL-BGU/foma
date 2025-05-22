import RPi.GPIO as GPIO
from gpiozero import PWMOutputDevice, BadPinFactory
import numpy as np


# GPIO.setmode(GPIO.BCM)  # use BCM pin numbering
# GPIO.setwarnings(False)
# GPIO.setup(17, GPIO.OUT)
# out = PWMOutputDevice(pin = 17, initial_value = 0.5, frequency = 700)
# out.pulse(fade_in_time = 5, fade_out_time = 5, n = None)

import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
freq, low, high = 100, 800, 2200

p = GPIO.PWM(17, freq)  # channel=12 frequency=50Hz
low_per = low * freq / 10000
high_per = high * freq / 10000
step = (high_per - low_per)/70

print(low_per, high_per)
# val = None
# while val!=0:
#     val = float(input("Enter value:"))
#     p.ChangeDutyCycle(val)
#     # try:
#     #     while 1:
#     #         for dc in range(0, 10):
#     #             print(dc)
#     #             p.ChangeDutyCycle(dc)
#     #             time.sleep(1)
#     #         for dc in range(10, -1, -1):
#     #             print(dc)
#     #             p.ChangeDutyCycle(dc)
#     #             time.sleep(1)
#     # except KeyboardInterrupt:
#     #     pass
#     time.sleep(0.5)
while True:
    input("Feed:")
    p.start(low_per)
    time.sleep(0.1)
    for dc in np.arange(low_per, high_per, step):
        print(dc)
        p.ChangeDutyCycle(dc)
        time.sleep(0.03)
    for dc in np.arange(high_per, low_per, -step):
        print(dc)
        p.ChangeDutyCycle(dc)
        time.sleep(0.03)
    # p.stop()
p.stop()
GPIO.cleanup()