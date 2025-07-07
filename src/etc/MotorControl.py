from time import sleep
from serial import Serial
from etc.PololuQik import *
from etc.ResetPin import *
import numpy as np
from gpiozero import RotaryEncoder


DEFAULT_UART_PORT = "/dev/ttyS0"
DEFAULT_MOTOR_SPEED = 127

"""
TopMotor - M0 on motorTopBottom
BottomMotor - M1 on motorTopBottom
RightMotor - M0 on motorRightLeft
LeftMotor - M1 on motorRightLeft

We'll set the default driving directin to top left
"""

class MotorControl(Serial):
    def __init__(self, resetPins: tuple, encoderChannels:tuple, port = DEFAULT_UART_PORT, speed = DEFAULT_MOTOR_SPEED, accl = 0, brake = 0, baudrate = 115200, timeout = 0.020):
        super().__init__(port = port, baudrate = baudrate, timeout = timeout)
        if not self.is_open:
            self.open()
        #init motor controllers instances
        self.motorTopBottom = PololuQik2s15v9(serial = self, resetPin = ResetPin(resetPins[0]), addr = 0x0C , multi_device = True)
        self.motorRightLeft = PololuQik2s15v9(serial = self, resetPin = ResetPin(resetPins[1]), addr = 0x0A , multi_device = True) # 0x0A

        self.encoderTop = RotaryEncoder(encoderChannels[0][0], encoderChannels[0][1])
        self.encoderBottom = RotaryEncoder(encoderChannels[1][0], encoderChannels[1][1])
        self.encoderLeft = RotaryEncoder(encoderChannels[2][0], encoderChannels[2][1])
        self.encoderRight = RotaryEncoder(encoderChannels[3][0], encoderChannels[3][1])
        
        self.speed = speed

        #set controllers to different IDs to avoid collision due to daisy-chain connection
        self.motorTopBottom.turnOn()
        self.motorRightLeft.turnOn()

        self.motorTopBottom.setAccelerations(accl, accl)
        self.motorRightLeft.setAccelerations(accl, accl)
        self.motorTopBottom.setBrakes(brake, brake)
        self.motorRightLeft.setBrakes(brake, brake)

    def shutdown(self):
        self.motorTopBottom.turnOff()
        self.motorRightLeft.turnOff()
        self.close()

    def move_by_angle(self, angle: int, speed = None):
        if not speed:
            speed = self.speed

        angle = angle % 360

        hSpeed = speed * np.sin(angle * np.pi/180)
        vSpeed = speed * np.cos(angle * np.pi/180)

        self.motorRightLeft.setSpeeds(-hSpeed, hSpeed)
        self.motorTopBottom.setSpeeds(vSpeed, -vSpeed)
        
    # def move_by_components(self, horizontal_component: float, vertical_component: float, speed = None):
    #     if not speed:
    #         speed = self.speed
        
    #     hSpeed = int(speed * horizontal_component)
    #     vSpeed = int(speed * vertical_component)
    #     self.motorRightLeft.setSpeeds(-hSpeed, hSpeed)
    #     self.motorTopBottom.setSpeeds(vSpeed, -vSpeed)
    def move_by_components(
        self,
        horizontal_component: float,
        vertical_component:   float,
        speed: float = None
    ):
        """
        Simple error‐based slowdown:
        - compute base_top/bottom/left/right
        - if top+bottom>0, top is faster → top = 90% of base_top
          if top+bottom<0, bottom is faster → bottom = 90% of base_bottom
        - same for left/right
        - uses round() instead of int() for speed calculations
        """
        if speed is None:
            speed = self.speed

        # 1) base speeds (rounded)
        top_speed    = round( vertical_component * speed)
        bottom_speed = round(-vertical_component * speed)
        left_speed   = round( horizontal_component * speed)
        right_speed  = round(-horizontal_component * speed)

        # 2) read encoder counts
        tb_error  = self.encoderTop.value + self.encoderBottom.value
        lr_error  = self.encoderLeft.value + self.encoderRight.value

        # 3) slow down *only* the faster wheel in each pair to 90%
        if   tb_error >  0:  # top is running faster
            top_speed    = round(top_speed    * 0.9)
        elif tb_error <  0:  # bottom is running faster
            bottom_speed = round(bottom_speed * 0.9)

        if   lr_error >  0:  # left is running faster
            left_speed   = round(left_speed   * 0.9)
        elif lr_error <  0:  # right is running faster
            right_speed  = round(right_speed  * 0.9)

        # 4) send to the two PololuQik controllers
        self.motorRightLeft.setSpeeds(left_speed,  right_speed)
        self.motorTopBottom.setSpeeds(top_speed,   bottom_speed)

    def move_by_wheel(self, wheel: str, speed = None):
        if not speed:
            speed = self.speed

        if wheel == 'top':
            self.motorTopBottom.setSpeeds(speed,0)
        elif wheel == 'bottom':
            self.motorTopBottom.setSpeeds(0,speed)
        elif wheel == 'left':
            self.motorRightLeft.setSpeeds(0,speed)
        elif wheel == 'right':
            self.motorRightLeft.setSpeeds(speed,0)
        else:
            raise ValueError("Invalid wheel, received: {}, allowed wheels are: ('top','bottom','left','right'), case sensitive.".format(wheel))
        
    def rotate(self, direction: float):
        speed = int(self.speed * direction / max(1, abs(direction)))
        self.motorRightLeft.setSpeeds(speed, speed)
        self.motorTopBottom.setSpeeds(speed, speed)
