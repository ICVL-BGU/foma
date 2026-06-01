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

        self.encoderTop = RotaryEncoder(encoderChannels[0][0], encoderChannels[0][1], max_steps = 0)
        self.encoderBottom = RotaryEncoder(encoderChannels[1][0], encoderChannels[1][1], max_steps = 0)
        self.encoderLeft = RotaryEncoder(encoderChannels[2][0], encoderChannels[2][1], max_steps = 0)
        self.encoderRight = RotaryEncoder(encoderChannels[3][0], encoderChannels[3][1], max_steps = 0)
        
        self.speed = speed
        self.punishment_factor = 0.85
        # proportional encoder-balance gain (stopgap heading hold)
        self.balance_kp = 0.08    # tune: raise till tight, back off ~30%
        self.balance_clip = 0.30  # max fractional correction per pair
        self.balance_deadband = 1  # ignore rate err <= this (encoder noise)
        self.balance_sign_v = 1   # flip to -1 if vertical linear curves wrong way
        self.balance_sign_h = 1   # flip to -1 if horizontal linear curves wrong way
        # prev cumulative counts -> per-cycle rate (avoid saturating on totals)
        self._prev_top = self._prev_bottom = 0
        self._prev_left = self._prev_right = 0

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
        
    def move_by_components(
        self,
        horizontal_component: float,
        vertical_component:   float,
        speed: float = None
    ):
        """
        Proportional encoder-balance heading hold (stopgap).

        Each axis driven by a wheel pair spinning equal+opposite. Unequal
        encoder counts => chassis yaw. Correction proportional to count
        mismatch, slow leader + speed laggard symmetrically, clipped.

        Replaces fixed-factor bang-bang `punishment_factor` (oscillated,
        cumulative-lag, int() truncation bias).

        Note: encoders measure wheel spin, not chassis yaw. Slip still
        drifts. Stopgap slows drift, not eliminate. Real fix = lidar/IMU.
        """
        if speed is None:
            speed = self.speed

        top_speed    = vertical_component * speed
        bottom_speed = -vertical_component * speed
        left_speed   = horizontal_component * speed
        right_speed  = -horizontal_component * speed

        # per-cycle rate = delta since last call (bounded, no saturation)
        t, b = self.encoderTop.steps, self.encoderBottom.steps
        l, r = self.encoderLeft.steps, self.encoderRight.steps
        d_top    = abs(t - self._prev_top)
        d_bottom = abs(b - self._prev_bottom)
        d_left   = abs(l - self._prev_left)
        d_right  = abs(r - self._prev_right)
        self._prev_top, self._prev_bottom = t, b
        self._prev_left, self._prev_right = l, r

        db = self.balance_deadband

        # boost laggard only, never cut leader => heavy axis won't stall
        # vertical axis = Top/Bottom pair
        if abs(vertical_component) > 1e-6:
            err = d_top - d_bottom  # +ve => top spinning faster this cycle
            if abs(err) <= db:
                err = 0
            corr = np.clip(self.balance_sign_v * self.balance_kp * err,
                           -self.balance_clip, self.balance_clip)
            if corr > 0:    bottom_speed *= (1 + corr)  # top leads -> boost bottom
            elif corr < 0:  top_speed    *= (1 - corr)  # bottom leads -> boost top

        # horizontal axis = Right/Left pair
        if abs(horizontal_component) > 1e-6:
            err = d_right - d_left  # +ve => right spinning faster this cycle
            if abs(err) <= db:
                err = 0
            corr = np.clip(self.balance_sign_h * self.balance_kp * err,
                           -self.balance_clip, self.balance_clip)
            if corr > 0:    left_speed  *= (1 + corr)  # right leads -> boost left
            elif corr < 0:  right_speed *= (1 - corr)  # left leads -> boost right

        # clamp to driver range (boost may overshoot), round (no trunc bias)
        lim = abs(speed)
        top_speed    = int(round(np.clip(top_speed,    -lim, lim)))
        bottom_speed = int(round(np.clip(bottom_speed, -lim, lim)))
        left_speed   = int(round(np.clip(left_speed,   -lim, lim)))
        right_speed  = int(round(np.clip(right_speed,  -lim, lim)))

        self.motorRightLeft.setSpeeds(right_speed,  left_speed)
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

    def reset_encoders(self):
        self.encoderTop.steps = 0
        self.encoderBottom.steps = 0
        self.encoderLeft.steps = 0
        self.encoderRight.steps = 0
        self._prev_top = self._prev_bottom = 0
        self._prev_left = self._prev_right = 0