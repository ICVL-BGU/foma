from gpiozero import LED

class ResetPin:
    def __init__(self, pin_number, set_warning=False, initial_value=False, active_value=False):
        """
        Initializes a GPIO pin using gpiozero LED abstraction.

        :param pin_number: The BCM pin number to control.
        :param set_warning: Set to True to enable GPIO warnings (not applicable in gpiozero).
        :param initial_value: Initial state of the pin (False for LOW, True for HIGH).
        :param active_value: Active state of the pin (True for HIGH, False for LOW).
        """
        self._pin_number = pin_number
        self._active_value = active_value
        self.led = LED(pin_number)

        # Set initial state
        if initial_value:
            self.led.on()
        else:
            self.led.off()

    def turnOff(self):
        """Turns the pin off."""
        if self._active_value:
            self.led.off()
        else:
            self.led.on()
        self._active_value = not self._active_value

    def turnOn(self):
        """Turns the pin on."""
        if self._active_value:
            self.led.on()
        else:
            self.led.off()
        self._active_value = not self._active_value
