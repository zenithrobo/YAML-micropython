from machine import PWM, Pin


class PwmDevice:
    def __init__(self, pin, freq=50, min_us=500, max_us=2500, initial_us=None):
        self.min_us = min_us
        self.max_us = max_us
        self.pulse_us = (min_us + max_us) // 2 if initial_us is None else initial_us
        self._pwm = PWM(Pin(pin), freq=freq)
        self._pwm.duty_ns(self.pulse_us * 1000)

    def set(self, pulse_us):
        pulse_us = max(self.min_us, min(self.max_us, int(pulse_us)))
        self.pulse_us = pulse_us
        self._pwm.duty_ns(pulse_us * 1000)

    def deinit(self):
        self._pwm.deinit()
