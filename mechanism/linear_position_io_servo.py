from lib.mechanism.linear_position_io import LinearPositionIO, LinearPositionInputs


class LinearPositionIOServo(LinearPositionIO):
    """
    LinearPositionIO backed by a PWM servo. No position sensor — tracks commanded position only.

    Args:
        servo: PwmServo instance
        min_us: pulse width at 0% duty (default 500 µs)
        max_us: pulse width at 100% duty (default 2500 µs)
        position_at_min_us: mechanism position (m) when servo is at min_us
        position_at_max_us: mechanism position (m) when servo is at max_us
    """

    def __init__(self, servo, min_us=500, max_us=2500,
                 position_at_min_us=0.0, position_at_max_us=0.05):
        self._servo = servo
        self._min_us = min_us
        self._max_us = max_us
        self._m_at_min_us = position_at_min_us
        self._m_at_max_us = position_at_max_us
        # clamping range always [lo, hi] regardless of servo mounting direction
        self._min_m = min(position_at_min_us, position_at_max_us)
        self._max_m = max(position_at_min_us, position_at_max_us)
        self._commanded_m = position_at_min_us
        self._offset_m = 0.0

    # ── LinearPositionIO ──────────────────────────────────────────────────────

    def update_inputs(self, inp):
        inp.connected = True
        inp.position_m = self._commanded_m + self._offset_m
        inp.velocity_m_s = 0.0

    def set_position(self, position_m, feedforward=0.0):
        target = position_m - self._offset_m
        target = max(self._min_m, min(self._max_m, target))
        self._servo.set(self._m_to_us(target))
        self._commanded_m = target

    def set_pid(self, kp, ki, kd):
        pass  # servo has internal controller

    def stop(self):
        pass  # servo holds position

    def home(self, position_m=0.0):
        self._offset_m = position_m - self._commanded_m

    # ── Internal ──────────────────────────────────────────────────────────────

    def _m_to_us(self, position_m):
        t = (position_m - self._m_at_min_us) / (self._m_at_max_us - self._m_at_min_us)
        return int(self._min_us + t * (self._max_us - self._min_us))
