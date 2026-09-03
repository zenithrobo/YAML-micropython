import math

from lib.mechanism.angular_position_io import AngularPositionIO, AngularPositionInputs


class AngularPositionIOServo(AngularPositionIO):
    """
    AngularPositionIO backed by a PWM servo. No position sensor — tracks commanded angle only.

    Args:
        servo: PwmServo instance
        min_us: pulse width at 0% duty (default 500 µs)
        max_us: pulse width at 100% duty (default 2500 µs)
        angle_at_min_us: mechanism angle (rad) when servo is at min_us
        angle_at_max_us: mechanism angle (rad) when servo is at max_us
    """

    def __init__(self, servo, min_us=500, max_us=2500,
                 angle_at_min_us=0.0, angle_at_max_us=math.pi):
        self._servo = servo
        self._min_us = min_us
        self._max_us = max_us
        self._rad_at_min_us = angle_at_min_us
        self._rad_at_max_us = angle_at_max_us
        # clamping range always [lo, hi] regardless of servo mounting direction
        self._min_rad = min(angle_at_min_us, angle_at_max_us)
        self._max_rad = max(angle_at_min_us, angle_at_max_us)
        self._commanded_rad = angle_at_min_us
        self._offset_rad = 0.0

    # ── AngularPositionIO ─────────────────────────────────────────────────────

    def update_inputs(self, inp):
        inp.connected = True
        inp.position_rad = self._commanded_rad + self._offset_rad
        inp.velocity_rad_s = 0.0

    def set_position(self, position_rad, feedforward=0.0):
        target = position_rad - self._offset_rad
        target = max(self._min_rad, min(self._max_rad, target))
        self._servo.set(self._rad_to_us(target))
        self._commanded_rad = target

    def set_pid(self, kp, ki, kd):
        pass  # servo has internal controller

    def stop(self):
        pass  # servo holds position

    def home(self, angle_rad=0.0):
        self._offset_rad = angle_rad - self._commanded_rad

    # ── Internal ──────────────────────────────────────────────────────────────

    def _rad_to_us(self, angle_rad):
        t = (angle_rad - self._rad_at_min_us) / (self._rad_at_max_us - self._rad_at_min_us)
        return int(self._min_us + t * (self._max_us - self._min_us))
