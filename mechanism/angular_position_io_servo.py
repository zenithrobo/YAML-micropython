import math

from lib.mechanism.angular_position_io import AngularPositionIO, AngularPositionInputs


class AngularPositionIOServo(AngularPositionIO):
    """
    AngularPositionIO — Real implementation backed by a single PwmServo.
    No position sensor: tracks commanded angle only.

    Style A — direct calibration (gear ratio implicit in the range):
        AngularPositionIOServo(servo,
                               min_us=500, max_us=2500,
                               min_angle_rad=0.0, max_angle_rad=math.pi)

    Style B — explicit mechanics:
        AngularPositionIOServo.from_reduction(servo,
                                              reduction=3.0,
                                              servo_travel_rad=math.pi,
                                              zero_angle_rad=0.0,
                                              min_us=500, max_us=2500)
        # reduction = servo_rad / mechanism_rad
        # mechanism_travel = servo_travel_rad / reduction
    """

    def __init__(self, servo, min_us, max_us, min_angle_rad, max_angle_rad):
        self._servo = servo
        self._min_us = min_us
        self._max_us = max_us
        self._min_rad = min_angle_rad
        self._max_rad = max_angle_rad
        self._commanded_rad = min_angle_rad
        self._offset_rad = 0.0

    @classmethod
    def from_reduction(cls, servo, reduction, servo_travel_rad,
                       zero_angle_rad=0.0, min_us=500, max_us=2500):
        mechanism_travel = servo_travel_rad / reduction
        return cls(servo, min_us, max_us,
                   zero_angle_rad, zero_angle_rad + mechanism_travel)

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

    def stop(self):
        pass  # servo holds position; no action needed

    def home(self, angle_rad=0.0):
        self._offset_rad = angle_rad - self._commanded_rad

    # ── Internal ──────────────────────────────────────────────────────────────

    def _rad_to_us(self, angle_rad):
        t = (angle_rad - self._min_rad) / (self._max_rad - self._min_rad)
        return int(self._min_us + t * (self._max_us - self._min_us))
