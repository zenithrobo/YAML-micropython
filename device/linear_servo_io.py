from lib.mechanism.linear_position_io import LinearPositionIO, LinearPositionInputs


class LinearServoIO(LinearPositionIO):
    """
    LinearPositionIO Real implementation backed by a single PwmServo.
    No position sensor — tracks commanded position only.

    Two construction paths:

    Style A — direct calibration (linkage implicit in the range):
        LinearServoIO(servo,
                      min_us=500,  max_us=2500,
                      min_position_m=0.0, max_position_m=0.05)

    Style B — explicit mechanics (linkage ratio + servo travel):
        LinearServoIO.from_linkage(servo,
                                   meter_per_rad=0.02,
                                   servo_travel_rad=3.14,
                                   zero_position_m=0.0,
                                   min_us=500, max_us=2500)
        # meter_per_rad: linear metres per radian of servo shaft
        #   rack-and-pinion → pitch_radius_m
        #   lever-arm pushrod → arm_length_m  (small-angle approx)
        #   lead-screw → lead_m_per_turn / (2π)
    """

    def __init__(self, servo, min_us, max_us, min_position_m, max_position_m):
        self._servo = servo
        self._min_us = min_us
        self._max_us = max_us
        self._min_m = min_position_m
        self._max_m = max_position_m
        self._commanded_m = min_position_m
        self._offset_m = 0.0

    @classmethod
    def from_linkage(cls, servo, meter_per_rad, servo_travel_rad,
                     zero_position_m=0.0, min_us=500, max_us=2500):
        linear_travel = meter_per_rad * servo_travel_rad
        return cls(servo, min_us, max_us,
                   zero_position_m, zero_position_m + linear_travel)

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

    def stop(self):
        pass  # servo holds current position; no action needed

    def home(self, position_m=0.0):
        self._offset_m = position_m - self._commanded_m

    # ── Internal ──────────────────────────────────────────────────────────────

    def _m_to_us(self, position_m):
        t = (position_m - self._min_m) / (self._max_m - self._min_m)
        return int(self._min_us + t * (self._max_us - self._min_us))
