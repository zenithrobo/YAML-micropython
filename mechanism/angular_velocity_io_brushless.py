from lib.mechanism.angular_velocity_io import AngularVelocityIO, AngularVelocityInputs
from lib.util.pid import PIDController


class AngularVelocityIOBrushless(AngularVelocityIO):
    """
    AngularVelocityIO backed by a brushless ESC with closed-loop velocity feedback.

    Args:
        esc: object with set_throttle(float) — 0.0 (stopped) to 1.0 (full speed)
        sensor: AngularVelocitySensor instance (e.g. ESC telemetry, tachometer)

    Call set_pid() before use; default gains are zero (open-loop feedforward only).
    update_inputs() must be called each loop before set_velocity() so the PID
    uses the freshest sensor reading.
    """

    def __init__(self, esc, sensor):
        self._esc = esc
        self._sensor = sensor
        self._pid = PIDController()
        self._pid.set_output_range(0.0, 1.0)
        self._last_velocity_rad_s = 0.0

    # ── AngularVelocityIO ─────────────────────────────────────────────────────

    def update_inputs(self, inp):
        v = self._sensor.get_velocity_rad_s()
        inp.connected = v is not None
        if v is not None:
            self._last_velocity_rad_s = v
            inp.velocity_rad_s = v

    def set_velocity(self, velocity_rad_s, feedforward=0.0):
        output = self._pid.calculate(self._last_velocity_rad_s, velocity_rad_s) + feedforward
        output = max(0.0, min(1.0, output))
        self._esc.set_throttle(output)

    def set_pid(self, kp, ki, kd):
        self._pid.set_gains(kp, ki, kd)

    def stop(self):
        self._pid.reset()
        self._esc.set_throttle(0.0)
