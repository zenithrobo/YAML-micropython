import math

from lib.mechanism.angular_velocity_io import AngularVelocityIO, AngularVelocityInputs
from lib.util.pid import PIDController

_TWO_PI = 2.0 * math.pi


class AngularVelocityIOBrushless(AngularVelocityIO):
    """
    AngularVelocityIO backed by a brushless ESC with closed-loop velocity feedback.

    Args:
        esc:          object with set(us: int) and stop() — e.g. PwmESC
        sensor:       AngularVelocitySensor instance
        spin_up_us:   minimum pulse width (µs) at which the motor actually spins
        safe_max_us:  maximum stable pulse width (µs) from calibration
        peak_rad_s:   angular velocity (rad/s) at safe_max_us — used for feedforward

    Feedforward is computed internally as target/peak (open-loop estimate).
    PID corrects residual error on top. Call set_pid() to enable closed-loop.
    update_inputs() must be called each loop before set_velocity().
    """

    def __init__(self, esc, sensor, spin_up_us, safe_max_us, peak_rad_s):
        self._esc = esc
        self._sensor = sensor
        self._spin_up_us = spin_up_us
        self._safe_max_us = safe_max_us
        self._peak_rad_s = peak_rad_s
        self._pid = PIDController()
        self._pid.set_output_range(-1.0, 1.0)
        self._pid.set_integral_range(200)   # anti-windup: ki * 200 rad ≈ 4% throttle
        self._last_velocity_rad_s = 0.0

    # ── AngularVelocityIO ─────────────────────────────────────────────────────

    def update_inputs(self, inp):
        v = self._sensor.get_velocity_rad_s()
        inp.connected = v is not None
        if v is not None:
            self._last_velocity_rad_s = v
            inp.velocity_rad_s = v

    def set_velocity(self, velocity_rad_s, feedforward=0.0):
        ff = velocity_rad_s / self._peak_rad_s          # open-loop estimate [0, 1]
        pid = self._pid.calculate(self._last_velocity_rad_s, velocity_rad_s)
        throttle = max(0.0, min(1.0, ff + pid + feedforward))
        us = int(self._spin_up_us + throttle * (self._safe_max_us - self._spin_up_us))
        self._esc.set(us)

    def set_pid(self, kp, ki, kd):
        self._pid.set_gains(kp, ki, kd)

    def poll(self):
        """Call at high frequency (>600 Hz at 9000 RPM) to update velocity estimate."""
        self._sensor.poll()

    def stop(self):
        self._pid.reset()
        self._esc.set(self._spin_up_us)   # back to arm/idle position
