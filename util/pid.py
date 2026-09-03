import time


class PIDController:
    """
    Discrete PID controller with auto-computed dt from ticks_us().

    Integral windup and output clamping are opt-in:
        pid.set_integral_range(max_abs)   # symmetric windup limit
        pid.set_output_range(min, max)    # output clamp

    Call reset() whenever the setpoint changes discontinuously or the
    mechanism is re-enabled, to avoid derivative kick and stale integral.
    """

    def __init__(self, kp=0.0, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral = 0.0
        self._last_error = None
        self._last_t_us = None
        self._integral_limit = None
        self._out_min = None
        self._out_max = None

    # ── Configuration ─────────────────────────────────────────────────────────

    def set_gains(self, kp, ki, kd):
        if (self.kp, self.ki, self.kd) != (kp, ki, kd):
            self.reset()
        self.kp, self.ki, self.kd = kp, ki, kd

    def set_integral_range(self, max_abs):
        """Clamp integral accumulator to ±max_abs."""
        self._integral_limit = abs(max_abs)

    def set_output_range(self, out_min, out_max):
        self._out_min, self._out_max = out_min, out_max

    # ── Runtime ───────────────────────────────────────────────────────────────

    def calculate(self, measurement, setpoint):
        """Return control output. Call once per loop."""
        now = time.ticks_us()
        error = setpoint - measurement

        if self._last_t_us is None:
            self._last_error = error
            self._last_t_us = now
            return self._clamp(self.kp * error)

        dt = time.ticks_diff(now, self._last_t_us) / 1_000_000.0
        self._last_t_us = now

        if dt > 0:
            self._integral += error * dt
            if self._integral_limit is not None:
                lim = self._integral_limit
                self._integral = max(-lim, min(lim, self._integral))
            derivative = (error - self._last_error) / dt
        else:
            derivative = 0.0

        self._last_error = error
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return self._clamp(output)

    def reset(self):
        self._integral = 0.0
        self._last_error = None
        self._last_t_us = None

    def at_setpoint(self, measurement, setpoint, tolerance):
        return abs(setpoint - measurement) <= tolerance

    # ── Internal ──────────────────────────────────────────────────────────────

    def _clamp(self, value):
        if self._out_min is not None:
            value = max(self._out_min, min(self._out_max, value))
        return value
