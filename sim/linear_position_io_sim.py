import math
import time

from lib.mechanism.linear_position_io import LinearPositionIO, LinearPositionInputs


class LinearPositionIOSim(LinearPositionIO):
    """
    Linear mechanism simulation (elevator, slide, angled lift…).

    Physics: m·a = F_motor − m·g·sin(angle_rad)
    Motor model: F_motor = kF · clamp(voltage, ±12)

    angle_rad: inclination from horizontal (0 = horizontal, π/2 = vertical).
    Gravity component = m·g·sin(angle_rad), so a horizontal slide has
    zero gravity load and a vertical elevator has full gravity load.

    set_position() teleports — matches FRC GenericElevatorIOSim behaviour.
    set_voltage() drives the physics for open-loop / voltage-control tests.
    """

    _G = 9.81

    def __init__(
        self,
        mass_kg,
        min_position_m,
        max_position_m,
        start_position_m=0.0,
        angle_rad=math.pi / 2,
        kf=1.0,
    ):
        self._mass = mass_kg
        self._min = min_position_m
        self._max = max_position_m
        self._pos = start_position_m
        self._vel = 0.0
        self._voltage = 0.0
        self._sin_angle = math.sin(angle_rad)
        self._kf = kf
        self._last_ms = None

    # ── LinearPositionIO interface ────────────────────────────────────────────

    def update_inputs(self, inp):
        now = time.ticks_ms()
        if self._last_ms is None:
            self._last_ms = now
            inp.connected = True
            inp.position_m = self._pos
            inp.velocity_m_s = 0.0
            return
        dt = time.ticks_diff(now, self._last_ms) * 0.001
        self._last_ms = now
        self._step(dt)
        inp.connected = True
        inp.position_m = self._pos
        inp.velocity_m_s = self._vel

    def set_position(self, position_m, feedforward=0.0):
        self._pos = max(self._min, min(self._max, position_m))
        self._vel = 0.0

    def stop(self):
        self.set_voltage(0.0)

    def home(self, position_m=0.0):
        self._pos = position_m
        self._vel = 0.0

    # ── Extra: voltage-drive for open-loop tests ──────────────────────────────

    def set_voltage(self, voltage_v):
        self._voltage = max(-12.0, min(12.0, float(voltage_v)))

    # ── Physics ───────────────────────────────────────────────────────────────

    def _step(self, dt):
        f_gravity = self._mass * self._G * self._sin_angle
        f_motor = self._kf * self._voltage
        accel = (f_motor - f_gravity) / self._mass
        self._vel += accel * dt
        self._pos += self._vel * dt
        if self._pos < self._min:
            self._pos = self._min
            self._vel = max(0.0, self._vel)
        elif self._pos > self._max:
            self._pos = self._max
            self._vel = min(0.0, self._vel)
