import math
import time

from lib.mechanism.angular_position_io import AngularPositionIO, AngularPositionInputs


class AngularPositionIOSim(AngularPositionIO):
    """
    Single-jointed arm simulation.

    Physics: I·α = τ_motor − m·g·l·cos(θ)
    Motor model: τ_motor = kT · clamp(voltage, ±12)

    set_position() teleports (matches FRC GenericArmIOSim behaviour —
    the profile/PID path is the app's concern, not the sim's).
    set_voltage() drives the physics for open-loop / voltage-control tests.
    """

    _G = 9.81

    def __init__(
        self,
        moi_kg_m2,
        arm_length_m,
        mass_kg,
        min_angle_rad,
        max_angle_rad,
        start_angle_rad=0.0,
        kt=1.0,
    ):
        self._moi = moi_kg_m2
        self._length = arm_length_m
        self._mass = mass_kg
        self._min = min_angle_rad
        self._max = max_angle_rad
        self._angle = start_angle_rad
        self._vel = 0.0
        self._voltage = 0.0
        self._kt = kt
        self._offset = 0.0
        self._last_ms = None

    # ── AngularPositionIO interface ───────────────────────────────────────────

    def update_inputs(self, inp):
        now = time.ticks_ms()
        if self._last_ms is None:
            self._last_ms = now
            inp.connected = True
            inp.position_rad = self._angle + self._offset
            inp.velocity_rad_s = 0.0
            return
        dt = time.ticks_diff(now, self._last_ms) * 0.001
        self._last_ms = now
        self._step(dt)
        inp.connected = True
        inp.position_rad = self._angle + self._offset
        inp.velocity_rad_s = self._vel

    def set_position(self, position_rad, feedforward=0.0):
        self._angle = position_rad - self._offset
        self._vel = 0.0

    def stop(self):
        self.set_voltage(0.0)

    def home(self, angle_rad=0.0):
        self._offset = angle_rad - self._angle

    # ── Extra: voltage-drive for open-loop tests ──────────────────────────────

    def set_voltage(self, voltage_v):
        self._voltage = max(-12.0, min(12.0, float(voltage_v)))

    # ── Physics ───────────────────────────────────────────────────────────────

    def _step(self, dt):
        tau_gravity = self._mass * self._G * self._length * math.cos(self._angle)
        tau_motor = self._kt * self._voltage
        alpha = (tau_motor - tau_gravity) / self._moi
        self._vel += alpha * dt
        self._angle += self._vel * dt
        if self._angle < self._min:
            self._angle = self._min
            self._vel = max(0.0, self._vel)
        elif self._angle > self._max:
            self._angle = self._max
            self._vel = min(0.0, self._vel)
