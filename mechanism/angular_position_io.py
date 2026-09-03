import time


class AngularPositionInputs:
    def __init__(self):
        self.connected = False
        self.position_rad = 0.0
        self.velocity_rad_s = 0.0


class AngularPositionIO:
    """
    Interface: a mechanism whose controlled quantity is angular position (radians).
    Typical implementations: servo arm + encoder, wrist joint, turret with feedback.
    Sim implementation included below.
    """

    def update_inputs(self, inp: AngularPositionInputs) -> None:
        pass

    def set_position(self, position_rad: float, feedforward: float = 0.0) -> None:
        pass

    def stop(self) -> None:
        pass

    def home(self, angle_rad: float = 0.0) -> None:
        """Re-declare the current physical angle as angle_rad (zero the encoder)."""
        pass


class AngularPositionIOSim(AngularPositionIO):
    """
    Sim: first-order lag toward setpoint.
    tau: time constant (seconds). feedforward is accepted but ignored in Sim.
    """

    def __init__(self, tau: float = 0.3):
        self._tau = tau
        self._pos = 0.0
        self._target = 0.0
        self._last_ms = None

    def set_position(self, position_rad, feedforward=0.0):
        self._target = position_rad

    def stop(self):
        self._target = self._pos

    def home(self, angle_rad=0.0):
        self._pos = angle_rad
        self._target = angle_rad

    def update_inputs(self, inp):
        now = time.ticks_ms()
        if self._last_ms is None:
            self._last_ms = now
            inp.connected = True
            inp.position_rad = self._pos
            inp.velocity_rad_s = 0.0
            return
        dt = time.ticks_diff(now, self._last_ms) * 0.001
        self._last_ms = now
        error = self._target - self._pos
        delta = error * (1.0 - 2.718 ** (-dt / self._tau))
        inp.velocity_rad_s = delta / dt if dt > 0 else 0.0
        self._pos += delta
        inp.position_rad = self._pos
        inp.connected = True
