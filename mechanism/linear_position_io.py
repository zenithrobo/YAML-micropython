import time


class LinearPositionInputs:
    def __init__(self):
        self.connected = False
        self.position_m = 0.0
        self.velocity_m_s = 0.0


class LinearPositionIO:
    """
    Interface: a mechanism whose controlled quantity is linear position (metres).
    Typical implementations: lead-screw + encoder, rack-and-pinion, cable drum.
    Sim implementation included below.
    """

    def update_inputs(self, inp: LinearPositionInputs) -> None:
        pass

    def set_position(self, position_m: float, feedforward: float = 0.0) -> None:
        pass

    def stop(self) -> None:
        pass

    def home(self, position_m: float = 0.0) -> None:
        """Re-declare the current physical location as position_m (zero the encoder)."""
        pass


class LinearPositionIOSim(LinearPositionIO):
    """
    Sim: first-order lag toward setpoint.
    tau: time constant (seconds). feedforward is accepted but ignored in Sim.
    """

    def __init__(self, tau: float = 0.3):
        self._tau = tau
        self._pos = 0.0
        self._target = 0.0
        self._last_ms = None

    def set_position(self, position_m, feedforward=0.0):
        self._target = position_m

    def stop(self):
        self._target = self._pos

    def home(self, position_m=0.0):
        self._pos = position_m
        self._target = position_m

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
        error = self._target - self._pos
        delta = error * (1.0 - 2.718 ** (-dt / self._tau))
        inp.velocity_m_s = delta / dt if dt > 0 else 0.0
        self._pos += delta
        inp.position_m = self._pos
        inp.connected = True
