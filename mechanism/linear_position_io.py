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

    def set_pid(self, kp: float, ki: float, kd: float) -> None:
        pass

    def home(self, position_m: float = 0.0) -> None:
        """Re-declare the current physical location as position_m (zero the encoder)."""
        pass

