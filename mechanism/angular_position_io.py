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

    def set_pid(self, kp: float, ki: float, kd: float) -> None:
        pass

    def home(self, angle_rad: float = 0.0) -> None:
        """Re-declare the current physical angle as angle_rad (zero the encoder)."""
        pass

