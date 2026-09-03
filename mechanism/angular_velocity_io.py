class AngularVelocityInputs:
    def __init__(self):
        self.connected = False
        self.velocity_rad_s = 0.0
        self.position_rad = 0.0


class AngularVelocityIO:
    """
    Interface: a mechanism whose controlled quantity is angular velocity (rad/s).
    Typical applications: flywheel, conveyor roller, drive wheel.
    """

    def poll(self) -> None:
        pass  # override in sensor-backed implementations

    def update_inputs(self, inp: AngularVelocityInputs) -> None:
        pass

    def set_velocity(self, velocity_rad_s: float, feedforward: float = 0.0) -> None:
        pass

    def set_pid(self, kp: float, ki: float, kd: float) -> None:
        pass

    def stop(self) -> None:
        pass
