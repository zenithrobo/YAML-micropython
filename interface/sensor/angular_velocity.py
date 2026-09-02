from abc import ABC, abstractmethod
import math


class AngularVelocitySensor(ABC):
    @abstractmethod
    def get_rpm(self) -> float:
        """
        Get the angular velocity in RPM.
        """

    def get_radians_per_second(self) -> float:
        """
        Get the angular velocity in radians per second.
        """
        return self.get_rpm() / 60.0 * 2.0 * math.pi

    def get_degrees_per_second(self) -> float:
        """
        Get the angular velocity in degrees per second.
        """
        return self.get_rpm() / 60.0 * 360.0

    def get_frequency(self) -> float:
        """
        Get the angular velocity in Hz.
        """
        return self.get_rpm() / 60.0

    def get_period(self) -> float:
        """
        Get the angular velocity period in seconds.
        """
        freq = self.get_frequency()
        if freq <= 0.0:
            return float("inf")
        return 1.0 / freq
