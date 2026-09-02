from abc import ABC, abstractmethod
import math


class AngleSensor(ABC):
    @abstractmethod
    def get_rotations(self) -> float:
        """
        Get the angle in rotations (number of full turns).
        Returns:
            float: Angle in rotations (1.0 = one full turn)
        """
        pass

    def get_radians(self) -> float:
        """
        Get the angle in radians.
        Returns:
            float: Angle in radians [0, 2π)
        """
        return self.get_rotations() * 2.0 * math.pi

    def get_degrees(self) -> float:
        """
        Get the angle in degrees.
        Returns:
            float: Angle in degrees [0, 360)
        """
        return self.get_rotations() * 360.0

    def get_normalized_rotation(self) -> float:
        """
        Get the angle normalized to [0, 1) rotation.
        Returns:
            float: Angle in rotations, normalized to [0, 1)
        """
        rotations = self.get_rotations()
        return rotations % 1.0

    def get_normalized_radians(self) -> float:
        """
        Get the angle normalized to [0, 2π) radians.
        Returns:
            float: Angle in radians, normalized to [0, 2π)
        """
        return self.get_normalized_rotation() * 2.0 * math.pi

    def get_normalized_degrees(self) -> float:
        """
        Get the angle normalized to [0, 360) degrees.
        Returns:
            float: Angle in degrees, normalized to [0, 360)
        """
        return self.get_normalized_rotation() * 360.0
