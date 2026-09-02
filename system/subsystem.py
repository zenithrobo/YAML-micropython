from abc import ABC, abstractmethod

class Subsystem(ABC):
    @abstractmethod
    def period(self):
        """
        Periodic method called every loop.
        Args:
            dt (float): Time delta in seconds.
        """
        pass
