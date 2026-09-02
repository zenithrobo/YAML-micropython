import _thread
import time

from lib.interface import angle_sensor, angular_velocity_sensor
from machine import I2C, Pin

AS5600_ADDR = 0x36
REG_ANGLE_HI = 0x0C
REG_ANGLE_LO = 0x0D
REG_STATUS = 0x0B
REG_RAW_ANGLE_HI = 0x0C
MAGNET_HIGH = 0x20
MAGNET_LOW = 0x10

SPEED_WINDOW_US = 20000
MAX_SAMPLE_GAP_US = 3500
RPM_FILTER_ALPHA = 0.5


class AS5600Iic(angle_sensor, angular_velocity_sensor):
    def __init__(self, sda, scl, i2c_id=0, address=AS5600_ADDR):
        self.i2c = I2C(i2c_id, scl=Pin(scl), sda=Pin(sda), freq=400000)
        self.address = address
        self._lock = _thread.allocate_lock()
        self.rpm = 0.0
        self.rpm_valid = False
        self._last_raw = None
        self._last_t = None
        self._win_start = None
        self._win_delta = 0

    def connected(self):
        try:
            with self._lock:
                return self.address in self.i2c.scan()
        except Exception:
            return False

    def raw_angle(self):
        try:
            with self._lock:
                data = self.i2c.readfrom_mem(self.address, REG_RAW_ANGLE_HI, 2)
        except Exception:
            return None
        return (data[0] << 8 | data[1]) & 0x0FFF

    def get_rotations(self):
        raw = self.raw_angle()
        if raw is None:
            return None
        return raw / 4096.0

    @staticmethod
    def _unwrap(raw, last_raw):
        delta = raw - last_raw
        if delta > 2048:
            delta -= 4096
        elif delta < -2048:
            delta += 4096
        return delta

    def sample(self):
        raw = self.raw_angle()
        if raw is None:
            self.rpm_valid = False
            self._last_raw = None
            self._last_t = None
            return None
        now = time.ticks_us()
        if self._last_raw is None or self._last_t is None or self._win_start is None:
            self._last_raw = raw
            self._last_t = now
            self._win_start = now
            self._win_delta = 0
            return None
        gap = time.ticks_diff(now, self._last_t)
        if gap <= 0 or gap > MAX_SAMPLE_GAP_US:
            self.rpm_valid = False
            self._last_raw = raw
            self._last_t = now
            self._win_start = now
            self._win_delta = 0
            return None
        self._win_delta += self._unwrap(raw, self._last_raw)
        self._last_raw = raw
        self._last_t = now
        if time.ticks_diff(now, self._win_start) < SPEED_WINDOW_US:
            return None
        window_us = time.ticks_diff(now, self._win_start)
        w = self._win_delta * 60000000.0 / (4096.0 * window_us)
        with self._lock:
            if self.rpm_valid:
                self.rpm += RPM_FILTER_ALPHA * (w - self.rpm)
            else:
                self.rpm = w
                self.rpm_valid = True
        self._win_start = now
        self._win_delta = 0
        return self.rpm

    def get_rpm(self):
        with self._lock:
            return self.rpm

    def magnet_status(self):
        if not self.connected():
            return "NO"
        with self._lock:
            status = self.i2c.readfrom_mem(self.address, REG_STATUS, 1)[0]
        if status & MAGNET_HIGH:
            return "HIGH"
        if status & MAGNET_LOW:
            return "LOW"
        return "OK"
