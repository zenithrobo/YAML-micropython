from machine import I2C, Pin
from lib.interface.absolute_angle import AbsoluteAngleSensor

AS5600_ADDR = 0x36
REG_RAW_ANGLE_HI = 0x0C
REG_STATUS = 0x0B
MAGNET_HIGH = 0x20
MAGNET_LOW = 0x10


class AS5600(AbsoluteAngleSensor):
    """AS5600 磁角度传感器(I2C)"""

    def __init__(self, sda, scl, i2c_id=0, address=AS5600_ADDR, freq=400000):
        self.i2c = I2C(i2c_id, scl=Pin(scl), sda=Pin(sda), freq=freq)
        self.address = address

    # ---- AbsoluteAngleSensor 契约:唯一必须实现的方法 ----
    def get_rotation(self):
        """当前绝对角度 [0,1) 圈;读一次即得;失败返回 None。"""
        raw = self._raw_angle()
        return None if raw is None else raw / 4096.0

    # ---- 设备诊断(转发芯片信号,不做判断逻辑)----
    def connected(self):
        try:
            return self.address in self.i2c.scan()
        except Exception:
            return False

    def magnet_ok(self):
        """磁场强度是否正常(转发 AS5600 的 MD/ML/MH 状态)。失败返回 False。"""
        try:
            status = self.i2c.readfrom_mem(self.address, REG_STATUS, 1)[0]
        except Exception:
            return False
        return not (status & MAGNET_HIGH or status & MAGNET_LOW)

    def magnet_status(self):
        """'OK' / 'HIGH'(太近) / 'LOW'(太远) / 'NO'(读失败)。"""
        try:
            status = self.i2c.readfrom_mem(self.address, REG_STATUS, 1)[0]
        except Exception:
            return "NO"
        if status & MAGNET_HIGH:
            return "HIGH"
        if status & MAGNET_LOW:
            return "LOW"
        return "OK"

    # ---- 私有:原始寄存器读取 ----
    def _raw_angle(self):
        try:
            data = self.i2c.readfrom_mem(self.address, REG_RAW_ANGLE_HI, 2)
        except Exception:
            return None
        return (data[0] << 8 | data[1]) & 0x0FFF
