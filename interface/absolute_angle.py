import math

from lib.interface.angular_velocity import AngularVelocitySensor
from lib.system.sensor import Sensor


class AbsoluteAngleSensor(Sensor):
    """绝对角度传感器(流量 / 无记忆)。

    语义:报告"当前在圆周上的位置",范围回绕 [0, 1) 圈。
    - 上电即知绝对位置(不需要归零),但**不记忆转过多少圈**。
    - 这是一个流量快照:两次读之间的历史不被保存。慢读会丢历史,
      高速差分会混叠。测速/累计不是它的职责。
    典型实现:AS5600、MT6701-SSI、KTH7824-SSI。

    实现者只需实现 get_rotation();其余单位换算由本基类导出。
    不要在这里做微分、累加、滤波 —— 那是 mechanism 的活。
    """

    # ---- 唯一需要子类实现的方法 ----
    def get_rotation(self) -> float:
        """当前绝对角度,单位=圈,范围 [0, 1)。读一次即得,不含时间。

        Returns:
            float: [0.0, 1.0) 的当前角度;读取失败返回 None。
        """
        raise NotImplementedError

    # ---- 以下为基类导出的单位视图,子类无需重写 ----
    def get_radians(self) -> float:
        """当前绝对角度,单位=弧度,范围 [0, 2π)。"""
        r = self.get_rotation()
        return None if r is None else r * 2.0 * math.pi

    def get_degrees(self) -> float:
        """当前绝对角度,单位=度,范围 [0, 360)。"""
        r = self.get_rotation()
        return None if r is None else r * 360.0


class AbsoluteToVelocity(AngularVelocitySensor):
    """把 AbsoluteAngleSensor(回绕角度/流量)微分成角速度。

    ⚠️ 回绕角每圈跳回 0,做差必须 unwrap,而 unwrap 要求两次 update()
       之间转 < 半圈,否则误判、测速跳变。**仅限低速!**
       (max_rpm 越高,半圈时间越短,越难保证 —— 见下 max_rpm 推导。)
       高速飞轮请用 RelativeToVelocity + ABZ,别用本适配器。

    必须由主循环反复调用 update();调用间隔必须 < max_gap(由 max_rpm 推导)。
    """

    def __init__(self, source, max_rpm, gap_safety=0.5, filter_alpha=None):
        self._src = source  # AbsoluteAngleSensor
        self._alpha = filter_alpha
        # 由 max_rpm 推导采样间隔上限:两次采样必须 < 半圈(防混叠的物理硬约束)
        us_per_half_turn = 0.5 * 60_000_000.0 / max_rpm
        self._max_gap_us = int(us_per_half_turn * gap_safety)
        self._last_rot = None
        self._last_us = None
        self._vel_rps = 0.0
        self._valid = False

    def poll(self):
        """采一次并更新速度估计。间隔超过 max_gap 会判为无效(可能已混叠)。"""
        rot = self._src.get_rotation()  # [0,1) 回绕
        now = time.ticks_us()
        if rot is None:
            self._invalidate()
            return None
        if self._last_rot is None:
            self._last_rot, self._last_us = rot, now
            return None
        gap = time.ticks_diff(now, self._last_us)
        if gap <= 0 or gap > self._max_gap_us:  # 采样太慢 → 可能跨了半圈 → 不可信
            self._invalidate()
            self._last_rot, self._last_us = rot, now
            return None
        d = rot - self._last_rot
        if d > 0.5:
            d -= 1.0  # unwrap:跨回绕点
        if d < -0.5:
            d += 1.0
        dt = gap / 1_000_000.0
        raw = d / dt
        self._last_rot, self._last_us = rot, now
        if self._alpha is None or not self._valid:
            self._vel_rps = raw
        else:
            self._vel_rps += self._alpha * (raw - self._vel_rps)
        self._valid = True
        return self._vel_rps

    def _invalidate(self):
        self._valid = False
        self._last_rot = self._last_us = None

    # ---- AngularVelocitySensor 契约 ----
    def get_velocity_rps(self):
        return self._vel_rps if self._valid else None
