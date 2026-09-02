import math


class AngularVelocitySensor:
    """角速度传感器(原生速度源 / 读一次即得)。

    语义:报告"当前转多快",带方向(正负),单位以"圈/秒(rev/s)"为基准。
    - 这是**原生速度**:时间已在设备内部消化(如电调 eRPM 的内部时钟、
      测速发电机的物理特性)。读一次就得到速度,不需要外部提供 dt。
    - 只有原生输出速度的源才实现本接口。**只输出角度的源不实现它** ——
      那种情况下速度是导出量,由 mechanism 的 Real 对角度差分得到
      (见 SKILL.md: velocity is a derivative, not a third kind of hardware)。
    典型实现:电调 eRPM 遥测、测速发电机(tachometer)。

    实现者只需实现 get_velocity_rps();其余单位视图由基类导出。
    不要在这里做滤波/微分/积分 —— 本接口只暴露原生速度。
    """

    _TWO_PI = 2.0 * math.pi

    # ---- 唯一需要子类实现的方法 ----
    def get_velocity_rps(self) -> float:
        """当前角速度,单位=圈/秒(rev/s),带符号。读一次即得。

        Returns:
            float: 转速(rev/s),正负表方向;读取失败返回 None。
        """
        raise NotImplementedError

    # ---- 以下为基类导出的单位视图,子类无需重写 ----
    def get_velocity_rad_s(self) -> float:
        """当前角速度,单位=弧度/秒(rad/s)。控制律最常用。"""
        v = self.get_velocity_rps()
        return None if v is None else v * self._TWO_PI

    def get_velocity_deg_s(self) -> float:
        """当前角速度,单位=度/秒(deg/s)。"""
        v = self.get_velocity_rps()
        return None if v is None else v * 360.0

    def get_velocity_rpm(self) -> float:
        """当前角速度,单位=转/分(rpm)。标定/直觉常用。"""
        v = self.get_velocity_rps()
        return None if v is None else v * 60.0
