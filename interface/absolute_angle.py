import math


class AbsoluteAngleSensor:
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
