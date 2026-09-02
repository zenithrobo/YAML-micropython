import math


class RelativeAngleSensor:
    """相对/增量角度传感器(存量 / 有记忆)。

    语义:报告"从某个零点(通常是开机点)累计转过了多少",单调累加、**不回绕**。
    - 记忆累计位置(3.5 = 转了三圈半),但**不知道绝对位置**(零点是任意的)。
    - 这是一个存量:幂等,慢读也不丢总量,差分不需要 unwrap。
    - 速度不是它的输出;若需要速度,由 mechanism 的 Real 对本传感器差分得到。
    典型实现:ABZ 增量编码器 + 硬件计数器(PCNT)。

    实现者只需实现 get_accumulated_rotations();其余单位换算由基类导出。
    注意:不要在这里做微分/滤波。累加应由硬件(PCNT)或明确标注的适配器完成,
    见 UnwrappingAccumulator(把绝对源软件累加成相对量,仅限低速)。
    """

    # ---- 唯一需要子类实现的方法 ----
    def get_accumulated_rotations(self) -> float:
        """自零点起的累计角度,单位=圈,可为任意大/负,**不回绕**。读一次即得,不含时间。

        Returns:
            float: 累计圈数(如 12.75);读取失败返回 None。
        """
        raise NotImplementedError

    # ---- 基类导出的单位视图 ----
    def get_accumulated_radians(self) -> float:
        """累计角度,单位=弧度,不回绕。"""
        r = self.get_accumulated_rotations()
        return None if r is None else r * 2.0 * math.pi

    def get_accumulated_degrees(self) -> float:
        """累计角度,单位=度,不回绕。"""
        r = self.get_accumulated_rotations()
        return None if r is None else r * 360.0

    def get_normalized_rotations(self) -> float:
        """归一化后的累计角度,单位=圈,不回绕。"""
        r = self.get_accumulated_rotations()
        return None if r is None else (r % 1.0)

    def get_normalized_radians(self) -> float:
        """归一化后的累计角度,单位=弧度,不回绕。"""
        r = self.get_accumulated_radians()
        return None if r is None else (r % (2.0 * math.pi))

    def get_normalized_degrees(self) -> float:
        """归一化后的累计角度,单位=度,不回绕。"""
        r = self.get_accumulated_degrees()
        return None if r is None else (r % 360.0)

    def reset_accumulated_rotations(self, rotations: float = 0.0) -> None:
        """把当前位置设为新的零点(累计归零)。可选实现;默认不支持。

        注意:这是相对量的语义 —— 它有"零点"这个概念,而绝对传感器没有,
        这正是两个接口不能合并的原因之一。
        """
        raise NotImplementedError

    def reset_accumulated_radians(self, radians: float = 0.0) -> None:
        self.reset_accumulated_rotations(radians / (2.0 * math.pi))

    def reset_accumulated_degrees(self, degrees: float = 0.0) -> None:
        self.reset_accumulated_radians(degrees / 360.0 * math.pi)
