class Sensor:
    def poll(self) -> None:
        """周期性驱动(主循环每帧调一次)。
        原生源(eRPM)无需采样 → 空实现;适配器在此采样+算 dt。
        默认空操作,所以原生源无需重写。"""
        pass

    def connect(self) -> bool:
        """连接传感器(初始化)。
        默认空操作,所以原生源无需重写。"""
        return False

    def warning(self) -> bool:
        """检查传感器是否警告。
        默认空操作,所以原生源无需重写。"""
        return True
