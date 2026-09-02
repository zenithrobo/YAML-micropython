class Sensor:
    def poll(self) -> None:
        """周期性驱动(主循环每帧调一次)。
        原生源(eRPM)无需采样 → 空实现;适配器在此采样+算 dt。
        默认空操作,所以原生源无需重写。"""
        pass
