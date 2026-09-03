from lib.device.ble_remote import BLERemote

DEFAULT_KEY_MAP = {
    "A1": "按键@: 单击",
    "B1": "按键@: 长按",
    "C1": "按键@: 长按释放",
    "A2": "按键A: 单击",
    "B2": "按键A: 长按",
    "C2": "按键A: 长按释放",
    "A3": "按键B: 单击",
    "B3": "按键B: 长按",
    "C3": "按键B: 长按释放",
    "A4": "按键C: 单击",
    "B4": "按键C: 长按",
    "C4": "按键C: 长按释放",
    "A5": "按键D: 单击",
    "B5": "按键D: 长按",
    "C5": "按键D: 长按释放",
    "A6": "按键R: 单击",
    "B6": "按键R: 长按",
    "C6": "按键R: 长按释放",
    "A7": "按键L: 单击",
    "B7": "按键L: 长按",
    "C7": "按键L: 长按释放",
    "D0": "方向: 无",
    "D1": "方向: 上",
    "D2": "方向: 下",
    "D3": "方向: 左",
    "D4": "方向: 右",
    "D5": "方向: 左上",
    "D6": "方向: 左下",
    "D7": "方向: 右上",
    "D8": "方向: 右下",
}


class LookbonRemote(BLERemote):
    """
    LOOKBON BLE 遥控。连接后即可直接收到按键通知，无需 GATT 发现。
    """

    def __init__(
        self,
        device_name="LOOKBON",
        target_mac=None,
        key_map=None,
        debug=True,
        reverse_mac=False,
    ):
        super().__init__(
            device_name,
            target_mac,
            key_map if key_map is not None else DEFAULT_KEY_MAP,
            debug,
            reverse_mac,
        )
