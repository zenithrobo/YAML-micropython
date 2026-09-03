import time
import ubinascii
import ubluetooth
from micropython import const

_IRQ_SCAN_RESULT = const(5)
_IRQ_SCAN_DONE = const(6)
_IRQ_PERIPHERAL_CONNECT = const(7)
_IRQ_PERIPHERAL_DISCONNECT = const(8)
_IRQ_GATTC_SERVICE_RESULT = const(9)
_IRQ_GATTC_SERVICE_DONE = const(10)
_IRQ_GATTC_CHARACTERISTIC_RESULT = const(11)
_IRQ_GATTC_CHARACTERISTIC_DONE = const(12)
_IRQ_GATTC_DESCRIPTOR_RESULT = const(13)
_IRQ_GATTC_DESCRIPTOR_DONE = const(14)
_IRQ_GATTC_NOTIFY = const(18)


def _hex_str(data):
    return str(ubinascii.hexlify(bytes(data)), "utf-8").upper()


def _adv_name(payload):
    payload = bytes(payload)
    n = 0
    while n + 1 < len(payload):
        ln = payload[n]
        if ln == 0:
            break
        typ = payload[n + 1]
        if typ in (0x09, 0x08):
            try:
                return payload[n + 2 : n + 1 + ln].decode("utf-8")
            except UnicodeError:
                return None
        n += 1 + ln
    return None


def _decode_mac(addr, reverse=False):
    b = bytes(addr)
    if reverse:
        b = b[::-1]
    return ":".join("{:02X}".format(x) for x in b)


class BLERemote:
    """
    BLE 遥控基类。

    子类通过覆写 _on_connected() / _on_disconnected() / _on_gattc_extra()
    来实现不同厂商的连接握手协议，对外暴露接口保持一致。
    """

    def __init__(
        self,
        device_name,
        target_mac=None,
        key_map=None,
        debug=True,
        reverse_mac=False,
    ):
        self.device_name = device_name
        self.target_mac = (
            target_mac.upper().replace("-", ":").replace(" ", "")
            if target_mac
            else None
        )
        self.key_map = key_map if key_map is not None else {}
        self.debug = debug
        self.reverse_mac = reverse_mac

        self.ble = ubluetooth.BLE()
        self.ble.active(True)
        self.ble.irq(self._irq)

        self.found = False
        self.connected = False
        self.scanning = False

        self.addr_type = None
        self.addr = None
        self.mac = ""
        self.conn_handle = None
        self.last_notify = 0

        self.on_key = None
        self.on_raw = None
        self.on_connect = None
        self.on_disconnect = None

    # ── 公开 API ──────────────────────────────────────────────────────────────

    def set_key_callback(self, callback):
        self.on_key = callback

    def set_raw_callback(self, callback):
        self.on_raw = callback

    def set_connect_callback(self, callback):
        self.on_connect = callback

    def set_disconnect_callback(self, callback):
        self.on_disconnect = callback

    def start(self, duration_ms=5000, interval_us=30000, window_us=30000):
        if self.scanning:
            if self.debug:
                print("already scanning")
            return
        self.found = False
        self.connected = False
        self.scanning = True
        if self.debug:
            print("scanning for:", self.target_mac or self.device_name)
        try:
            self.ble.gap_scan(duration_ms, interval_us, window_us)
        except OSError as e:
            if self.debug:
                print("scan start failed:", e, "- will retry")
            self.scanning = False

    def stop_scan(self):
        self.scanning = False
        try:
            self.ble.gap_scan(None)
        except Exception as e:
            if self.debug:
                print("stop scan failed:", e)

    def disconnect(self):
        if self.conn_handle is not None:
            try:
                self.ble.gap_disconnect(self.conn_handle)
            except Exception as e:
                if self.debug:
                    print("disconnect failed:", e)

    def close(self):
        self.stop_scan()
        self.disconnect()
        self.ble.irq(None)
        self.ble.active(False)

    def stale(self, timeout_ms=10000):
        if not self.connected:
            return True
        return time.ticks_diff(time.ticks_ms(), self.last_notify) > timeout_ms

    # ── 子类钩子 ──────────────────────────────────────────────────────────────

    def _on_connected(self, conn_handle):
        """BLE 连接建立后调用。默认立即触发 on_connect（适用于 LOOKBON 等简单协议）。"""
        if self.on_connect:
            self.on_connect(self.mac)

    def _on_disconnected(self):
        """BLE 断开后调用。子类可在此清理协议状态。"""
        pass

    def _on_gattc_extra(self, event, data):
        """NOTIFY 之外的 GATTC 事件。需要 GATT 发现的子类在此处理。"""
        pass

    def _transform_key(self, key_hex, key_text):
        """通知预处理钩子。返回 (emit, emit_text)；emit=False 则丢弃本次通知。"""
        return True, key_text

    # ── 内部实现 ─────────────────────────────────────────────────────────────

    def _mark_active(self):
        self.last_notify = time.ticks_ms()

    def _handle_notify(self, conn_handle, value_handle, notify_data):
        self._mark_active()
        key_hex = _hex_str(notify_data)
        key_text = self.key_map.get(key_hex)
        emit, emit_text = self._transform_key(key_hex, key_text)
        if not emit:
            return

        if self.debug:
            if key_hex in self.key_map:
                if emit_text and emit_text != "RELEASE":
                    print(self.mac, "key:", emit_text)
            else:
                print(self.mac, "unknown h:", value_handle, "data:", key_hex)

        if self.on_raw:
            self.on_raw(key_hex, notify_data, value_handle)
        if self.on_key:
            self.on_key(key_hex, emit_text)

    def _irq(self, event, data):
        if event == _IRQ_SCAN_RESULT:
            if self.found:
                return
            addr_type, addr, adv_type, rssi, adv_data = data
            mac = _decode_mac(addr, self.reverse_mac)
            name = _adv_name(adv_data)

            matched = False
            if self.target_mac:
                matched = mac == self.target_mac
            elif name and name.upper() == self.device_name.upper():
                matched = True

            if matched:
                self.found = True
                self.scanning = False
                self.addr_type = addr_type
                self.addr = bytes(addr)
                self.mac = mac
                print("found:", self.mac)
                self.ble.gap_scan(None)
                self.ble.gap_connect(addr_type, addr)

        elif event == _IRQ_SCAN_DONE:
            self.scanning = False
            if not self.found and self.debug:
                print("scan done: not found")

        elif event == _IRQ_PERIPHERAL_CONNECT:
            conn_handle, addr_type, addr = data
            self.conn_handle = conn_handle
            self.connected = True
            self._mark_active()
            if self.debug:
                print("connected:", self.mac, "handle:", conn_handle)
            self._on_connected(conn_handle)

        elif event == _IRQ_PERIPHERAL_DISCONNECT:
            conn_handle, addr_type, addr = data
            self.connected = False
            self.conn_handle = None
            if self.debug:
                print("disconnected:", self.mac)
            self._on_disconnected()
            if self.on_disconnect:
                self.on_disconnect(self.mac)

        elif event == _IRQ_GATTC_NOTIFY:
            conn_handle, value_handle, notify_data = data
            self._handle_notify(conn_handle, value_handle, notify_data)

        else:
            self._on_gattc_extra(event, data)
