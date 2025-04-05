import time
import threading
from evdev import InputDevice, categorize, ecodes
from evdev import list_devices

class GamepadState:
    def __init__(self):
        # 按键状态（True 表示按下）
        self.A = False
        self.B = False
        self.X = False
        self.Y = False
        self.BACK = False
        self.START = False
        self.LB = False
        self.RB = False
        self.L3 = False    # 左摇杆按下
        self.R3 = False    # 右摇杆按下
        self.LT = 0        # 扳机值（0~255/1023）
        self.RT = 0
        self.LEFT_X = 0.0
        self.LEFT_Y = 0.0
        self.RIGHT_X = 0.0
        self.RIGHT_Y = 0.0
        self.DPAD_X = 0  # -1 左，0 无，1 右
        self.DPAD_Y = 0  # -1 上，0 无，1 下

    def __repr__(self):
        return (
            f"A={self.A} B={self.B} X={self.X} Y={self.Y} "
            f"BACK={self.BACK} START={self.START} "
            f"LB={self.LB} RB={self.RB} L3={self.L3} R3={self.R3} "
            f"LT={self.LT} RT={self.RT} "
            f"LEFT=({self.LEFT_X:.2f}, {self.LEFT_Y:.2f}) "
            f"RIGHT=({self.RIGHT_X:.2f}, {self.RIGHT_Y:.2f}) "
            f"DPAD=({self.DPAD_X}, {self.DPAD_Y})"
        )

class GamepadHandler:
    def __init__(self, device_path=None, deadzone=5000):
        self.device_path = device_path
        self.deadzone = deadzone
        self.is_connect = False
        self.gamepad = None  # 不立即绑定
        self.key_action_map = {
            "BTN_A": "A", "BTN_B": "B", "BTN_WEST": "Y", "BTN_NORTH": "X",
            "BTN_SELECT": "BACK", "BTN_START": "START",
            "BTN_TL": "LB", "BTN_TR": "RB",
            "BTN_THUMBL": "L3", "BTN_THUMBR": "R3",
        }
        self.state = GamepadState()

    def find_gamepad(self, keywords=("X-Box", "Xbox", "BEITONG", "pad")):
        for path in list_devices():
            dev = InputDevice(path)
            if any(k.lower() in dev.name.lower() for k in keywords):
                return path
        raise RuntimeError("未找到手柄设备")

    def normalize(self, val):
        return round(val / 32767.0, 2)

    def listen(self):
        while True:
            # 如果尚未绑定，先尝试绑定
            if not self.gamepad:
                self.reconnect()
            try:
                for event in self.gamepad.read_loop():
                    self.process_event(event)
                    self.handle_state()
            except (OSError, IOError) as e:
                print(f"设备断开或不可读: {e}")
                self.gamepad = None  # 清除旧设备，重新绑定
                time.sleep(1)

    def reconnect(self):
        print("尝试重新绑定手柄...")
        while True:
            try:
                self.device_path = self.find_gamepad()
                self.gamepad = InputDevice(self.device_path)
                self.is_connect = True
                print(f"手柄已重新绑定: {self.gamepad.name} @ {self.device_path}")
                break
            except Exception as e:
                self.is_connect = False
                print(f"手柄查找失败: {e}，1 秒后重试")
                time.sleep(1)

    def process_event(self, event):
        if event.type == ecodes.EV_KEY:
            key_event = categorize(event)
            keycode = key_event.keycode
            if isinstance(keycode, (list, tuple)):
                keycode = keycode[0]
            button = self.key_action_map.get(keycode, keycode)
            if key_event.keystate == key_event.key_down:
                setattr(self.state, button, True)
            elif key_event.keystate == key_event.key_up:
                setattr(self.state, button, False)

        elif event.type == ecodes.EV_ABS:
            code = event.code
            value = event.value
            # 左摇杆
            if code == ecodes.ABS_X:
                self.state.LEFT_X = -self.normalize(value)
            elif code == ecodes.ABS_Y:
                self.state.LEFT_Y = -self.normalize(value)
            # 右摇杆
            elif code == ecodes.ABS_RX:
                self.state.RIGHT_X = -self.normalize(value)
            elif code == ecodes.ABS_RY:
                self.state.RIGHT_Y = -self.normalize(value)
            # 十字键（HAT）
            elif code == ecodes.ABS_HAT0X:
                self.state.DPAD_X = -value
            elif code == ecodes.ABS_HAT0Y:
                self.state.DPAD_Y = -value
            # 扳机
            elif code == ecodes.ABS_Z:
                self.state.LT = value
            elif code == ecodes.ABS_RZ:
                self.state.RT = value

    def handle_state(self):
        # 在这里可以添加对 state 的处理逻辑
        pass

if __name__ == '__main__':
    handler = GamepadHandler()  # 不传 device_path，自动绑定
    threading.Thread(target=handler.listen, daemon=True).start()
    while True:
        if handler.is_connect:
            print(handler.state)  # 或者直接使用 handler.state.A、handler.state.LEFT_X 等
        time.sleep(1)