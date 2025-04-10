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
        return (f"A={str(self.A):5} B={str(self.B):5} X={str(self.X):5} Y={str(self.Y):5} "
                f"BACK={str(self.BACK):5} START={str(self.START):5} "
                f"LB={str(self.LB):5} RB={str(self.RB):5} L3={str(self.L3):5} R3={str(self.R3):5} "
                f"LT={self.LT:5} RT={self.RT:5} "
                f"LEFT=(X: {self.LEFT_X:5.2f}, Y: {self.LEFT_Y:5.2f}) "
                f"RIGHT=(X: {self.RIGHT_X:5.2f}, Y: {self.RIGHT_Y:5.2f}) "
                f"DPAD=({self.DPAD_X:2}, {self.DPAD_Y:2})")

class GamepadHandler:
    def __init__(self):
        self.__thread = None
        self.__RunFlag = None
        self.__gamepad = None  # 不立即绑定
        self.__device_path = None
        self.__is_connect = False
        self.__key_action_map = {
            "BTN_A": "A", "BTN_B": "B", "BTN_WEST": "Y", "BTN_NORTH": "X",
            "BTN_SELECT": "BACK", "BTN_START": "START",
            "BTN_TL": "LB", "BTN_TR": "RB",
            "BTN_THUMBL": "L3", "BTN_THUMBR": "R3",
        }
        self.state = GamepadState()
        self.__start_server()

    def __find_gamepad(self, keywords=("X-Box", "Xbox", "BEITONG", "pad")):
        for path in list_devices():
            dev = InputDevice(path)
            if any(k.lower() in dev.name.lower() for k in keywords):
                return path
        raise RuntimeError("未找到手柄设备")

    def __normalize(self, val):
        return round(val / 32767.0, 2)

    def __listen(self):
        while self.__RunFlag:
            # 如果尚未绑定，先尝试绑定
            if not self.__gamepad:
                self.__reconnect()
            try:
                for event in self.__gamepad.read_loop():
                    self.__process_event(event)
                    self.__handle_state()
            except (OSError, IOError) as e:
                print(f"设备断开或不可读: {e}")
                self.__gamepad = None  # 清除旧设备，重新绑定
                time.sleep(1)

    def __reconnect(self):
        print("尝试重新绑定手柄...")
        while self.__RunFlag:
            try:
                self.__device_path = self.__find_gamepad()
                self.__gamepad = InputDevice(self.__device_path)
                self.__is_connect = True
                print(f"手柄已重新绑定: {self.__gamepad.name} @ {self.__device_path}")
                break
            except Exception as e:
                self.__is_connect = False
                print(f"手柄查找失败: {e}，1 秒后重试")
                time.sleep(1)

    def __process_event(self, event):
        if event.type == ecodes.EV_KEY:
            key_event = categorize(event)
            keycode = key_event.keycode
            if isinstance(keycode, (list, tuple)):
                keycode = keycode[0]
            button = self.__key_action_map.get(keycode, keycode)
            if key_event.keystate == key_event.key_down:
                setattr(self.state, button, True)
            elif key_event.keystate == key_event.key_up:
                setattr(self.state, button, False)

        elif event.type == ecodes.EV_ABS:
            code = event.code
            value = event.value
            # 左摇杆
            if code == ecodes.ABS_X:
                self.state.LEFT_X = -self.__normalize(value)
            elif code == ecodes.ABS_Y:
                self.state.LEFT_Y = -self.__normalize(value)
            # 右摇杆
            elif code == ecodes.ABS_RX:
                self.state.RIGHT_X = -self.__normalize(value)
            elif code == ecodes.ABS_RY:
                self.state.RIGHT_Y = -self.__normalize(value)
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
        elif event.type != 0:
            print(event.type)

    def __handle_state(self):
        # 在这里可以添加对 state 的处理逻辑
        pass

    def __start_server(self):
        self.__RunFlag = True
        self.__thread = threading.Thread(target=self.__listen, daemon=True)
        self.__thread.start()

    def stop_server(self):
        # 设置标志位为 False，触发服务器关闭
        self.__RunFlag = False
        if self.__thread:
            self.__thread.join()  # 等待线程结束
        print("Server shutdown requested.")
    def get_connect_state(self):
        return self.__is_connect

if __name__ == '__main__':
    rc = GamepadHandler()
    while not rc.state.A:
        if rc.get_connect_state():
            print(rc.state)
        time.sleep(1)
    print("Game Over !!!")