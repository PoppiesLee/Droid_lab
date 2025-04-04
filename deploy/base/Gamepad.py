# lsusb
# sudo apt install evtest
# sudo evtest
# 查看 Xbox360 Controller event
# 查看X-Box event
# /dev/input/event22:     Microsoft X-Box 360 pad
# conda pip install evdev
import time
import threading
from evdev import InputDevice, categorize, ecodes

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
        self.LT = 0      # 扳机值（0~255/1023）
        self.RT = 0
        self.L3 = False    # 左摇杆按下
        self.R3 = False    # 右摇杆按下
        self.LB_status = 0          # 0:未按，1:短按，2:长按

        # 摇杆值（归一化到 -1.0 ~ 1.0）
        self.LEFT_X = 0.0
        self.LEFT_Y = 0.0
        self.RIGHT_X = 0.0
        self.RIGHT_Y = 0.0

        # 十字键（方向键）
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

from evdev import list_devices

class GamepadHandler:
    def __init__(self, device_path=None, deadzone=5000):
        self.device_path = device_path or self.find_gamepad()
        self.deadzone = deadzone
        self.gamepad = InputDevice(self.device_path)

        print(f"已绑定手柄: {self.gamepad.name} @ {self.device_path}")

        self.key_action_map = {
            "BTN_A": "A", "BTN_B": "B", "BTN_WEST": "Y", "BTN_NORTH": "X",
            "BTN_SELECT": "BACK", "BTN_START": "START",
            "BTN_TL": "LB", "BTN_TR": "RB",
            "BTN_THUMBL": "L3", "BTN_THUMBR": "R3",
        }

        self.state = {
            "buttons": set(),
            "left_stick": [0.0, 0.0],
            "right_stick": [0.0, 0.0],
            "dpad": [0, 0],
            "triggers": {"LT": 0, "RT": 0},
        }

    def find_gamepad(self, keywords=("X-Box", "Xbox", "BEITONG", "pad")):
        for path in list_devices():
            dev = InputDevice(path)
            if any(k.lower() in dev.name.lower() for k in keywords):
                return path
        raise RuntimeError("❌ 未找到手柄设备")


    def normalize(self, val):
        return round(val / 32767.0, 2)

    def listen(self, callback=None):
        for event in self.gamepad.read_loop():
            self.process_event(event)
            if callback:
                callback(self.state)

    def process_event(self, event):
        if event.type == ecodes.EV_KEY:
            key_event = categorize(event)
            keycode = key_event.keycode
            if isinstance(keycode, (list, tuple)):
                keycode = keycode[0]
            button = self.key_action_map.get(keycode, keycode)

            if key_event.keystate == key_event.key_down:
                self.state["buttons"].add(button)
            elif key_event.keystate == key_event.key_up:
                self.state["buttons"].discard(button)

        elif event.type == ecodes.EV_ABS:
            code = event.code
            value = event.value

            # 左摇杆
            if code == ecodes.ABS_X:
                self.state["left_stick"][0] = -self.normalize(value)
            elif code == ecodes.ABS_Y:
                self.state["left_stick"][1] = -self.normalize(value)
            # 右摇杆
            elif code == ecodes.ABS_RX:
                self.state["right_stick"][0] = -self.normalize(value)
            elif code == ecodes.ABS_RY:
                self.state["right_stick"][1] = -self.normalize(value)
            # 十字键（HAT）
            elif code == ecodes.ABS_HAT0X:
                self.state["dpad"][0] = -value
            elif code == ecodes.ABS_HAT0Y:
                self.state["dpad"][1] = -value
            # 扳机
            elif code == ecodes.ABS_Z:
                self.state["triggers"]["LT"] = value
            elif code == ecodes.ABS_RZ:
                self.state["triggers"]["RT"] = value

    def get_state(self):
        return self.state.copy()

def handle(state):
    buttons = state["buttons"]
    rc.A = "A" in buttons
    rc.B = "B" in buttons
    rc.X = "X" in buttons
    rc.Y = "Y" in buttons
    rc.BACK = "BACK" in buttons
    rc.START = "START" in buttons
    rc.LB = "LB" in buttons
    rc.RB = "RB" in buttons
    rc.L3 = "L3" in buttons  # 左摇杆按下
    rc.R3 = "R3" in buttons  # 右摇杆按下
    rc.LT = state["triggers"]["LT"]
    rc.RT = state["triggers"]["RT"]
    rc.LEFT_Y, rc.LEFT_X = state["left_stick"]
    rc.RIGHT_Y, rc.RIGHT_X = state["right_stick"]
    rc.DPAD_Y, rc.DPAD_X = state["dpad"]

    # 可选：调试打印
    # print(rc)


if __name__ == '__main__':
    handler = GamepadHandler()  # 不传 device_path，自动绑定
    # handler = GamepadHandler('/dev/input/event22')
    rc = GamepadState()
    threading.Thread(target=handler.listen, args=(handle,), daemon=True).start()
    while True:
        print(rc)  # 或者直接使用 rc.A、rc.LEFT_X 等
        time.sleep(0.05)



# sim2real

# if __name__ == '__main__':
#
#     handler = GamepadHandler()
#     rc = GamepadState()
#     threading.Thread(target=handler.listen, args=(handle,), daemon=True).start()
#     mode_path = Config.robot_config.mode_path
#     print("load mode = ", mode_path)
#     channel = insecure_channel('192.168.55.10:50051')
#     # jit
#     policy = torch.jit.load(mode_path)
#     # onnx
#     # policy = ort.InferenceSession(mode_path)
#     mybot = Sim2Real(Config, policy, channel,rc)
#     mybot.init_robot()
#     mybot.run()