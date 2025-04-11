# 本代码适用于macos以及linux环境下的手柄键值读取，遥控器键值需要通过手动调用get_gamepad()更新
import time
import pygame

pygame.init()
pygame.joystick.init()

# Xbox 手柄按钮映射
BUTTON_MAP = {
    0: "A",
    1: "B",
    2: "X",
    3: "Y",
    4: "BACK",
    # 5: "HOME",
    6: "START",
    7: "L3",
    8: "R3",
    9: "LB",
    10: "RB",
    # macos 下有线或2.4G无线连接可以用
    # 11: "UP",
    # 12: "DOWN",
    # 13: "LEFT",
    # 14: "RIGHT",
}


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
        # macos 下有线或2.4G无线连接可以用
        # self.UP   = 0
        # self.DOWN = 0
        # self.LEFT   = 0
        # self.RIGHT = 0
        # macos 下蓝牙连接可以用
        self.DPAD_X = 0
        self.DPAD_Y = 0

    def __repr__(self):
        return (f"A={str(self.A):5} B={str(self.B):5} X={str(self.X):5} Y={str(self.Y):5} "
                f"BACK={str(self.BACK):5} START={str(self.START):5} "
                f"LB={str(self.LB):5} RB={str(self.RB):5} L3={str(self.L3):5} R3={str(self.R3):5} "
                f"LT={self.LT:5} RT={self.RT:5} "
                f"LEFT=(X: {self.LEFT_X:5.2f}, Y: {self.LEFT_Y:5.2f}) "
                f"RIGHT=(X: {self.RIGHT_X:5.2f}, Y: {self.RIGHT_Y:5.2f}) "
                # f"DPAD0=({self.UP:2}, {self.DOWN:2}), ({self.LEFT:2}, {self.RIGHT:2})"
                f"DPAD=({self.DPAD_X:2}, {self.DPAD_Y:2})")


class GamepadHandler:
    def __init__(self):
        self.__thread = None
        self.__RunFlag = True
        self.__gamepad = None  # 不立即绑定
        self.__device_path = None
        self.__is_connect = False
        self.state = GamepadState()

    def __reconnect(self):
        print("尝试重新绑定手柄...")
        while self.__RunFlag:
            try:
                # 检测已连接的手柄数量
                joystick_count = pygame.joystick.get_count()
                if joystick_count == 0:
                    print("未检测到手柄")
                # 初始化第一个手柄
                self.__gamepad = pygame.joystick.Joystick(0)
                self.__gamepad.init()
                self.__is_connect = True
                print(f"手柄名称: {self.__gamepad.get_name()}")
                break
            except Exception as e:
                self.__is_connect = False
                print(f"手柄查找失败: {e}，1 秒后重试")
                time.sleep(1)

    def __process_event(self, event):
        # 处理手柄按钮按下
        if event.type == pygame.JOYBUTTONDOWN:
            if event.button in BUTTON_MAP:
                button_name = BUTTON_MAP[event.button]
                setattr(self.state, button_name, True)  # 设为按下状态

        # 处理手柄按钮释放
        elif event.type == pygame.JOYBUTTONUP:
            if event.button in BUTTON_MAP:
                button_name = BUTTON_MAP[event.button]
                setattr(self.state, button_name, False)  # 设为释放状态

        if event.type == pygame.JOYAXISMOTION:
            if event.axis == 0:  # 左摇杆 X 轴
                self.state.LEFT_X = -event.value
            elif event.axis == 1:  # 左摇杆 Y 轴
                self.state.LEFT_Y = -event.value
            elif event.axis == 2:  # 右摇杆 X 轴
                self.state.RIGHT_X = -event.value
            elif event.axis == 3:  # 右摇杆 Y 轴
                self.state.RIGHT_Y = -event.value

        if event.type == pygame.JOYAXISMOTION:
            if event.axis == 4:  # LT（Xbox 手柄）
                self.state.LT = int((event.value + 1) * 127.5)  # 转换为 0~255
            elif event.axis == 5:  # RT（Xbox 手柄）
                self.state.RT = int((event.value + 1) * 127.5)

        if event.type == pygame.JOYHATMOTION:
            # event.hat 通常为 0（主方向键）
            dx, dy = event.value  # 值范围：(-1, 0, 1)
            self.state.DPAD_X = dx
            self.state.DPAD_Y = dy
        # print(self.state)

    def get_gamepad(self):
        if not self.__gamepad:
            self.__reconnect()
        try:
            for event in pygame.event.get():
                self.__process_event(event)
        except (OSError, IOError) as e:
            print(f"设备断开或不可读: {e}")
            self.__gamepad = None  # 清除旧设备，重新绑定
        return self.state

    def start_test(self):
        while self.__RunFlag:
            state = self.get_gamepad()
            print(state)
            time.sleep(1)


if __name__ == '__main__':
    rc = GamepadHandler()
    rc.start_test()
    print("Game Over !!!")
