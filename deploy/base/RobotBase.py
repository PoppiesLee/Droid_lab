import math
import time
from base.Base import NanoSleep
from base.ArmBase import ArmBase
from base.LegBase import LegBase
from droidgym.Config import Config


class RobotBase(ArmBase, LegBase):
    def __init__(self, _cfg):
        ArmBase.__init__(self, _cfg)
        LegBase.__init__(self, _cfg)
        self.robot_actions = self.armActions + self.legActions

    # GRPC functions
    def get_robot_config(self):
        self.get_arm_config()
        self.get_leg_config()

    def get_robot_state(self):
        self.get_arm_state()
        self.get_leg_state()

    def set_robot_command(self):
        self.set_arm_command()
        self.set_leg_command()

    def set_robot_path(self, T, qd):
        s0, s1, st = 0.0, 0.0, 0.0
        tt = 0.0
        dt = 0.002
        q0 = [0.0] * self.robot_actions
        for idx in range(self.armActions):
            q0[idx] = self.armState.position[idx]
        for idx in range(self.legActions):
            q0[idx + self.armActions] = self.legState.position[idx]
        timer = NanoSleep(2)  # 创建一个1毫秒的NanoSleep对象
        while tt < T + dt / 2.0:
            start_time = time.perf_counter()
            self.get_robot_state()
            st = min(tt / T, 1.0)
            s0 = 0.5 * (1.0 + math.cos(math.pi * st))
            s1 = 1 - s0

            for idx in range(self.armActions):  # 假设关节数量是18
                qt = s0 * q0[idx] + s1 * qd[idx]
                self.armCommand.position[idx] = qt
            for idx in range(self.legActions):
                qt = s0 * q0[idx + self.armActions] + s1 * qd[idx + self.armActions]
                self.legCommand.position[idx] = qt
            self.set_robot_command()
            tt += dt
            timer.waiting(start_time)  # 等待下一个时间步长

    def run(self):
        T = 0.5  # 总时间
        dt0 = [0.] * self.robot_actions  # 假设 NMC 是一个定义好的常量，表示关节数量
        D2R = math.pi / 180.0
        # 填充 dt1 和 dt2 列表
        dt1 = [-30 * D2R, 10 * D2R, 0, 100 * D2R, -100 * D2R, 30 * D2R, 10 * D2R, 0, 100 * D2R, -100 * D2R,
               0, 0, 30 * D2R, -60 * D2R, 30 * D2R, 0, 0, 0, 0, 0]
        dt2 = [30 * D2R, 10 * D2R, 0, 100 * D2R, -100 * D2R, -30 * D2R, 10 * D2R, 0, 100 * D2R, -100 * D2R,
               0, 0, 0, 0, 0, 0, 0, 30 * D2R, -60 * D2R, 30 * D2R]

        # 执行关节规划
        for i in range(2):
            print("wave round %d" % (i * 2 + 1))
            self.set_robot_path(T, dt1)
            print("wave round %d" % (i * 2 + 2))
            self.set_robot_path(T, dt2)
        print("return to zero")
        gBot.set_robot_path(T, dt0)


if __name__ == '__main__':
    gBot = RobotBase(Config)
    gBot.run()

