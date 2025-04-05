import math
import time
import numpy as np
from deploy.base.Base import NanoSleep
from deploy.base.ArmBase import ArmBase
from deploy.base.LegBase import LegBase
from deploy.base.Config import Config


class RobotBase(ArmBase, LegBase):
    def __init__(self, _cfg):
        ArmBase.__init__(self)
        LegBase.__init__(self)
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
        T = 1  # 总时间
        arm_dt0 = np.zeros(self.armActions)
        arm_dt1 = np.zeros(self.armActions)
        arm_dt2 = np.zeros(self.armActions)
        if self.armActions == 8:
            arm_dt0 = np.array([round(math.radians(d), 4) for d in [-30, 10, 0,  80, -30, 10, 0,  80]])
            arm_dt1 = np.array([round(math.radians(d), 4) for d in [-30, 10, 0, 100,  30, 10, 0, 100]])
            arm_dt2 = np.array([round(math.radians(d), 4) for d in [30,  10, 0, 100, -30, 10, 0, 100]])
        if self.armActions == 10:
            arm_dt0 = np.array([round(math.radians(d), 4) for d in [-30, 10, 0,  80, -100, -30, 10, 0,  80, -100]])
            arm_dt1 = np.array([round(math.radians(d), 4) for d in [-30, 10, 0, 100, -100,  30, 10, 0, 100, -100]])
            arm_dt2 = np.array([round(math.radians(d), 4) for d in [30,  10, 0, 100, -100, -30, 10, 0, 100, -100]])

        leg_dt0 = np.zeros(self.legActions)
        leg_dt1 = np.zeros(self.legActions)
        leg_dt2 = np.zeros(self.legActions)
        if self.legActions == 10:
            leg_dt1 = np.array([round(math.radians(d), 4) for d in [0, 0, 30, -60, 30, 0, 0, 0, 0, 0]])
            leg_dt2 = np.array([round(math.radians(d), 4) for d in [0, 0, 0, 0, 0, 0, 0, 30, -60, 30]])
        elif self.legActions == 12:
            leg_dt1 = np.array([round(math.radians(d), 4) for d in [0, 0, 30, -60, 30, -0.3, 0, 0, 0, 0, 0, 0.3]])
            leg_dt2 = np.array([round(math.radians(d), 4) for d in [0, 0, 0, 0, 0, 0.3, 0, 0, 30, -60, 30, -0.3]])

        dt0 = np.concatenate((arm_dt0, leg_dt0))
        dt1 = np.concatenate((arm_dt1, leg_dt1))
        dt2 = np.concatenate((arm_dt2, leg_dt2))

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

