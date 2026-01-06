import math
import time
import csv
import numpy as np
from grpc import insecure_channel
from torch.nn.init import zeros_

from deploy.base.Base import *
from deploy.base.ConfigE1_DOG import Config
from droidup.api import droidup_msg_pb2 as msg_pb2
from droidup.api import leg_service_pb2_grpc as leg_pb2_grpc
class LegBase:
    def __init__(self):
        print("Initializing LegBase")
        self.LegEnvCfg = Config
        self.legActions = self.LegEnvCfg.num_leg_actions
        self.legConfigs = msg_pb2.DroidConfigs()
        self.legState = msg_pb2.DroidStateResponse()
        self.legCommand = msg_pb2.DroidCommandRequest()

        channel = insecure_channel(self.LegEnvCfg.grpc_channel + ":50051")
        print("Successfully connected to： ", self.LegEnvCfg.grpc_channel + ":50051")
        self.legStub = leg_pb2_grpc.LegServiceStub(channel)
        init_command(self.legCommand, self.legActions)
        # 建立通信，获取机器人底层信息
        self.get_leg_config()
        self.get_leg_state()
        set_joint_mode(self.legCommand, self.LegEnvCfg, self.legActions)

        # self.joint_time_logs = [[] for _ in range(self.legActions)]
        # self.joint_qt_logs = [[] for _ in range(self.legActions)]
        # self.joint_qc_logs = [[] for _ in range(self.legActions)]

    def get_leg_config(self):
        empty_request = msg_pb2.Empty()
        self.legConfigs = self.legStub.GetLegConfig(empty_request)
        print_configs(self.legConfigs)

    def get_leg_state(self):
        empty_request = msg_pb2.Empty()
        self.legState = self.legStub.GetLegState(empty_request)
        # print_state(self.legState, self.legConfigs)

    def set_leg_command(self):
        response = self.legStub.SetLegCommand(self.legCommand)
        if not response:  # Assuming the RPC method returns a response
            print("RPC failed")

    def set_leg_path(self, T, qd):
        s0, s1, st = 0.0, 0.0, 0.0
        tt = 0.0
        dt = 0.002
        q0 = [0.0] * self.legActions
        for idx in range(self.legActions):
            q0[idx] = self.legState.position[idx]
        timer = NanoSleep(2)  # 创建一个1毫秒的NanoSleep对象
        while tt < T + dt / 2.0:
            start_time = time.perf_counter()
            self.get_leg_state()
            st = min(tt / T, 1.0)
            s0 = 0.5 * (1.0 + math.cos(math.pi * st))
            s1 = 1 - s0
            for idx in range(self.legActions):
                qt = s0 * q0[idx] + s1 * qd[idx]
                self.legCommand.position[idx] = qt
            self.set_leg_command()
            tt += dt
            # for idx in range(self.legActions):
            #     qt_sample = self.legCommand.position[idx]  # 类似 Mtr->mc
            #     qc_sample = self.legState.position[idx]  # 类似 Mtr->aec
            #     self.joint_time_logs[idx].append(tt)
            #     self.joint_qt_logs[idx].append(qt_sample)
            #     self.joint_qc_logs[idx].append(qc_sample)

            timer.waiting(start_time)

    def testLeg(self):
        T = 0.5  # 总时间
        dt0 = np.zeros(self.legActions)
        dt1 = np.zeros(self.legActions)
        dt2 = np.zeros(self.legActions)
        D2R = math.pi / 180.0
               #  pitch    roll    yaw     knee    A_pitch  A_roll       pitch   roll     yaw    knee   A_pitch   A_roll  yaw
        # dt1 = [   0*D2R,  0*D2R,  0*D2R,  0*D2R,  0*D2R,  0*D2R,         0*D2R,  0*D2R,  0*D2R,  0*D2R,  0*D2R,  0*D2R,  0*D2R]
        # dt2 = [   0*D2R,  0*D2R,  0*D2R,  0*D2R,  0*D2R,  0*D2R,         0*D2R,  0*D2R,  0*D2R,  0*D2R,  0*D2R,  0*D2R,    0]
        dt1 = [round(math.radians(d), 4) for d in [0, 0, 0, 0, 0, 0, 0, 0]]
        dt2 = [round(math.radians(d), 4) for d in [20, 0, 0, -20, 20, 0, 0, -20]]
        # 执行关节规划
        for i in range(1000):
            gBot.get_leg_state()
            print("wave round %d" % (i * 2 + 1))
            self.set_leg_path(T, dt1)
            print("wave round %d" % (i * 2 + 2))
            self.set_leg_path(T, dt2)
        print("return to zero")
        self.set_leg_path(T, dt0)
        # for idx in range(self.legActions):
        #     file_path = f"/home/bot/下载/csvplot/data/joint_{idx}.csv"
        #     with open(file_path, mode="w", newline="") as f:
        #         writer = csv.writer(f)
        #         writer.writerow(["time", "qt", "qc"])
        #         for t, mc, aec in zip(
        #                 self.joint_time_logs[idx],
        #                 self.joint_qt_logs[idx],
        #                 self.joint_qc_logs[idx]):
        #             writer.writerow([f"{t:.6f}", f"{mc:.6f}", f"{aec:.6f}"])
        #     print(f"关节 {idx} 数据已保存到: {file_path}")

# --- 主程序入口 ---
if __name__ == '__main__':
    gBot = LegBase()
    gBot.testLeg()
