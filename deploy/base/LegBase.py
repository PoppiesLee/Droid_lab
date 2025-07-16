import math
import time
import csv
import numpy as np
from grpc import insecure_channel
from torch.nn.init import zeros_

from deploy.base.Base import *
from deploy.base.ConfigX3 import Config
from deploy.protos import droid_msg_pb2 as msg_pb2
from deploy.protos import leg_service_pb2_grpc as leg_pb2_grpc
class LegBase:
    def __init__(self):
        print("Initializing LegBase")
        self.LegEnvCfg = Config
        self.legActions = self.LegEnvCfg.num_leg_actions
        self.legConfigs = msg_pb2.DroidConfigs()
        self.legState = msg_pb2.DroidStateResponse()
        self.legCommand = msg_pb2.DroidCommandRequest()
        #
        # # --- CSV 保存相关的新增内容 ---
        # self.csv_file = None      # 文件对象
        # self.csv_writer = None    # CSV写入器
        # self.start_time_for_csv = time.time() # 用于CSV的时间戳基准
        #
        # # 确保目录存在，并创建文件名
        # timestamp = time.strftime("%Y%m%d-%H%M%S")
        # self.csv_filename = f"joint_positions_{timestamp}.csv"

        channel = insecure_channel(self.LegEnvCfg.grpc_channel + ":50051")
        print("Successfully connected to： ", self.LegEnvCfg.grpc_channel + ":50051")
        self.legStub = leg_pb2_grpc.LegServiceStub(channel)
        init_command(self.legCommand, self.legActions)
        # 建立通信，获取机器人底层信息
        self.get_leg_config()
        self.get_leg_state()
        set_joint_mode(self.legCommand, self.LegEnvCfg, self.legActions)

    #     # --- CSV 保存相关的新增内容：初始化 CSV 文件 ---
    #     try:
    #         self.csv_file = open(self.csv_filename, 'w', newline='') # 'w' 写入模式，newline='' 防止空行
    #         self.csv_writer = csv.writer(self.csv_file)
    #         print(f"CSV file '{self.csv_filename}' created successfully.")
    #     except Exception as e:
    #         print(f"Error creating CSV file: {e}")
    #         self.csv_file = None # 如果创建失败，确保文件对象为 None
    #
    # # --- CSV 保存相关的新增内容：析构函数，确保文件关闭 ---
    # def __del__(self):
    #     if self.csv_file:
    #         self.csv_file.close()
    #         print(f"CSV file '{self.csv_filename}' closed.")

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
            for idx in range(self.legActions):  # 假设关节数量是18
                qt = s0 * q0[idx] + s1 * qd[idx]
                self.legCommand.position[idx] = qt
            self.set_leg_command()
            # # --- CSV 保存相关的新增内容：写入数据 ---
            # if self.csv_writer:
            #     row_data = []
            #     # 添加指令位置数据
            #     for idx in range(self.legActions):
            #         # 确保 last_commanded_position 有足够多的元素
            #         if idx < len(self.legCommand.position):
            #             row_data.append(self.legCommand.position[idx])
            #         else:
            #             row_data.append(0.0)  # 填充默认值
            #
            #     for idx in range(self.legActions):
            #         if idx < len(self.legState.position):
            #             row_data.append(self.legState.position[idx])
            #         else:
            #             row_data.append(0.0)  # 如果数据不足，填充默认值
            #     try:
            #         self.csv_writer.writerow(row_data)
            #     except Exception as e:
            #         print(f"Error writing to CSV file: {e}")
            tt += dt
            timer.waiting(start_time)

    def testLeg(self):
        T = 0.7  # 总时间
        dt0 = np.zeros(self.legActions)
        dt1 = np.zeros(self.legActions)
        dt2 = np.zeros(self.legActions)
        D2R = math.pi / 180.0
        # dt1 = [0, 0,   0,  20*D2R,  0,   0, 0,    0,  -20*D2R,  0,   0, 0]
        # dt2 = [0, 0,   0,  -20*D2R,  0,   0, 0,    0,  20*D2R,  0,   0, 0]
             # mwr   mhrl    mhpl    mhyl    mkpl   mapl   marl    mwy       mhrr    mhpr    mhyr    mkpr    mapr    marr
        dt1 = [ 0, 20*D2R,   40*D2R,     0, -80*D2R, 40*D2R,      0,         0,    20*D2R,   40*D2R,     0, -80*D2R, 40*D2R,      0]
        dt2 = [ 0,      0,       0,      0,     0,     0,      0,     0,        0,       0,      0,      0,      0,      0]
        for i in range(14):
            dt0[i] = dt0[i]
            dt1[i] = dt1[i]
            dt2[i] = dt2[i]
        # while(True):
        #     print(self.q0)
        # 执行关节规划
        for i in range(10000):
            gBot.get_leg_state()
            # print(gBot.legState.position[1],gBot.legState.position[2])
            print("wave round %d" % (i * 2 + 1))
            self.set_leg_path(T, dt1)
            print("wave round %d" % (i * 2 + 2))
            self.set_leg_path(T, dt2)
        print("return to zero")
        self.set_leg_path(T, dt0)

# --- 主程序入口 ---
if __name__ == '__main__':
    gBot = LegBase()
    gBot.testLeg()
