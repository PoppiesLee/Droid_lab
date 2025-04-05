import math
from grpc import insecure_channel
from deploy.base.Base import *
from deploy.base.Config import Config
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
        channel = insecure_channel(self.LegEnvCfg.grpc_channel + ":50051")
        print("Successfully connected to： ", self.LegEnvCfg.grpc_channel + ":50051")
        self.legStub = leg_pb2_grpc.LegServiceStub(channel)
        init_command(self.legCommand, self.legActions)
        # 建立通信，获取机器人底层信息
        self.get_leg_config()
        self.get_leg_state()
        # 电机空间控制或关节空间控制
        set_motor_mode(self.legCommand, self.legConfigs)

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
            tt += dt
            timer.waiting(start_time)  # 等待下一个时间步长

    def testLeg(self):
        T = 1  # 总时间
        dt0 = np.zeros(self.legActions)
        dt1 = np.zeros(self.legActions)
        dt2 = np.zeros(self.legActions)
        # 填充 dt1 和 dt2 列表
        if self.legActions == 10:
            dt1 = [round(math.radians(d), 4) for d in [0, 0, 30, -60, 30, 0, 0, 0, 0, 0]]
            dt2 = [round(math.radians(d), 4) for d in [0, 0, 0, 0, 0, 0, 0, 30, -60, 30]]
        elif self.legActions == 12:
            dt1 = [round(math.radians(d), 4) for d in [0, 0, 30, -60, 30, -0.3, 0, 0, 0, 0, 0, 0.3]]
            dt2 = [round(math.radians(d), 4) for d in [0, 0, 0, 0, 0, 0.3, 0, 0, 30, -60, 30, -0.3]]
        # 执行关节规划
        for i in range(2):
            print("wave round %d" % (i * 2 + 1))
            self.set_leg_path(T, dt1)
            print("wave round %d" % (i * 2 + 2))
            self.set_leg_path(T, dt2)
        print("return to zero")
        gBot.set_leg_path(T, dt0)


if __name__ == '__main__':
    gBot = LegBase()
    gBot.testLeg()
