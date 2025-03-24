import math
from Base import *
from deploy.policies.Config import Config
from grpc import insecure_channel
from deploy.protos import arm_service_pb2_grpc as arm_pb2_grpc
from deploy.protos import droid_msg_pb2 as msg_pb2


class ArmBase:
    def __init__(self):
        print("Initializing ArmBase")
        self.ArmEnvCfg = Config
        self.armActions = self.ArmEnvCfg.num_arm_actions
        # grpc defines
        self.armConfigs = msg_pb2.DroidConfigs()
        self.armState = msg_pb2.DroidArmResponse()
        self.armCommand = msg_pb2.DroidCommandRequest()
        channel = insecure_channel(self.ArmEnvCfg.grpc_channel + ":50052")
        self.armStub = arm_pb2_grpc.ArmServiceStub(channel)
        init_command(self.armCommand, self.ArmEnvCfg.num_arm_actions)
        for idx in range(12):
            self.armCommand.finger.append(10)
        # 建立通信，获取机器人底层信息
        self.get_arm_config()
        self.get_arm_state()
        # 电机空间控制或关节空间控制
        set_motor_mode(self.armCommand, self.armConfigs)
        # self.set_joint_mode(self.armCommand)

    def get_arm_config(self):
        empty_request = msg_pb2.Empty()
        self.armConfigs = self.armStub.GetArmConfig(empty_request)

    def get_arm_state(self):
        empty_request = msg_pb2.Empty()
        self.armState = self.armStub.GetArmState(empty_request)
        # print_state(self.armState, self.armConfigs)

    def set_arm_command(self):
        response = self.armStub.SetArmCommand(self.armCommand)
        if not response:  # Assuming the RPC method returns a response
            print("RPC failed")

    def set_arm_path(self, T, qd):
        s0, s1, st = 0.0, 0.0, 0.0
        tt = 0.0
        dt = 0.002
        q0 = [0.0] * self.armActions
        for idx in range(self.armActions):
            q0[idx] = self.armState.position[idx]
        timer = NanoSleep(2)  # 创建一个1毫秒的NanoSleep对象
        while tt < T + dt / 2.0:
            start_time = time.perf_counter()
            self.get_arm_state()
            st = min(tt / T, 1.0)
            s0 = 0.5 * (1.0 + math.cos(math.pi * st))
            s1 = 1 - s0

            for idx in range(self.armActions):  # 假设关节数量是18
                qt = s0 * q0[idx] + s1 * qd[idx]
                self.armCommand.position[idx] = qt
            self.set_arm_command()
            tt += dt
            timer.waiting(start_time)  # 等待下一个时间步长

    def testArm(self):
        T = 1  # 总时间
        D2R = math.pi / 180.0
        dt0 = [-30, 10, 0, 80, -30, 10, 0, 80]  # 假设 NMC 是一个定义好的常量，表示关节数量
        dt0 = [x * D2R for x in dt0]
        # 填充 dt1 和 dt2 列表
        dt1 = [-30 * D2R, 10 * D2R, 0, 100 * D2R, 30 * D2R, 10 * D2R, 0, 100 * D2R]
        dt2 = [30 * D2R, 10 * D2R, 0, 100 * D2R, -30 * D2R, 10 * D2R, 0, 100 * D2R]

        # 执行关节规划
        for i in range(2):
            print("wave round %d" % (i * 2 + 1))
            # for idx in range(12):
            #     self.armCommand.finger[idx] = 50
            self.set_arm_path(T, dt1)
            print("wave round %d" % (i * 2 + 2))
            # for idx in range(12):
            #     self.armCommand.finger[idx] = 5
            self.set_arm_path(T, dt2)
        print("return to zero")
        gBot.set_arm_path(T, dt0)


if __name__ == '__main__':
    gBot = ArmBase()
    gBot.testArm()


