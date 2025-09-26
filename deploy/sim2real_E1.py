import math
import time
import numpy as np
from tqdm import tqdm
import onnxruntime as ort
from base.LegBase import LegBase
from base.Base import get_command
from base.Base import set_joint_mode
from base.Base import set_joint_mode_E1
from tools.Gamepad import GamepadHandler
from tools.CircularBuffer import CircularBuffer
from tools.load_env_config import load_configuration
from base.Base import NanoSleep, euler_to_quaternion, quat_rotate_inverse

onnx_mode_path = f"policies/policy.onnx"

IsaacLabJointOrder = ['left_hip_pitch_joint', 'right_hip_pitch_joint', 'left_hip_roll_joint', 'right_hip_roll_joint', 'left_hip_yaw_joint', 'right_hip_yaw_joint', 'left_knee_joint', 'right_knee_joint', 'left_ankle_roll_joint', 'right_ankle_roll_joint', 'left_ankle_pitch_joint', 'right_ankle_pitch_joint']
MujocoJointOrder = ['left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint', 'left_ankle_roll_joint', 'left_ankle_pitch_joint', 'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint', 'right_ankle_roll_joint', 'right_ankle_pitch_joint']
RealJointOrder = ['left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint', 'left_ankle_roll_joint', 'left_ankle_pitch_joint', 'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint', 'right_ankle_roll_joint', 'right_ankle_pitch_joint']

Mujoco_to_Isaac_indices = [MujocoJointOrder.index(joint) for joint in IsaacLabJointOrder]
Isaac_to_Real_indices = [IsaacLabJointOrder.index(joint) for joint in RealJointOrder]
Isaac_to_Mujoco_indices = [IsaacLabJointOrder.index(joint) for joint in MujocoJointOrder]
Real_to_Isaac_indices = [RealJointOrder.index(joint) for joint in IsaacLabJointOrder]
# Isaac_to_Real_indices = []
# for joint in IsaacLabJointOrder:
#     if joint == "left_knee_joint":
#         Isaac_to_Real_indices.extend([RealJointOrder.index("left_knee_joint_motor1"),
#                                       RealJointOrder.index("left_knee_joint_motor2")])
#     elif joint == "right_knee_joint":
#         Isaac_to_Real_indices.extend([RealJointOrder.index("right_knee_joint_motor1"),
#                                       RealJointOrder.index("right_knee_joint_motor2")])
#     else:
#         Isaac_to_Real_indices.append(RealJointOrder.index(joint))
#
# # 反向映射：真实机器人 -> Isaac
# Real_to_Isaac_indices = []
# for joint in RealJointOrder:
#     if "knee_joint_motor" in joint:  # 两个电机都对应 Isaac 的一个膝关节
#         if "left" in joint:
#             Real_to_Isaac_indices.append(IsaacLabJointOrder.index("left_knee_joint"))
#         else:
#             Real_to_Isaac_indices.append(IsaacLabJointOrder.index("right_knee_joint"))
#     else:
#         Real_to_Isaac_indices.append(IsaacLabJointOrder.index(joint))

class Sim2Real(LegBase):
    def __init__(self):
        LegBase.__init__(self)
        self.num_actions = 12
        self.num_observations = 47
        self.gait_frequency = 0
        self.cfg = load_configuration("policies/env_cfg.json", RealJointOrder)
        self.run_flag = True
        # joint target
        self.command = [0., 0., 0.]
        self.target_q = np.zeros(self.num_actions, dtype=np.double)
        self.action = np.zeros(self.num_actions, dtype=np.double)
        self.onnx_policy = ort.InferenceSession(onnx_mode_path)
        self.hist_obs = CircularBuffer(self.num_observations, self.cfg.hist_length)

        set_joint_mode_E1(self.legCommand, self.cfg, 14)
        self.rc = GamepadHandler()

    def init_robot(self):
        print("default_joints: ", self.cfg.default_joints)
        motor_targets = self.joints_to_motors(self.cfg.default_joints)
        self.set_leg_path(1, motor_targets)
        # self.set_leg_path(1, self.cfg.default_joints[:self.legActions])
        timer = NanoSleep(self.cfg.decimation)  # 创建一个decimation毫秒的NanoSleep对象
        print("单击三开始, LT按压到底到底急停")
        while (self.rc.state.START == False) and (self.run_flag == True):  # CH6
            start_time = time.perf_counter()
            self.get_leg_state()
            if self.rc.state.LT > 64:
                print("紧急停止！！！")
                exit()
            timer.waiting(start_time)

    def update_rc_command(self):
        self.command[0] = get_command(self.command[0], self.rc.state.LEFT_Y  * 0.5, 0.01)
        self.command[1] = get_command(self.command[1], self.rc.state.LEFT_X  * 0.5, 0.01)
        self.command[2] = get_command(self.command[2], self.rc.state.RIGHT_X * 0.5, 0.01)
        print(self.command)
        # 遥控器键值变步频处理
        if abs(self.command[0]) < 0.1 and abs(self.command[1]) < 0.1 and abs(self.command[2]) < 0.1:
            self.gait_frequency = 0
        else:
            self.gait_frequency = 1.0

    def motors_to_joints(self, motor_pos: np.ndarray, motor_vel: np.ndarray):
        joint_pos = np.zeros(12)
        joint_vel = np.zeros(12)

        # 左腿
        joint_pos[0:3] = motor_pos[0:3]
        joint_pos[3] = 0.5 * (motor_pos[3] + motor_pos[4])  # 左膝
        joint_pos[4:6] = motor_pos[5:7]

        joint_vel[0:3] = motor_vel[0:3]
        joint_vel[3] = 0.5 * (motor_vel[3] + motor_vel[4])
        joint_vel[4:6] = motor_vel[5:7]

        # 右腿
        joint_pos[6:9] = motor_pos[7:10]
        joint_pos[9] = 0.5 * (motor_pos[10] + motor_pos[11])  # 右膝
        joint_pos[10:12] = motor_pos[12:14]

        joint_vel[6:9] = motor_vel[7:10]
        joint_vel[9] = 0.5 * (motor_vel[10] + motor_vel[11])
        joint_vel[10:12] = motor_vel[12:14]
        return joint_pos, joint_vel

    def joints_to_motors(self, joint_pos: np.ndarray):
        motor_pos = np.zeros(14)
        motor_vel = np.zeros(14)

        # 左腿
        motor_pos[0:3] = joint_pos[0:3]
        motor_pos[3] = joint_pos[3]  # 左膝 - 电机1
        motor_pos[4] = joint_pos[3]  # 左膝 - 电机2
        motor_pos[5:7] = joint_pos[4:6]

        # motor_vel[0:3] = joint_vel[0:3]
        # motor_vel[3] = joint_vel[3]
        # motor_vel[4] = joint_vel[3]
        # motor_vel[5:7] = joint_vel[4:6]

        # 右腿
        motor_pos[7:10] = joint_pos[6:9]
        motor_pos[10] = joint_pos[9]  # 右膝 - 电机1
        motor_pos[11] = joint_pos[9]  # 右膝 - 电机2
        motor_pos[12:14] = joint_pos[10:12]

        # motor_vel[7:10] = joint_vel[6:9]
        # motor_vel[10] = joint_vel[9]
        # motor_vel[11] = joint_vel[9]
        # motor_vel[12:14] = joint_vel[10:12]
        return motor_pos

    def get_obs(self, gait_process):
        q_raw = np.array(self.legState.position)
        dq_raw = np.array(self.legState.velocity)
        q, dq = self.motors_to_joints(q_raw, dq_raw)

        base_euler = np.array(self.legState.imu_euler)
        base_ang_vel = np.array(self.legState.imu_gyro)

        base_euler[base_euler > math.pi] -= 2 * math.pi
        eq = euler_to_quaternion(base_euler[0], base_euler[1], base_euler[2])
        eq = np.array(eq, dtype=np.double)
        project_gravity = quat_rotate_inverse(eq, np.array([0., 0., -1]))
        self.update_rc_command()

        obs = np.zeros([self.num_observations], dtype=np.float32)
        obs[0:3] = base_ang_vel
        obs[3:6] = project_gravity
        obs[6:9] = self.command
        obs[9] = np.cos(2 * np.pi * gait_process) * (self.gait_frequency > 1.0e-8)
        obs[10] = np.sin(2 * np.pi * gait_process) * (self.gait_frequency > 1.0e-8)
        obs[11: 23] = (q- self.cfg.default_joints)[Real_to_Isaac_indices]
        obs[23: 35] = dq[Real_to_Isaac_indices]
        obs[35: 47] = self.action[Real_to_Isaac_indices]
        obs = np.clip(obs, -100, 100)
        return q, dq, obs

    def get_action(self, obs):
        obs = [np.array(obs, dtype=np.float32)]
        action =np.array(self.onnx_policy.run(None, {"obs": obs})[0].tolist()[0])
        self.action = np.clip(action[Isaac_to_Real_indices], -100.0,100.0)
        return self.action * self.cfg.action_scale + self.cfg.default_joints

    def run(self):
        pre_tic = 0
        gait_process = 0
        motor_targets = np.zeros(14)
        duration_second = self.cfg.decimation * self.cfg.dt  # 单位:s
        duration_millisecond = duration_second * 1000  # 单位：ms
        timer = NanoSleep(duration_millisecond)  # 创建一个decimation毫秒的NanoSleep对象
        pbar = tqdm(range(int(0xfffffff0 / duration_second)),
                    desc="X03 running...")  # x * 0.001, ms -> s
        start = time.perf_counter()
        for _ in pbar:
            start_time = time.perf_counter()
            self.get_leg_state()
            if self.rc.state.LT > 64:
                print("紧急停止！！！")
                exit()
            q, dq, obs = self.get_obs(gait_process)
            self.hist_obs.append(obs)
            self.target_q = self.get_action(self.hist_obs.get())
            motor_targets = self.joints_to_motors(self.target_q)
            for idx in range(self.legActions):
                # self.legCommand.position[idx] = self.target_q[idx]
                self.legCommand.position[idx] = motor_targets[idx]
            self.set_leg_command()
            pbar.set_postfix(
                realCycle=f"{self.legState.system_tic - pre_tic}ms",  # 实际循环周期，单位毫秒
                calculateTime=f"{(time.perf_counter() - start_time) * 1000:.3f}ms",  # 计算用时，单位毫秒
                runTime=f"{(time.perf_counter() - start):.3f}s"  # 运行时间，单位秒
            )
            pre_tic = self.legState.system_tic
            gait_process = np.fmod(gait_process + duration_second * self.gait_frequency, 1.0)
            timer.waiting(start_time)
        self.set_leg_path(1, self.cfg.default_joints)


if __name__ == '__main__':
    mybot = Sim2Real()
    mybot.init_robot()   # 屈膝状态
    time.sleep(1)
    mybot.run()
