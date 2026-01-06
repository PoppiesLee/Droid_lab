import math
import time
import numpy as np
from tqdm import tqdm
import onnxruntime as ort
from base.LegBase import LegBase
from base.Base import get_command
from base.Base import set_joint_mode
from tools.Gamepad import GamepadHandler
from tools.CircularBuffer import CircularBuffer
from tools.load_env_config import load_configuration
from base.Base import NanoSleep, euler_to_quaternion, quat_rotate_inverse
from scipy.spatial.transform import Rotation as R

onnx_mode_path = f"policies/policy.onnx"

IsaacLabJointOrder = ['left_hip_pitch_joint', 'right_hip_pitch_joint', 'left_hip_roll_joint', 'right_hip_roll_joint', 'left_hip_yaw_joint', 'right_hip_yaw_joint', 'left_knee_joint', 'right_knee_joint', 'left_ankle_pitch_joint', 'right_ankle_pitch_joint', 'left_ankle_roll_joint', 'right_ankle_roll_joint']
RealJointOrder = ['left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint', 'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint']
# 找到 IsaacLabJointOrder 中每个关节在 MujocoJointOrder 中的索引-30*D2R,  0*D2R,  0*D2R,  60*D2R, -30*D2R,  0*D2R
# Mujoco_to_Isaac_indices = [MujocoJointOrder.index(joint) for joint in IsaacLabJointOrder]
Isaac_to_Real_indices = [IsaacLabJointOrder.index(joint) for joint in RealJointOrder]
# 找到 MujocoJointOrder 中每个关节在 IsaacLabJointOrder 中的索引
# Isaac_to_Mujoco_indices = [IsaacLabJointOrder.index(joint) for joint in MujocoJointOrder]
Real_to_Isaac_indices = [RealJointOrder.index(joint) for joint in IsaacLabJointOrder]

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
        set_joint_mode(self.legCommand, self.cfg, self.legActions)
        self.rc = GamepadHandler()

    def init_robot(self):
        print("default_joints: ", self.cfg.default_joints)
        self.set_leg_path(1, self.cfg.default_joints[:self.legActions])
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
        self.command[0] = get_command(self.command[0], self.rc.state.LEFT_Y   * 0.5, 0.01)
        self.command[1] = get_command(self.command[1], self.rc.state.RIGHT_X  * 0.5, 0.01)
        self.command[2] = get_command(self.command[2], self.rc.state.LEFT_X   * 0.5, 0.01)
        # 遥控器键值变步频处理
        if abs(self.command[0]) < 0.1 and abs(self.command[1]) < 0.1 and abs(self.command[2]) < 0.1:
            self.gait_frequency = 0
        else:
            self.gait_frequency = 1.5

    def get_gravity_orientation_from_rpy(self, roll, pitch):
        rot = R.from_euler('xy', [roll, pitch])
        g_world = np.array([0, 0, -1])
        g_local = rot.inv().apply(g_world)
        return g_local

    def get_obs(self, gait_process):
        q = np.array(self.legState.position[:12])
        dq = np.array(self.legState.velocity[:12])

        base_euler = np.array(self.legState.imu_euler)
        base_ang_vel = np.array(self.legState.imu_gyro)

        base_euler[base_euler > math.pi] -= 2 * math.pi
        eq = euler_to_quaternion(base_euler[0], base_euler[1], base_euler[2])
        # eq[1] =eq[1] + 0.05
        eq = np.array(eq, dtype=np.double)
        project_gravity = quat_rotate_inverse(eq, np.array([0., 0., -1]))
        # project_gravity =  self.get_gravity_orientation_from_rpy(base_euler[0], base_euler[1])
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
        return self.action * self.cfg.action_scale  + self.cfg.default_joints

    def run(self):
        pre_tic = 0
        gait_process = 0
        duration_second = self.cfg.decimation * self.cfg.dt  # 单位:s
        duration_millisecond = duration_second * 1000  # 单位：ms
        timer = NanoSleep(duration_millisecond)  # 创建一个decimation毫秒的NanoSleep对象
        pbar = tqdm(range(int(0xfffffff0 / duration_second)),
                    desc="E1 running...")  # x * 0.001, ms -> s
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
            self.target_q = np.append(self.target_q, 0)
            for idx in range(self.legActions):
                self.legCommand.position[idx] = self.target_q[idx]
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
