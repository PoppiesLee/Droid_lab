import math
import time
import numpy as np
from tqdm import tqdm
import onnxruntime as ort
from base.LegBase import LegBase
from base.Base import set_joint_mode
from tools.CircularBuffer import CircularBuffer
from tools.load_env_config import load_configuration
from base.Base import NanoSleep, euler_to_quaternion, quat_rotate_inverse

from tools.aoa_ctrl import AoaReader
MAX_LINE_VEL  = 1.5
MAX_ANGLE_VEL = 0.8

onnx_mode_path = f"policies/policy.onnx"
#                           0            1            2              3               4                5               6            7            8              9
IsaacLabJointOrder = ['L_hip_yaw', 'R_hip_yaw', 'L_hip_roll', 'R_hip_roll', 'L_hip_pitch', 'R_hip_pitch', 'L_knee_pitch', 'R_knee_pitch', 'L_ankle_pitch', 'R_ankle_pitch']
MujocoJointOrder   = ['L_hip_yaw', 'L_hip_roll', 'L_hip_pitch', 'L_knee_pitch', 'L_ankle_pitch', 'R_hip_yaw', 'R_hip_roll', 'R_hip_pitch', 'R_knee_pitch', 'R_ankle_pitch']
# 找到 IsaacLabJointOrder 中每个关节在 MujocoJointOrder 中的索引
Mujoco_to_Isaac_indices = [MujocoJointOrder.index(joint) for joint in IsaacLabJointOrder]
# 找到 MujocoJointOrder 中每个关节在 IsaacLabJointOrder 中的索引
Isaac_to_Mujoco_indices = [IsaacLabJointOrder.index(joint) for joint in MujocoJointOrder]
print("Mujoco to IsaacLab indices:", Mujoco_to_Isaac_indices)
print("IsaacLab to Mujoco indices:", Isaac_to_Mujoco_indices)

def get_command(last_value, current_value, max_increment):
    """
    返回一个值，该值满足最大增量限制。

    :param last_value: 上一个值
    :param current_value: 当前值
    :param max_increment: 最大增量
    :return: 限制后的值
    """
    # 计算当前值与上一个值之间的差值
    increment = current_value - last_value
    # 如果差值的绝对值超过了最大增量，则限制它
    if abs(increment) > max_increment:
        # 如果差值为正，则增加最大增量；如果差值为负，则减少最大增量
        return last_value + (max_increment if increment > 0 else -max_increment)
    # 如果差值在允许范围内，则直接返回当前值
    return current_value


class Sim2Real(LegBase):
    def __init__(self):
        LegBase.__init__(self)
        self.num_actions = 10
        self.num_observations = 41
        self.gait_frequency = 0
        self.cfg = load_configuration("policies/env_cfg.json", MujocoJointOrder)
        # self.cfg.default_joints[4] = 0.5
        # self.cfg.default_joints[9] = 0.5
        self.run_flag = True
        self.aoa_reader = AoaReader()
        self.aoa_reader.start_server()
        # joint target
        self.command = [0., 0., 0.]
        self.target_q = np.zeros(self.num_actions, dtype=np.double)
        self.action = np.zeros(self.num_actions, dtype=np.double)
        self.onnx_policy = ort.InferenceSession(onnx_mode_path)
        self.hist_obs = CircularBuffer(self.num_observations, self.cfg.hist_length)
        set_joint_mode(self.legCommand, self.cfg, self.legActions)

    def init_robot(self):
        print("default_joints: ", self.cfg.default_joints)
        self.set_leg_path(1, self.cfg.default_joints[:self.legActions])
        timer = NanoSleep(self.cfg.decimation)  # 创建一个decimation毫秒的NanoSleep对象
        self.get_leg_state()
        temp_tic = self.legState.system_tic
        while self.legState.rc_keys[0] > 64:
            if self.legState.system_tic - temp_tic > 1000:
                temp_tic = self.legState.system_tic
                print("请将CH8急停左滑到底", self.legState.system_tic, self.legState.rc_du)
            start_time = time.perf_counter()
            self.get_leg_state()
            timer.waiting(start_time)

        print("单击CH6开始, CH8右滑到底急停")
        while (self.legState.rc_keys[3] == 0) and (self.run_flag == True):  # CH6
            start_time = time.perf_counter()
            self.get_leg_state()
            if self.legState.rc_keys[0] > 64:
                print("紧急停止！！！")
                exit()
            timer.waiting(start_time)

    def update_rc_command(self):
        if (self.legState.rc_keys[1] == 0) and (self.legState.rc_keys[2] == 0):
            if self.aoa_reader.result_event.is_set():  # 检查是否有新的结果
                self.aoa_reader.result_event.clear()  # 清除事件
                if self.aoa_reader.result['nodes']:
                    distance = self.aoa_reader.result['nodes'][0]['dis'] - 0.5
                    angle = self.aoa_reader.result['nodes'][0]['angle'] * math.pi / 90.0
                    distance = get_command(self.command[0], distance, 0.5)
                    angle = get_command(self.command[2], angle, 0.5)
                    self.command[0] = min(distance, MAX_LINE_VEL)
                    self.command[2] = min(angle, MAX_ANGLE_VEL)
                # print("aoa command: ", self.command)
        else:
            self.command = [self.legState.rc_du[0] * 2, self.legState.rc_du[1] * 0.5, -self.legState.rc_du[3] * 0.8]
            # print("t8s command: ", self.command)


    def get_obs(self, gait_process):
        q = np.array(self.legState.position)
        dq = np.array(self.legState.velocity)

        base_euler = np.array(self.legState.imu_euler)
        base_ang_vel = np.array(self.legState.imu_gyro)

        base_euler[base_euler > math.pi] -= 2 * math.pi
        eq = euler_to_quaternion(base_euler[0], base_euler[1], base_euler[2])
        eq = np.array(eq, dtype=np.double)
        project_gravity = quat_rotate_inverse(eq, np.array([0., 0., -1]))
        self.update_rc_command()
        # 遥控器键值变步频处理
        if abs(self.command[0]) < 0.1 and abs(self.command[1]) < 0.1 and abs(self.command[2]) < 0.1:
            self.gait_frequency = 0
        else:
            # max_abs_command = max(abs(self.command[0]), abs(self.command[1]), abs(self.command[2]))
            self.gait_frequency = 1.5
        obs = np.zeros([self.num_observations], dtype=np.float32)
        obs[0:3] = base_ang_vel * 1.0
        obs[3:6] = project_gravity * 1.0
        obs[6:9] = self.command
        obs[9] = np.cos(2 * np.pi * gait_process) * (self.gait_frequency > 1.0e-8)
        obs[10] = np.sin(2 * np.pi * gait_process) * (self.gait_frequency > 1.0e-8)
        obs[11: 21] = (q- self.cfg.default_joints)[Mujoco_to_Isaac_indices]
        obs[21: 31] = dq[Mujoco_to_Isaac_indices]
        obs[31: 41] = self.action[Mujoco_to_Isaac_indices]
        obs = np.clip(obs, -100, 100)
        return q, dq, obs

    def get_action(self, obs):
        obs = [np.array(obs, dtype=np.float32)]
        action =np.array(self.onnx_policy.run(None, {"obs": obs})[0].tolist()[0])
        self.action = np.clip(action[Isaac_to_Mujoco_indices], -100.0,100.0)
        return self.action * self.cfg.action_scale + self.cfg.default_joints

    def run(self):
        pre_tic = 0
        gait_process = 0
        duration_second = self.cfg.decimation * self.cfg.dt  # 单位:s
        duration_millisecond = duration_second * 1000  # 单位：ms
        timer = NanoSleep(duration_millisecond)  # 创建一个decimation毫秒的NanoSleep对象
        pbar = tqdm(range(int(0xfffffff0 / duration_second)),
                    desc="x02 running...")  # x * 0.001, ms -> s
        start = time.perf_counter()
        for _ in pbar:
            start_time = time.perf_counter()
            self.get_leg_state()
            if self.legState.rc_keys[0] > 64:
                print("紧急停止！！！")
                exit()
            q, dq, obs = self.get_obs(gait_process)
            self.hist_obs.append(obs)
            self.target_q = self.get_action(self.hist_obs.get())
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
