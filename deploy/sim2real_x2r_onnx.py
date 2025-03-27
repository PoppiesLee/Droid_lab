import math
import time
import numpy as np
from tqdm import tqdm
import onnxruntime as ort
from base.Base import NanoSleep
from base.LegBase import LegBase
from base.Base import set_joint_mode
from tools.load_env_config import load_configuration
from tools.CircularBuffer import CircularBuffer

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

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to a quaternion (w, x, y, z).
    :param roll: Rotation around X-axis (in radians)
    :param pitch: Rotation around Y-axis (in radians)
    :param yaw: Rotation around Z-axis (in radians)
    :return: Quaternion as a tuple (w, x, y, z)
    """
    # Compute half angles
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    # Calculate quaternion components
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [w, x, y, z]

def quat_rotate_inverse(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate a vector by the inverse of a quaternion along the last dimension of q and v.

    Args:
        q: The quaternion in (w, x, y, z). Shape is (4,).
        v: The vector in (x, y, z). Shape is (3,).

    Returns:
        The rotated vector in (x, y, z). Shape is (3,).
    """
    v = np.array(v)
    q_w = q[0]
    q_vec = q[1:]
    a = v * (2.0 * q_w ** 2 - 1.0)
    b = np.cross(q_vec, v) * q_w * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    return a - b + c


class Sim2Real(LegBase):
    def __init__(self):
        LegBase.__init__(self)
        self.num_actions = 10
        self.num_observations = 41
        self.gait_frequency = 0
        self.cfg = load_configuration("policies/env_cfg.json", MujocoJointOrder)
        self.run_flag = True
        # joint target
        self.command = [0., 0., 0.]
        self.target_q = np.zeros(self.num_actions, dtype=np.double)
        self.action = np.zeros(self.num_actions, dtype=np.double)
        self.onnx_policy = ort.InferenceSession(onnx_mode_path)
        self.hist_obs = CircularBuffer(self.num_observations, self.cfg.hist_length)
        set_joint_mode(self.legCommand, self.cfg, self.legActions)

    def init_robot(self):
        self.set_leg_path(1, self.cfg.default_joints[:self.legActions])
        timer = NanoSleep(self.cfg.decimation)  # 创建一个decimation毫秒的NanoSleep对象
        self.get_leg_state()
        temp_tic = self.legState.system_tic
        while self.legState.rc_keys[0] > 64:
            if self.legState.system_tic - temp_tic > 1000:
                temp_tic = self.legState.system_tic
                print("请将CH8急停左滑到底", self.legState.system_tic)
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

    def get_obs(self, gait_process):
        q = np.array(self.legState.position)
        dq = np.array(self.legState.velocity)

        base_euler = np.array(self.legState.imu_euler)
        base_ang_vel = np.array(self.legState.imu_gyro)

        base_euler[base_euler > math.pi] -= 2 * math.pi
        eq = euler_to_quaternion(base_euler[0], base_euler[1], base_euler[2])
        eq = np.array(eq, dtype=np.double)
        project_gravity = quat_rotate_inverse(eq, np.array([0., 0., -1]))
        self.command = [self.legState.rc_du[0] * 2, self.legState.rc_du[1] * 0.5, -self.legState.rc_du[3] * 0.5]
        # 遥控器键值变步频处理
        if abs(self.command[0]) < 0.1 and abs(self.command[1]) < 0.1 and abs(self.command[2]) < 0.1:
            self.gait_frequency = 0
        else:
            max_abs_command = max(abs(self.command[0]), abs(self.command[1]), abs(self.command[2]))
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
        pbar = tqdm(range(int(500 / duration_second)),
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
