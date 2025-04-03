import os
import copy
import mujoco
import mujoco_viewer
import numpy as np
from tqdm import tqdm
import onnxruntime as ort
from deploy import DEPLOY_FOLDER_DIR
from tools.load_env_config import load_configuration
from tools.CircularBuffer import CircularBuffer


onnx_mode_path = os.path.join(DEPLOY_FOLDER_DIR, f"policies/policy.onnx")
env_config_path = os.path.join(DEPLOY_FOLDER_DIR, f"policies/env_cfg.json")
mujoco_model_path = os.path.join(DEPLOY_FOLDER_DIR, f"../legged_lab/assets/droid/x2/scene.xml")

MAX_LINE_VEL  = 1.5
MAX_ANGLE_VEL = 0.5

#                           0            1            2              3               4                5               6            7            8              9
IsaacLabJointOrder = ['L_hip_yaw', 'R_hip_yaw', 'L_hip_roll', 'R_hip_roll', 'L_hip_pitch', 'R_hip_pitch', 'L_knee_pitch', 'R_knee_pitch', 'L_ankle_pitch', 'R_ankle_pitch']
MujocoJointOrder   = ['L_hip_yaw', 'L_hip_roll', 'L_hip_pitch', 'L_knee_pitch', 'L_ankle_pitch', 'R_hip_yaw', 'R_hip_roll', 'R_hip_pitch', 'R_knee_pitch', 'R_ankle_pitch']
# 找到 IsaacLabJointOrder 中每个关节在 MujocoJointOrder 中的索引
Mujoco_to_Isaac_indices = [MujocoJointOrder.index(joint) for joint in IsaacLabJointOrder]
# 找到 MujocoJointOrder 中每个关节在 IsaacLabJointOrder 中的索引
Isaac_to_Mujoco_indices = [IsaacLabJointOrder.index(joint) for joint in MujocoJointOrder]
print("Mujoco to IsaacLab indices:", Mujoco_to_Isaac_indices)
print("IsaacLab to Mujoco indices:", Isaac_to_Mujoco_indices)


def quat_to_grav(q, v):
    shape = q.shape
    q_w = q[-1]
    q_vec = q[:3]
    # a = v * (2.0 * q_w ** 2 - 1.0).unsqueeze(-1)
    a = v * np.expand_dims(2.0 * q_w ** 2 - 1.0, axis=-1)
    # b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    b = np.cross(q_vec, v) * np.expand_dims(q_w, axis=-1) * 2.0
    # c = q_vec * torch.bmm(q_vec.view(shape[0], 1, 3), v.view(shape[0], 3, 1)).squeeze(-1) * 2.0
    c = q_vec * np.expand_dims(np.sum(q_vec * v, axis=-1), axis=-1) * 2.0
    return a - b + c


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


class Sim2Mujo():
    def __init__(self):
        self.gait_frequency = 0
        self.num_actions = 10
        self.num_observations = 41
        # joint target
        self.command = [0., 0., 0.]
        self.target_q = np.zeros(self.num_actions, dtype=np.double)
        self.action = np.zeros(self.num_actions, dtype=np.double)

        self.onnx_policy = ort.InferenceSession(onnx_mode_path)
        self.model = mujoco.MjModel.from_xml_path(filename=mujoco_model_path)
        actuators = self.get_joint_names()
        self.cfg = load_configuration(env_config_path, actuators)
        self.hist_obs = CircularBuffer(self.num_observations, self.cfg.hist_length)
        self.model.opt.timestep = self.cfg.dt
        self.data = mujoco.MjData(self.model)
        mujoco.mj_step(self.model, self.data)
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data, width=1500,height=1500)
        self.cnt_pd_loop = 0

    def get_joint_names(self):
        actuators = []
        for i in range(0, self.model.nu):
            joint_id = self.model.actuator_trnid[i]  # 获取关节 ID
            _name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id[0])  # 获取关节名称
            actuators.append(_name)
        print("\nMujoco actuators order:\n",actuators, "\n")
        return actuators

    def get_obs(self, gait_process):
        q = self.data.qpos.astype(np.float32)[7:]
        dq = self.data.qvel.astype(np.float32)[6:]
        ang_vel = self.data.qvel[3:6].astype(np.float32)
        quat = self.data.qpos[3:7].astype(np.float32)
        quat[:] = quat[[1, 2, 3, 0]]
        proj_grav = quat_to_grav(quat, [0, 0, -1])
        # euler = quaternion_to_euler_array(quat)
        # euler[euler > math.pi] -= 2 * math.pi
        self.command = [0., 0., 0.]
        # 遥控器键值变步频处理
        if abs(self.command[0]) < 0.1 and abs(self.command[1]) < 0.1 and abs(self.command[2]) < 0.1:
            self.gait_frequency = 0
        else:
            max_abs_command = max(abs(self.command[0]), abs(self.command[1]), abs(self.command[2]))
            self.gait_frequency = 1.5
        obs = np.zeros(self.num_observations, dtype=np.float32)
        obs[0:3] = ang_vel
        obs[3:6] = proj_grav
        obs[6:9] = self.command
        obs[9] = np.cos(2 * np.pi * gait_process) * (self.gait_frequency > 1.0e-8)
        obs[10] = np.sin(2 * np.pi * gait_process) * (self.gait_frequency > 1.0e-8)
        obs[11: 21] = (q- self.cfg.default_joints)[Mujoco_to_Isaac_indices]
        obs[21: 31] = dq[Mujoco_to_Isaac_indices]
        obs[31: 41] = self.action[Mujoco_to_Isaac_indices]
        obs = np.clip(obs, -100, 100)
        return q, dq, obs

    def pd_control(self, target_q, q, dq):  # mujoco关节顺序输入输出
        self.data.ctrl = np.clip(
            self.cfg.dof_stiffness * (target_q - q) - self.cfg.dof_damping * dq,
            -self.cfg.effort_limit, self.cfg.effort_limit)  # Clamp torques
        mujoco.mj_step(self.model, self.data)
        self.viewer.cam.lookat[:] = self.data.qpos.astype(np.float32)[0:3]
        self.viewer.render()

    def get_action(self, obs):
        obs = [np.array(obs, dtype=np.float32)]
        action =np.array(self.onnx_policy.run(None, {"obs": obs})[0].tolist()[0])
        self.action = np.clip(action[Isaac_to_Mujoco_indices], -100.0,100.0)
        return self.action * self.cfg.action_scale + self.cfg.default_joints

    def run(self):
        gait_process = 0
        self.cnt_pd_loop = 0
        duration_second = self.cfg.decimation * self.cfg.dt  # 单位:s
        for _ in tqdm(range(int(0xfffffff0 / duration_second)), desc="Simulating..."):
            # Obtain an observation

            # 1000hz -> 100hz
            if self.cnt_pd_loop % self.cfg.decimation == 0:
                q, dq, obs = self.get_obs(gait_process)
                self.hist_obs.append(obs)
                self.target_q = self.get_action(self.hist_obs.get())
                gait_process = np.fmod(gait_process + duration_second * self.gait_frequency, 1.0)
            # Generate PD control
            self.pd_control(self.target_q, q, dq)
            self.cnt_pd_loop += 1
        self.viewer.close()


    def init_robot(self):
        print("default_joints: ", self.cfg.default_joints)
        final_goal = self.cfg.default_joints
        target_sequence = []
        target = self.data.qpos.astype(np.double)[-self.num_actions:]

        while np.max(np.abs(target - final_goal)) > 0.01:
            target -= np.clip((target - final_goal), -0.01, 0.01)
            target_sequence += [copy.deepcopy(target)]
        self.cnt_pd_loop = 0
        while self.cnt_pd_loop < len(target_sequence):
            self.target_q = target_sequence[self.cnt_pd_loop]
            for i in range(self.cfg.decimation):
                pc = self.data.qpos.astype(np.double)[7:]
                vc = self.data.qvel.astype(np.double)[6:]
                # Generate PD control
                self.pd_control(self.target_q, pc, vc)  # Calc torques
            self.cnt_pd_loop += 1
        print("Initializing robot default pos ok")


if __name__ == '__main__':
    mybot = Sim2Mujo()
    mybot.init_robot()
    print("start main run")
    try:
        while True:
            mybot.run()
    except KeyboardInterrupt:
        print("\n用户中断，停止程序")
    finally:
        print("\n停止程序")
