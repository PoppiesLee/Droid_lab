import math
import copy
import torch
import mujoco
import mujoco_viewer
import numpy as np
from tqdm import tqdm
import onnxruntime as ort
from tools.load_env_config import load_configuration
from deploy.tools.CircularBuffer import CircularBuffer

onnx_mode_path = f"policies/policy.onnx"
mujoco_model_path = f"../legged_lab/assets/droid/x02a/scene.xml"

#                           0            1              2              3               4                5               6               7               8                 9                10             11
IsaacLabJointOrder = ['L_hip_yaw',   'R_hip_yaw',  'L_hip_roll',   'R_hip_roll',   'L_hip_pitch',  'R_hip_pitch', 'L_knee_pitch', 'R_knee_pitch', 'L_ankle_pitch', 'R_ankle_pitch',  'L_ankle_roll', 'R_ankle_roll']
MujocoJointOrder =   ['L_hip_yaw',  'L_hip_roll', 'L_hip_pitch', 'L_knee_pitch', 'L_ankle_pitch', 'L_ankle_roll',    'R_hip_yaw',   'R_hip_roll',   'R_hip_pitch',  'R_knee_pitch', 'R_ankle_pitch', 'R_ankle_roll']
# IsaacLab to Mujoco indices: [0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11]
# Mujoco to IsaacLab indices: [0, 2, 4, 6, 8, 10, 1, 3, 5, 7, 9, 11]
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

class Sim2Droid:
    def __init__(self, ):
        self.num_actions = 12
        self.num_observations = 45
        # joint target
        self.target_q = np.zeros(self.num_actions, dtype=np.double)
        self.target_dq = np.zeros(self.num_actions, dtype=np.double)

        self.action = np.zeros(self.num_actions, dtype=np.double)

        self.onnx_policy = ort.InferenceSession(onnx_mode_path)
        self.model = mujoco.MjModel.from_xml_path(filename=mujoco_model_path)
        actuators = self.get_joint_names()
        self.cfg = load_configuration("policies/env_cfg.json", actuators)
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

    def get_obs(self, _cnt_pd_loop):
        q = self.data.qpos.astype(np.float32)[7:]
        dq = self.data.qvel.astype(np.float32)[6:]
        ang_vel = self.data.sensor('gyro').data.astype(np.float32)
        quat = self.data.sensor('bq').data[[1, 2, 3, 0]].astype(np.float32)
        proj_grav = quat_to_grav(quat, [0, 0, -1])
        # euler = quaternion_to_euler_array(quat)
        # euler[euler > math.pi] -= 2 * math.pi

        obs = np.zeros(self.num_observations, dtype=np.float32)
        obs[0:3] = ang_vel
        obs[3:6] = proj_grav
        obs[6] = 0.5
        obs[7] = 0.
        obs[8] = 0.
        obs[9: 21] = q[Mujoco_to_Isaac_indices] - self.cfg.default_joints[Mujoco_to_Isaac_indices]
        obs[21: 33] = dq[Mujoco_to_Isaac_indices]
        obs[33: 45] = self.action[Mujoco_to_Isaac_indices]
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
        obs = np.array(obs, dtype=np.float32)
        obs = np.expand_dims(obs, axis=0)
        action =np.array(self.onnx_policy.run(None, {"obs": obs})[0].tolist()[0])
        self.action = np.clip(action[Isaac_to_Mujoco_indices], -100.0,100.0)
        return self.action * self.cfg.action_scale + self.cfg.default_joints

    def run(self):
        self.cnt_pd_loop = 0
        duration_second = self.cfg.decimation * self.cfg.dt  # 单位:s
        for _ in tqdm(range(int(50 / duration_second)), desc="Simulating..."):
            # Obtain an observation
            q, dq, obs = self.get_obs(self.cnt_pd_loop)
            # 1000hz -> 100hz
            if self.cnt_pd_loop % self.cfg.decimation == 0:
                self.hist_obs.append(obs)
                self.target_q = self.get_action(self.hist_obs.get())
            # Generate PD control
            self.pd_control(self.target_q, q, dq)
            self.cnt_pd_loop += 1
        self.viewer.close()

    def init_robot(self):
        final_goal = self.cfg.default_joints[:]
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
        # while (self.keys[3] == 0) and (self.run_flag == True):
        #     self.pc = self.data.qpos.astype(np.double)[-self.cfg.env.num_actions:]
        #     self.vc = self.data.qvel.astype(np.double)[-self.cfg.env.num_actions:]
        #     # Generate PD control
        #     tau = self.pd_control(self.target_q, self.pc, self.target_dq, self.vc)  # Calc torques
        #     self.set_sim_target(tau)
        # self.set_real_stop()

if __name__ == '__main__':
    mybot = Sim2Droid()
    # mybot.init_robot()
    # mybot.spin()
    print("start main run")
    mybot.run()
