import math
import copy
import torch
import mujoco
import mujoco_viewer
import numpy as np
from pynput import keyboard
from tqdm import tqdm
import onnxruntime as ort
from tools.aoa_ctrl import AoaReader
from tools.load_env_config import load_configuration
from deploy.tools.CircularBuffer import CircularBuffer
from tools.data_var import Q, data_dict, QKey_list
from datetime import datetime
import pickle


onnx_mode_path = f"policies/policy.onnx"
mujoco_model_path = f"../legged_lab/assets/droid/E1/scene.xml"
MAX_LINE_VEL  = 2.0
MAX_ANGLE_VEL = 0.5

#                           0            1            2              3               4                5               6            7            8              9
IsaacLabJointOrder = ['left_hip_pitch_joint', 'right_hip_pitch_joint', 'left_hip_roll_joint', 'right_hip_roll_joint', 'left_hip_yaw_joint', 'right_hip_yaw_joint', 'left_knee_joint', 'right_knee_joint', 'left_ankle_pitch_joint', 'right_ankle_pitch_joint', 'left_ankle_roll_joint', 'right_ankle_roll_joint']
MujocoJointOrder = ['left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint', 'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint']
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

class Sim2Mujo:
    def __init__(self):
        self.gait_frequency = 0
        self.num_actions = 12
        self.num_observations = 47
        self.aoa_reader = AoaReader()
        self.aoa_reader.start_server()
        # joint target
        self.command = [0., 0., 0.]
        self.target_q = np.zeros(self.num_actions, dtype=np.double)
        self.action = np.zeros(self.num_actions, dtype=np.double)

        self.onnx_policy = ort.InferenceSession(onnx_mode_path)
        self.model = mujoco.MjModel.from_xml_path(filename=mujoco_model_path)
        actuators = self.get_joint_names()
        self.cfg = load_configuration("policies/env_cfg.json", actuators)
        self.hist_obs = CircularBuffer(self.num_observations, self.cfg.hist_length)
        self.model.opt.timestep = self.cfg.dt
        self.data = mujoco.MjData(self.model)
        mujoco.mj_step(self.model, self.data)
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data, width=1500, height=1500)
        self.cnt_pd_loop = 0

        # velocity target
        self.command = [0., 0., 0.]
        self.last_command = [0., 0., 0.]  # 用于增量限制
        # 启动键盘监听线程
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )
        self.listener.start()


    def _on_press(self, key):
        """处理按键按下"""
        try:
            if key.char == '8':
                self.command[0] = MAX_LINE_VEL    # 前进
            elif key.char == '2':
                self.command[0] = -MAX_LINE_VEL   # 后退
            elif key.char == '4':
                self.command[1] = MAX_LINE_VEL    # 左移（可选）
            elif key.char == '6':
                self.command[1] = -MAX_LINE_VEL   # 右移（可选）
            elif key.char == '7':
                self.command[2] = MAX_ANGLE_VEL   # 左转
            elif key.char == '9':
                self.command[2] = -MAX_ANGLE_VEL  # 右转
        except AttributeError:
            pass

        # 应用增量限制
        for i in range(3):
            self.command[i] = get_command(
                self.last_command[i],
                self.command[i],
                max_increment=0.1
            )
        self.last_command = copy.deepcopy(self.command)

    def _on_release(self, key):
        """处理按键释放"""
        try:
            if key.char in ['0']:
                self.command = [0.0, 0.0, 0.0]
                self.last_command = [0.0, 0.0, 0.0]
        except AttributeError:
            pass

        # 按 ESC 退出
        if key == keyboard.Key.esc:
            self.listener.stop()
            self.viewer.close()
            exit(0)

    def get_joint_names(self):
        actuators = []
        for i in range(0, self.model.nu):
            joint_id = self.model.actuator_trnid[i]  # 获取关节 ID
            _name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id[0])  # 获取关节名称
            actuators.append(_name)
        print("\nMujoco actuators order:\n", actuators, "\n")
        return actuators

    def get_obs(self, gait_process):
        q = self.data.qpos.astype(np.float32)[7:]
        dq = self.data.qvel.astype(np.float32)[6:]
        # ang_vel = self.data.qvel[3:6].astype(np.float32)
        # quat = self.data.qpos[3:7].astype(np.float32)
        ang_vel = self.data.sensor('gyro').data.astype(np.float32)
        quat = self.data.sensor('bq').data[[1, 2, 3, 0]].astype(np.float32)
        # quat[:] = quat[[1, 2, 3, 0]]
        proj_grav = quat_to_grav(quat, [0, 0, -1])
        if self.aoa_reader.result_event.is_set():  # 检查是否有新的结果
            self.aoa_reader.result_event.clear()  # 清除事件
            if self.aoa_reader.result['nodes']:
                distance = self.aoa_reader.result['nodes'][0]['dis'] - 0.5
                angle = -self.aoa_reader.result['nodes'][0]['angle'] * math.pi / 180.0
                distance = get_command(self.command[0], distance, 0.01)
                angle = get_command(self.command[2], angle, 0.01)
                self.command[0] = min(distance, MAX_LINE_VEL)
                self.command[2] = min(angle, MAX_ANGLE_VEL)
        command = self.command
        if abs(self.command[0]) < 0.1 and abs(self.command[1]) < 0.1 and abs(self.command[2]) < 0.1:
            self.gait_frequency = 0
        else:
            max_abs_command = max(abs(self.command[0]), abs(self.command[1]), abs(self.command[2]))
            self.gait_frequency = 1.5
        obs = np.zeros(self.num_observations, dtype=np.float32)
        obs[0:3] = ang_vel
        obs[3:6] = proj_grav
        obs[6:9] = command
        obs[9] = np.cos(2 * np.pi * gait_process) * (self.gait_frequency > 1.0e-8)
        obs[10] = np.sin(2 * np.pi * gait_process) * (self.gait_frequency > 1.0e-8)
        obs[11: 23] = (q - self.cfg.default_joints)[Mujoco_to_Isaac_indices]
        obs[23: 35] = dq[Mujoco_to_Isaac_indices]
        obs[35: 47] = self.action[Mujoco_to_Isaac_indices]
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
        action = np.array(self.onnx_policy.run(None, {"obs": obs})[0].tolist()[0])
        self.action = np.clip(action[Isaac_to_Mujoco_indices], -100.0, 100.0)
        return self.action * self.cfg.action_scale + self.cfg.default_joints

    def run(self):
        SAVE_DATA = True
        # 格式化日期时间为字符串，例如：2024-11-26_15-30-00
        date_time_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        # 构建文件名
        data_name = f"data_{date_time_str}.pickle"
        if SAVE_DATA:
            f_pickle = open('./tools/data/' + data_name, 'wb')

        gait_process = 0
        self.cnt_pd_loop = 0
        time = 0
        target_q = np.zeros(self.num_actions, dtype=np.double)
        tau = np.zeros(self.num_actions, dtype=np.double)
        tauc = np.zeros(self.num_actions, dtype=np.double)
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
            time += self.cfg.dt
            target_q[:] = self.target_q[:]
            tau[:] = self.data.ctrl[:]
            tauc = self.data.actuator_force

            if self.cnt_pd_loop % self.cfg.decimation == 0 and SAVE_DATA:
                # 数据保存 q, dq, target_q
                gyro = obs[0:3]
                # vx_command = cmd.vx
                # vx_current = vel_base[0]
                for key in QKey_list:
                    data_dict[key] = np.append(data_dict[key], eval(Q[key][4]))  # 数据保存
                if self.cnt_pd_loop % 1000 == 0:
                    f_pickle = open('./tools/data/' + data_name, 'wb')
                    pickle.dump({'Q': Q, 'data': data_dict}, f_pickle)
                    print("save_data")
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
        # print("\n=== Joint Order Check ===")
        # for i, name in enumerate(MujocoJointOrder):
        #     print(
        #         f"{i:02d} Real: {name:25s}  Isaac idx: {Mujoco_to_Isaac_indices[i]:2d} -> Isaac name: {IsaacLabJointOrder[Mujoco_to_Isaac_indices[i]]}")


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
