import os
import mujoco
import mujoco_viewer
import numpy as np
from tqdm import tqdm
from deploy import DEPLOY_FOLDER_DIR
from deploy.base.Config import Config


mujoco_model_path = os.path.join(DEPLOY_FOLDER_DIR, f"../legged_lab/assets/droid/x2/x2d20/scene.xml")

class Sim2Mujo():
    def __init__(self):
        self.cfg = Config
        self.model = mujoco.MjModel.from_xml_path(filename=mujoco_model_path)
        self.num_joint = self.model.nu
        actuators = self.get_joint_names()
        self.model.opt.timestep = self.cfg.dt
        self.data = mujoco.MjData(self.model)
        mujoco.mj_step(self.model, self.data)
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data, width=1500,height=1500)
        self.cfg.default_joints = np.zeros(self.num_joint, dtype=np.float32)
        self.cfg.dof_stiffness = np.zeros(self.num_joint, dtype=np.float32)
        self.cfg.dof_damping = np.zeros(self.num_joint, dtype=np.float32)
        self.cfg.effort_limit = np.zeros(self.num_joint, dtype=np.float32)
        self.cnt_pd_loop = 0

    def get_joint_names(self):
        actuators = []
        for i in range(0, self.num_joint):
            joint_id = self.model.actuator_trnid[i]  # 获取关节 ID
            _name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id[0])  # 获取关节名称
            actuators.append(_name)
        print("\nMujoco actuators order:\n",actuators, "\n")
        return actuators

    def get_robot_state(self):
        q = self.data.qpos.astype(np.float32)[7:]
        dq = self.data.qvel.astype(np.float32)[6:]
        ang_vel = self.data.qvel[3:6].astype(np.float32)
        quat = self.data.qpos[3:7].astype(np.float32)
        quat[:] = quat[[1, 2, 3, 0]]
        return q, dq, quat, ang_vel

    def set_robot_state(self, target_q, q, dq):  # mujoco关节顺序输入输出
        self.data.ctrl = np.clip(self.cfg.dof_stiffness * (target_q - q) - self.cfg.dof_damping * dq,
                                 -self.cfg.effort_limit, self.cfg.effort_limit)  # Clamp torques
        mujoco.mj_step(self.model, self.data)
        self.viewer.cam.lookat[:] = self.data.qpos.astype(np.float32)[0:3]
        self.viewer.render()

    def run(self):
        self.cnt_pd_loop = 0
        duration_second = 0.01  # 单位:s
        for _ in tqdm(range(int(self.cfg.run_duration / duration_second)), desc="Simulating..."):
            q, dq, quat, ang_vel = self.get_robot_state()
            # if self.cnt_pd_loop % 10 == 0:
            # Generate PD control
            self.set_robot_state(0, q, dq)
            self.cnt_pd_loop += 1
        self.viewer.close()

if __name__ == '__main__':
    mybot = Sim2Mujo()

    print("start main run")
    try:
        while True:
            mybot.run()
    except KeyboardInterrupt:
        print("\n用户中断，停止程序")
    finally:
        print("\n停止程序")
