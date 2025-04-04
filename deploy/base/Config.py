import os
import numpy as np

# 获取当前文件的完整路径
current_file_path = os.path.abspath(__file__)
# 获取当前文件所在的目录
current_dir = os.path.dirname(current_file_path)

class Config:
    dt = 0.001
    decimation = 10
    num_arm_actions = 10
    num_leg_actions = 10
    hands_enable = True
    num_actions = 10
    num_observations = 660
    default_joints = np.array([0., 0., 0.4, -0.8, 0.4, 0., 0., 0., 0.4, -0.8, 0.4, 0.], dtype=np.float32)
    dof_stiffness = np.array([100, 160, 160, 160, 20, 20, 100, 160, 160, 160, 20, 20], dtype=np.float32)
    # dof_stiffness*=0.8
    dof_damping = np.array([2, 6, 6, 6, 2, 2, 2, 6, 6, 6, 2, 2], dtype=np.float32)
    run_duration = 100.0  # 单位s
    gait_frequency = 1.5  # sec
    action_scale = 0.5

    grpc_channel = '192.168.55.110'
    effort_limit = np.array([50, 110, 72, 150, 45, 20,  50, 110, 72, 150, 45, 20], dtype=np.float32)  # 峰值扭矩