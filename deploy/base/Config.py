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
    num_leg_actions = 12
    hands_enable = True
    num_actions = 12
    num_observations = 660
    default_joints = np.array([0,    0,   0,    0.3,   0,   -0.6,  0.3,   0,    0.3,   0,   -0.6,  0.3], dtype=np.float32)
    dof_stiffness = np.array([170,  50, 100,  100,  30, 150, 15, 100,  100,  30, 150, 15], dtype=np.float32)
    dof_stiffness*=1.0
    dof_damping = np.array([5, 1, 4, 4, 1, 4, 2, 4, 4, 1, 4, 2], dtype=np.float32)
    run_duration = 100.0  # 单位s
    gait_frequency = 1.5  # sec
    action_scale = 0.25

    # grpc_channel = '192.168.254.100'
    grpc_channel = '192.168.55.21'
    effort_limit = np.array([100, 50, 100, 100, 50, 100,  20, 100, 100, 50, 100, 20], dtype=np.float32)  # 峰值扭矩