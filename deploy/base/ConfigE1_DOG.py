import os
import numpy as np

# 获取当前文件的完整路径
current_file_path = os.path.abspath(__file__)
# 获取当前文件所在的目录
current_dir = os.path.dirname(current_file_path)

class Config:
    dt = 0.001
    decimation = 10
    num_arm_actions = 8
    num_leg_actions = 8
    hands_enable = True
    num_actions = 12
    num_observations = 660
    default_joints = np.array([0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0], dtype=np.float32)
    # default_joints = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float32)

    dof_stiffness = np.array([100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
                              100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0], dtype=np.float32)
    dof_stiffness*=1.0
    dof_damping   = np.array([3.0,  3.0, 3.0,  3.0, 3.0,  3.0, 3.0,  3.0,
                              3.0,  3.0, 3.0,  3.0, 3.0,  3.0, 3.0,  3.0], dtype=np.float32)
    # dof_damping = np.array([10.000, 10.000,  5.00, 5.00, 5.000, 5.000, 5.00, 10.0000, 10.0000, 5.000, 5.000, 5.000, 5.000, 5.000], dtype=np.float32)
    run_duration = 100.0  # 单位s
    gait_frequency = 1.5  # sec
    action_scale = 0.25

    # grpc_channel = '192.168.254.100'
    grpc_channel = '192.168.51.2'
    effort_limit = np.array([36, 36, 14, 36, 36, 36, 14, 36, 36, 36, 14, 36, 36, 36, 14, 36], dtype=np.float32)  # 峰值扭矩