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
    num_leg_actions = 13
    hands_enable = True
    num_actions = 13
    num_observations = 660   #pitch  roll  yaw   knee   A_pitch   A_roll    pitch    roll  yaw   knee   A_pitch   A_roll  yaw
    default_joints = np.array([-0.3,  0,   0,    0.60,   -0.3,       0,       -0.3,   0,   0,   0.60,   -0.3,       0,     0], dtype=np.float32)
    # default_joints = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float32)

    dof_stiffness = np.array([200.0000, 100.0000, 100.0000, 200.0000,  30.0000,  10.0000, 200.0000, 100.0000, 100.0000, 200.0000,  30.0000,  10.0000,    200.0], dtype=np.float32)
    dof_stiffness*=1.0
    dof_damping = np.array([    5.0000,   5.0000,   3.0000,   5.0000,   2.0000,   2.0000,   5.0000,   5.0000,   3.0000,   5.0000,   2.0000,   2.0000,      5.0], dtype=np.float32)
    # dof_damping = np.array([10.000, 10.000,  5.00, 5.00, 5.000, 5.000, 5.00, 10.0000, 10.0000, 5.000, 5.000, 5.000, 5.000, 5.000], dtype=np.float32)
    run_duration = 100.0  # 单位s
    gait_frequency = 1.5  # sec
    action_scale = 0.25

    # grpc_channel = '192.168.254.100'
    grpc_channel = '192.168.51.2'
    effort_limit = np.array([60, 36, 36, 60, 36, 14, 60, 36, 36, 60, 36, 14, 60], dtype=np.float32)  # 峰值扭矩