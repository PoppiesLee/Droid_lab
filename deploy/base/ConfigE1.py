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
    num_leg_actions = 14
    hands_enable = True
    num_actions = 14
    num_observations = 660   #roll  pitch  yaw   knee1  knee2  A_pitch A_roll  roll  pitch  yaw   knee1  knee2  A_pitch A_roll
    default_joints = np.array([0,  -0.39,   0,   0.78,  0.78,   -0.39,     0,   0,  -0.39,   0,   0.78,  0.78,   -0.39,    0], dtype=np.float32)
    # default_joints = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float32)

    dof_stiffness = np.array([100, 100, 30, 100, 100,  20,  30, 100, 100, 30, 100, 100,  20,  30], dtype=np.float32)
    dof_stiffness*=1.0
    dof_damping = np.array([    3,   3,   2,   2,   2,   3,   2,   3,   3,   2,   2,   2,   3,   2], dtype=np.float32)
    run_duration = 100.0  # 单位s
    gait_frequency = 1.5  # sec
    action_scale = 0.25

    # grpc_channel = '192.168.254.100'
    grpc_channel = '192.168.55.110'
    effort_limit = np.array([60, 60, 14, 28, 28, 28, 28, 60, 60, 14, 28, 28, 28, 28], dtype=np.float32)  # 峰值扭矩