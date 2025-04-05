import os
import numpy as np

# 获取当前文件的完整路径
current_file_path = os.path.abspath(__file__)
# 获取当前文件所在的目录
current_dir = os.path.dirname(current_file_path)

class Config:
    dt = 0.001
    decimation = 10
    run_duration = 0xffff  # 单位s
    num_arm_actions = 10
    num_leg_actions = 10
    gait_frequency = 1.5  # sec
    hands_enable = True
    default_joints = None
    dof_stiffness = None
    dof_damping = None
    effort_limit = None

    grpc_channel = '192.168.55.110'