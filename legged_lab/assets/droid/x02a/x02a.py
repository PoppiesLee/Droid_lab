# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for Droid robots.

The following configurations are available:

* :obj:`X02A_CFG`: X02A humanoid robot

Reference: https://github.com/Droidrobotics/Droid_ros
"""
import os
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

##
# Configuration
##
# 构建相对路径（假设要从当前脚本所在目录的父目录出发去找到目标文件）
current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
usd_X02A_path = os.path.join(current_dir, "x02a/x02a.usd")


X02A_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=usd_X02A_path,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=4
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.13),
        joint_pos={
            # ".*_shoulder_pitch": -0.262,  # -15 degrees
            # ".*_shoulder_roll": 0.0,
            # ".*_shoulder_yaw": 0.0,
            # ".*_elbow": 1.920,  # 110 degrees
            ".*_hip_yaw": 0.0,
            ".*_hip_roll": 0.05,
            ".*_hip_pitch": 0.0,
            ".*_knee_pitch": 0.0,
            ".*_ankle_pitch": 0.0,
            ".*_ankle_roll": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_roll", ".*_hip_pitch", ".*_hip_yaw", ".*_knee_pitch"],
            effort_limit={
                ".*_hip_roll": 117.8,
                ".*_hip_pitch": 72,
                ".*_hip_yaw": 83.5,
                ".*_knee_pitch": 190.8,
            },
            velocity_limit={
                ".*_hip_roll": 90,
                ".*_hip_pitch": 97.5,
                ".*_hip_yaw": 43,
                ".*_knee_pitch": 90.38,
            },
            stiffness={
                ".*_hip_roll": 160.0,
                ".*_hip_pitch": 160.0,
                ".*_hip_yaw": 100.0,
                ".*_knee_pitch": 160.0,
            },
            damping={
                ".*_hip_roll": 6.0,
                ".*_hip_pitch": 6.0,
                ".*_hip_yaw": 2.0,
                ".*_knee_pitch": 6.0,
            },
            armature={
                ".*_hip_roll": 0.01,
                ".*_hip_pitch": 0.01,
                ".*_hip_yaw": 0.01,
                ".*_knee_pitch": 0.01,
            },
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch", ".*_ankle_roll"],
            effort_limit={
                ".*_ankle_pitch": 64.8,
                ".*_ankle_roll": 20.67,
            },
            velocity_limit={
                ".*_ankle_pitch": 216.66,
                ".*_ankle_roll": 148,
            },
            stiffness={
                ".*_ankle_pitch": 20.0,
                ".*_ankle_roll": 20.0,
            },
            damping={
                ".*_ankle_pitch": 2.0,
                ".*_ankle_roll": 2.0,
            },
            armature={
                ".*_ankle_pitch": 0.01,
                ".*_ankle_roll": 0.01,
            },
        ),
        # "arms": ImplicitActuatorCfg(
        #     joint_names_expr=[".*_shoulder_pitch", ".*_shoulder_roll", ".*_shoulder_yaw", ".*_elbow"],
        #     effort_limit=300,
        #     velocity_limit=100.0,
        #     stiffness={
        #         ".*_shoulder_pitch": 40.0,
        #         ".*_shoulder_roll": 40.0,
        #         ".*_shoulder_yaw": 40.0,
        #         ".*_elbow": 40.0,
        #     },
        #     damping={
        #         ".*_shoulder_pitch": 10.0,
        #         ".*_shoulder_roll": 10.0,
        #         ".*_shoulder_yaw": 10.0,
        #         ".*_elbow": 10.0,
        #     },
        # ),
    },
)
"""Configuration for the Droid X02A Humanoid robot."""


X02A_MINIMAL_CFG = X02A_CFG.copy()
X02A_MINIMAL_CFG.spawn.usd_path = usd_X02A_path
"""Configuration for the Droid X02A Humanoid robot with fewer collision meshes.

This configuration removes most collision meshes to speed up simulation.
"""
