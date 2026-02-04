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
from isaaclab.actuators import ImplicitActuatorCfg, IdealPDActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from legged_lab.assets import ISAAC_ASSET_DIR

E1_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        # usd_path=f"{ISAAC_ASSET_DIR}/droid/E1-1/E1-1.usd",
        asset_path=f"{ISAAC_ASSET_DIR}/droid/E1/E1.urdf",
        fix_base=False,  # 不固定机器人基座
        replace_cylinders_with_capsules=True,
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
            enabled_self_collisions=False, solver_position_iteration_count=8, solver_velocity_iteration_count=4
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)  # 关节驱动增益(初始化为0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.744),
        joint_pos={
            # ".*waist_yaw_joint": 0.0,
            ".*_hip_pitch_joint": -0.3,
            ".*_hip_roll_joint": 0.0,     # 2 deg
            ".*_hip_yaw_joint": 0.00,    # 12 deg
            ".*_knee_joint": 0.60,  # -24 deg
            ".*_ankle_pitch_joint": -0.3,  #  13 deg
            ".*_ankle_roll_joint":  0.00,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        # "waist": ImplicitActuatorCfg(
        #     joint_names_expr=[".*waist_yaw_joint"],
        #     effort_limit_sim={
        #         ".*waist_yaw_joint": 60.0
        #     },
        #     velocity_limit_sim={
        #         ".*waist_yaw_joint": 20.42
        #     },
        #     stiffness={
        #         ".*waist_yaw_joint":  100.0
        #     },
        #     damping={
        #         ".*waist_yaw_joint": 5.0
        #     },
        #     armature={
        #         ".*waist_yaw_joint": 0.01
        #     },
        # ),
        "legs": IdealPDActuatorCfg(
            joint_names_expr=[".*_hip_pitch_joint", ".*_hip_roll_joint", ".*_hip_yaw_joint", ".*_knee_joint"],
            effort_limit_sim={
                ".*_hip_pitch_joint": 60.0,
                ".*_hip_roll_joint": 36.0,
                ".*_hip_yaw_joint": 36.0,
                ".*_knee_joint": 36.0
            },
            velocity_limit_sim={
                ".*_hip_pitch_joint": 20.42,
                ".*_hip_roll_joint": 50.265,
                ".*_hip_yaw_joint": 50.265,
                ".*_knee_joint": 30.16,
            },
            stiffness={
                ".*_hip_pitch_joint": 200.0,
                ".*_hip_roll_joint": 100.0,
                ".*_hip_yaw_joint": 100.0,
                ".*_knee_joint": 200.0,
            },
            damping={
                ".*_hip_pitch_joint": 5.0,
                ".*_hip_roll_joint": 5.0,
                ".*_hip_yaw_joint": 3.0,
                ".*_knee_joint": 5.0,
            },
            armature={
                ".*_hip_pitch_joint": 0.01,
                ".*_hip_roll_joint": 0.01,
                ".*_hip_yaw_joint": 0.01,
                ".*_knee_joint": 0.01,
            },
        ),
        "feet": IdealPDActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            effort_limit_sim={
                ".*_ankle_pitch_joint": 28.0,
                ".*_ankle_roll_joint": 14.0
            },
            velocity_limit_sim={
                ".*_ankle_pitch_joint": 30.16,
                ".*_ankle_roll_joint": 32.987
            },
            stiffness={
                ".*_ankle_pitch_joint": 20.0,
                ".*_ankle_roll_joint": 10.0
            },
            damping={
                ".*_ankle_pitch_joint": 2.0,
                ".*_ankle_roll_joint": 1.0,
            },
            armature={
                ".*_ankle_pitch_joint": 0.01,
                ".*_ankle_roll_joint": 0.01
            },
        ),
    },
)

E1_DOG_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=f"{ISAAC_ASSET_DIR}/droid/E1_DOG/E1_DOG.urdf",
        fix_base=False,  # 不固定机器人基座
        replace_cylinders_with_capsules=True,
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
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=4
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)  # 关节驱动增益(初始化为0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.40),
        joint_pos={
            "FR_hip_joint": -0.1,
            "RR_hip_joint": -0.1,
            "FL_hip_joint": 0.1,
            "RL_hip_joint": 0.1,
            "FL_thigh_joint": 0.8,
            "FR_thigh_joint": 0.8,
            "RL_thigh_joint": 1.0,
            "RR_thigh_joint": 1.0,
            ".*_calf_joint": -1.5,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": IdealPDActuatorCfg(
            joint_names_expr=[".*R_hip_joint", ".*L_hip_joint", ".*R_thigh_joint", ".*L_thigh_joint"],
            effort_limit_sim={
                ".*R_hip_joint":36.0,
                ".*L_hip_joint":36.0,
                ".*R_thigh_joint":36.0,
                ".*L_thigh_joint":36.0,
            },
            velocity_limit_sim={
                ".*R_hip_joint":50.265,
                ".*L_hip_joint":50.265,
                ".*R_thigh_joint":50.265,
                ".*L_thigh_joint":50.265,
            },
            stiffness={
                ".*R_hip_joint":100,
                ".*L_hip_joint":100,
                ".*R_thigh_joint":100,
                ".*L_thigh_joint":100,
            },
            damping={
                ".*R_hip_joint":3.0,
                ".*L_hip_joint":3.0,
                ".*R_thigh_joint":3.0,
                ".*L_thigh_joint":3.0,
            },
            armature={
                ".*R_hip_joint":0.01,
                ".*L_hip_joint":0.01,
                ".*R_thigh_joint":0.01,
                ".*L_thigh_joint":0.01,
            },
        ),
        "feet": IdealPDActuatorCfg(
            joint_names_expr=[".*_calf_joint"],
            effort_limit_sim={
                ".*_calf_joint":36.0,
            },
            velocity_limit_sim={
                ".*_calf_joint":50.265,
            },
            stiffness={
                ".*_calf_joint":100,
            },
            damping={
                ".*_calf_joint":3.0,
            },
            armature={
                ".*_calf_joint":0.01,
            },
        )
    },
)

G1_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=f"{ISAAC_ASSET_DIR}/droid/G1/g1.urdf",
        fix_base=False,  # 不固定机器人基座
        replace_cylinders_with_capsules=True,
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
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=4
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)  # 关节驱动增益(初始化为0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.80),
        joint_pos={
            ".*waist_yaw_joint": 0.0,
            ".*_hip_pitch_joint": -0.20,
            ".*_hip_roll_joint": 0.0,
            ".*_hip_yaw_joint": 0.0,
            ".*_knee_joint": 0.42,
            ".*_ankle_pitch_joint": -0.23,
            ".*_ankle_roll_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.90,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_yaw_joint",
                ".*_hip_roll_joint",
                ".*_hip_pitch_joint",
                ".*_knee_joint",
                ".*waist_yaw_joint",
            ],
            effort_limit_sim={
                ".*_hip_yaw_joint": 88.0,
                ".*_hip_roll_joint": 88.0,
                ".*_hip_pitch_joint": 88.0,
                ".*_knee_joint": 139.0,
                ".*waist_yaw_joint": 88.0,
            },
            velocity_limit_sim={
                ".*_hip_yaw_joint": 32.0,
                ".*_hip_roll_joint": 20.0,
                ".*_hip_pitch_joint": 32.0,
                ".*_knee_joint": 20.0,
                ".*waist_yaw_joint": 32.0,
            },
            stiffness={
                ".*_hip_yaw_joint": 150.0,
                ".*_hip_roll_joint": 150.0,
                ".*_hip_pitch_joint": 200.0,
                ".*_knee_joint": 200.0,
                ".*waist_yaw_joint": 200.0,
            },
            damping={
                ".*_hip_yaw_joint": 5.0,
                ".*_hip_roll_joint": 5.0,
                ".*_hip_pitch_joint": 5.0,
                ".*_knee_joint": 5.0,
                ".*waist_yaw_joint": 5.0,
            },
            armature={
                ".*_hip_pitch_joint": 0.01,
                ".*_hip_roll_joint": 0.01,
                ".*_hip_yaw_joint": 0.01,
                ".*_knee_joint": 0.01,
                ".*waist_yaw_joint": 0.01,
            },
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            effort_limit_sim={
                ".*_ankle_pitch_joint": 35.0,
                ".*_ankle_roll_joint": 35.0,
            },
            velocity_limit_sim={
                ".*_ankle_pitch_joint": 30.0,
                ".*_ankle_roll_joint": 30.0,
            },
            stiffness={
                ".*_ankle_pitch_joint": 20.0,
                ".*_ankle_roll_joint": 20.0
            },
            damping={
                ".*_ankle_pitch_joint": 2.0,
                ".*_ankle_roll_joint": 2.0,
            },
            armature={
                ".*_ankle_pitch_joint": 0.01,
                ".*_ankle_roll_joint": 0.01
            },
        ),
    },
)

"""Configuration for the Droid X2R Humanoid robot."""
