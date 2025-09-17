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
from legged_lab.assets import ISAAC_ASSET_DIR

X02A_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_ASSET_DIR}/droid/X2C_PRO/X2C_PRO.usd",
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
            enabled_self_collisions=True, solver_position_iteration_count=4, solver_velocity_iteration_count=4
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.13),
        joint_pos={
            # ".*_shoulder_pitch": -0.5236,  # -15 degrees
            # ".*_shoulder_roll": 0.1745,
            # ".*_shoulder_yaw": 0.0,
            # ".*_elbow": 1.3963,  # 110 degrees
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
            effort_limit_sim={
                ".*_hip_roll": 117.8,
                ".*_hip_pitch": 72,
                ".*_hip_yaw": 83.5,
                ".*_knee_pitch": 190.8,
            },
            velocity_limit_sim={
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
            effort_limit_sim={
                ".*_ankle_pitch": 64.8,
                ".*_ankle_roll": 20.67,
            },
            velocity_limit_sim={
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
        #     effort_limit_sim={
        #         ".*_shoulder_pitch": 100.0,
        #         ".*_shoulder_roll": 100.0,
        #         ".*_shoulder_yaw": 100.0,
        #         ".*_elbow": 100.0,
        #     },
        #     velocity_limit_sim={
        #         ".*_shoulder_pitch": 50.0,
        #         ".*_shoulder_roll": 50.0,
        #         ".*_shoulder_yaw": 50.0,
        #         ".*_elbow": 50.0,
        #     },
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
        #     armature={
        #         ".*_shoulder_pitch": 0.01,
        #         ".*_shoulder_roll": 0.01,
        #         ".*_shoulder_yaw": 0.01,
        #         ".*_elbow": 0.01,
        #     },
        # ),
    },
)
"""Configuration for the Droid X02A Humanoid robot."""

X2_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_ASSET_DIR}/droid/x2/x2d10/x2d10.usd",
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
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.91),
        joint_pos={
            ".*_hip_yaw": 0.0,
            ".*_hip_roll": 0.0349,     # 2 deg
            ".*_hip_pitch": 0.2094,    # 12 deg
            ".*_knee_pitch": -0.4189,  # -24 deg
            ".*_ankle_pitch": 0.2269,  #  13 deg
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_roll", ".*_hip_pitch", ".*_hip_yaw", ".*_knee_pitch"],
            effort_limit_sim={
                ".*_hip_roll": 117.8,
                ".*_hip_pitch": 72,
                ".*_hip_yaw": 83.5,
                ".*_knee_pitch": 190.8,
            },
            velocity_limit_sim={
                ".*_hip_roll": 90,
                ".*_hip_pitch": 97.5,
                ".*_hip_yaw": 43,
                ".*_knee_pitch": 90.38,
            },
            stiffness={
                ".*_hip_roll": 200.0,
                ".*_hip_pitch": 200.0,
                ".*_hip_yaw": 100.0,
                ".*_knee_pitch": 200.0,
            },
            damping={
                ".*_hip_roll": 4.0,
                ".*_hip_pitch": 4.0,
                ".*_hip_yaw": 2.0,
                ".*_knee_pitch": 4.0,
            },
            armature={
                ".*_hip_roll": 0.01,
                ".*_hip_pitch": 0.01,
                ".*_hip_yaw": 0.01,
                ".*_knee_pitch": 0.01,
            },
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch"],
            effort_limit_sim={
                ".*_ankle_pitch": 64.8,
            },
            velocity_limit_sim={
                ".*_ankle_pitch": 216.66,
            },
            stiffness={
                ".*_ankle_pitch": 20.0,
            },
            damping={
                ".*_ankle_pitch": 2.0,
            },
            armature={
                ".*_ankle_pitch": 0.01,
            },
        ),
    },
)

"""Configuration for the Droid X2 Humanoid robot."""

X2R_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_ASSET_DIR}/droid/x2r/x2r10/x2r10.usd",
        # usd_path=f"{ISAAC_ASSET_DIR}/droid/x2rw10/x2rw10.usd",
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
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.91),
        joint_pos={
            ".*_hip_yaw": 0.0,
            ".*_hip_roll": 0.0349,     # 2 deg
            ".*_hip_pitch": 0.2094,    # 12 deg
            ".*_knee_pitch": -0.4189,  # -24 deg
            ".*_ankle_pitch": 0.2269,  #  13 deg
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_roll", ".*_hip_pitch", ".*_hip_yaw", ".*_knee_pitch"],
            effort_limit_sim={
                ".*_hip_roll": 117.8,
                ".*_hip_pitch": 72,
                ".*_hip_yaw": 83.5,
                ".*_knee_pitch": 190.8,
            },
            velocity_limit_sim={
                ".*_hip_roll": 90,
                ".*_hip_pitch": 97.5,
                ".*_hip_yaw": 43,
                ".*_knee_pitch": 90.38,
            },
            stiffness={
                ".*_hip_roll": 200.0,
                ".*_hip_pitch": 200.0,
                ".*_hip_yaw": 100.0,
                ".*_knee_pitch": 200.0,
            },
            damping={
                ".*_hip_roll": 4.0,
                ".*_hip_pitch": 4.0,
                ".*_hip_yaw": 2.0,
                ".*_knee_pitch": 4.0,
            },
            armature={
                ".*_hip_roll": 0.01,
                ".*_hip_pitch": 0.01,
                ".*_hip_yaw": 0.01,
                ".*_knee_pitch": 0.01,
            },
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch"],
            effort_limit_sim={
                ".*_ankle_pitch": 64.8,
            },
            velocity_limit_sim={
                ".*_ankle_pitch": 216.66,
            },
            stiffness={
                ".*_ankle_pitch": 20.0,
            },
            damping={
                ".*_ankle_pitch": 2.0,
            },
            armature={
                ".*_ankle_pitch": 0.01,
            },
        ),
    },
)

X03_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/home/bot/GIT-USST/leggedlab/legged_lab/assets/droid/X03/X03.usd",
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
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.124),
        joint_pos={
            ".*waist_roll": 0.0,
            ".*waist_yaw": 0.0,
            ".*_hip_roll": 0.0,     # 2 deg
            ".*_hip_pitch": 0.3,    # 12 deg
            ".*_hip_yaw": 0.0,
            ".*_knee_pitch": -0.6,  # -24 deg
            ".*_ankle_pitch": 0.3,  #  13 deg
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "waist": ImplicitActuatorCfg(
            joint_names_expr=[".*waist_roll", ".*waist_yaw"],
            effort_limit_sim={
                ".*waist_roll": 120.0,
                ".*waist_yaw": 100.0
            },
            velocity_limit_sim={
                ".*waist_roll": 45.0,
                ".*waist_yaw": 45.0
            },
            stiffness={
                ".*waist_roll": 250.0,
                ".*waist_yaw": 200.0
            },
            damping={
                ".*waist_roll": 5.0,
                ".*waist_yaw": 1.0
            },
            armature={
                ".*waist_roll": 0.01,
                ".*waist_yaw": 0.01
            },
        ),
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_roll", ".*_hip_pitch", ".*_hip_yaw", ".*_knee_pitch"],
            effort_limit_sim={
                ".*_hip_roll": 100.0,
                ".*_hip_pitch": 100,
                ".*_hip_yaw": 50.0,
                ".*_knee_pitch": 120.0,
            },
            velocity_limit_sim={
                ".*_hip_roll": 30,
                ".*_hip_pitch": 37.5,
                ".*_hip_yaw": 43,
                ".*_knee_pitch": 90.38,
            },
            stiffness={
                ".*_hip_roll": 100.0,
                ".*_hip_pitch": 100.0,
                ".*_hip_yaw": 50.0,
                ".*_knee_pitch": 120.0,
            },
            damping={
                ".*_hip_roll": 4.0,
                ".*_hip_pitch": 4.0,
                ".*_hip_yaw": 1.0,
                ".*_knee_pitch": 4.0,
            },
            armature={
                ".*_hip_roll": 0.01,
                ".*_hip_pitch": 0.01,
                ".*_hip_yaw": 0.01,
                ".*_knee_pitch": 0.01,
            },
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch"],
            effort_limit_sim={
                ".*_ankle_pitch": 30.0,
            },
            velocity_limit_sim={
                ".*_ankle_pitch": 10,
            },
            stiffness={
                ".*_ankle_pitch": 15.0,
            },
            damping={
                ".*_ankle_pitch": 2.0,
            },
            armature={
                ".*_ankle_pitch": 0.01,
            },
        ),
    },
)

X03B_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/home/bot/GIT-USST/leggedlab/legged_lab/assets/droid/X3B/X03B.usd",
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
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.846),
        joint_pos={
            ".*waist_roll": 0.0,
            ".*waist_yaw": 0.0,
            ".*_hip_roll": 0.0,     # 2 deg
            ".*_hip_pitch": 0.3,    # 12 deg
            ".*_hip_yaw": 0.0,
            ".*_knee_pitch": -0.578,  # -24 deg
            ".*_ankle_pitch": 0.278,
            ".*_ankle_roll": 0.0,  #  13 deg
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "waist": ImplicitActuatorCfg(
            joint_names_expr=[".*waist_roll", ".*waist_yaw"],
            effort_limit_sim={
                ".*waist_roll": 200.0,
                ".*waist_yaw": 200.0
            },
            velocity_limit_sim={
                ".*waist_roll": 45.0,
                ".*waist_yaw": 45.0
            },
            stiffness={
                ".*waist_roll": 200.0,
                ".*waist_yaw":  100.0
            },
            damping={
                ".*waist_roll": 5.0,
                ".*waist_yaw": 5.0
            },
            armature={
                ".*waist_roll": 0.01,
                ".*waist_yaw": 0.01
            },
        ),
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_roll", ".*_hip_pitch", ".*_hip_yaw", ".*_knee_pitch"],
            effort_limit_sim={
                ".*_hip_roll": 200.0,
                ".*_hip_pitch": 200.0,
                ".*_hip_yaw": 200.0,
                ".*_knee_pitch": 200.0,
            },
            velocity_limit_sim={
                ".*_hip_roll": 30,
                ".*_hip_pitch": 37.5,
                ".*_hip_yaw": 43,
                ".*_knee_pitch": 90.38,
            },
            stiffness={
                ".*_hip_roll": 250.0,
                ".*_hip_pitch": 250.0,
                ".*_hip_yaw": 200.0,
                ".*_knee_pitch": 250.0,
            },
            damping={
                ".*_hip_roll": 5.0,
                ".*_hip_pitch": 5.0,
                ".*_hip_yaw": 5.0,
                ".*_knee_pitch": 5.0,
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
            effort_limit_sim={
                ".*_ankle_pitch": 64.8,
                ".*_ankle_roll": 20.0
            },
            velocity_limit_sim={
                ".*_ankle_pitch": 100,
                ".*_ankle_roll": 100
            },
            stiffness={
                ".*_ankle_pitch": 30.0,
                ".*_ankle_roll": 10.0
            },
            damping={
                ".*_ankle_pitch": 2.0,
                ".*_ankle_roll": 1.0,
            },
            armature={
                ".*_ankle_pitch": 0.01,
                ".*_ankle_roll": 0.01
            },
        ),
    },
)

x3_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/home/bot/GIT-USST/leggedlab/legged_lab/assets/droid/x3/X3.usd",
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
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.846),
        joint_pos={
            ".*waist_roll": 0.0,
            ".*waist_yaw": 0.0,
            ".*_hip_roll": 0.0,     # 2 deg
            ".*_hip_pitch": 0.3,    # 12 deg
            ".*_hip_yaw": 0.0,
            ".*_knee_pitch": -0.578,  # -24 deg
            ".*_ankle_pitch": 0.278,
            ".*_ankle_roll": 0.0,  #  13 deg
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "waist": ImplicitActuatorCfg(
            joint_names_expr=[".*waist_roll", ".*waist_yaw"],
            effort_limit_sim={
                ".*waist_roll": 200.0,
                ".*waist_yaw": 200.0
            },
            velocity_limit_sim={
                ".*waist_roll": 45.0,
                ".*waist_yaw": 45.0
            },
            stiffness={
                ".*waist_roll": 300.0,
                ".*waist_yaw":  100.0
            },
            damping={
                ".*waist_roll": 5.0,
                ".*waist_yaw": 5.0
            },
            armature={
                ".*waist_roll": 0.01,
                ".*waist_yaw": 0.01
            },
        ),
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_roll", ".*_hip_pitch", ".*_hip_yaw", ".*_knee_pitch"],
            effort_limit_sim={
                ".*_hip_roll": 200,
                ".*_hip_pitch": 200,
                ".*_hip_yaw": 83.5,
                ".*_knee_pitch": 200,
            },
            velocity_limit_sim={
                ".*_hip_roll": 30,
                ".*_hip_pitch": 37.5,
                ".*_hip_yaw": 43,
                ".*_knee_pitch": 90.38,
            },
            stiffness={
                ".*_hip_roll": 300.0,
                ".*_hip_pitch": 300.0,
                ".*_hip_yaw": 200.0,
                ".*_knee_pitch": 300.0,
            },
            damping={
                ".*_hip_roll": 5.0,
                ".*_hip_pitch": 5.0,
                ".*_hip_yaw": 5.0,
                ".*_knee_pitch": 5.0,
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
            effort_limit_sim={
                ".*_ankle_pitch": 64.8,
                ".*_ankle_roll": 20.0
            },
            velocity_limit_sim={
                ".*_ankle_pitch": 100,
                ".*_ankle_roll": 100
            },
            stiffness={
                ".*_ankle_pitch": 30.0,
                ".*_ankle_roll": 10.0
            },
            damping={
                ".*_ankle_pitch": 2.0,
                ".*_ankle_roll": 1.0,
            },
            armature={
                ".*_ankle_pitch": 0.01,
                ".*_ankle_roll": 0.01
            },
        ),
    },
)

X2C_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_ASSET_DIR}/droid/X2C_PRO/X2C_PRO.usd",
        # usd_path=f"{ISAAC_ASSET_DIR}/droid/x2rw10/x2rw10.usd",
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
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.03),
        joint_pos={
            ".*_hip_yaw": 0.0,
            ".*_hip_roll": 0.0,     # 2 deg
            ".*_hip_pitch": 0.20,    # 12 deg
            ".*_knee_pitch": -0.40,  # -24 deg
            ".*_ankle_pitch": 0.20,  #  13 deg
            ".*_ankle_roll": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_roll", ".*_hip_pitch", ".*_hip_yaw", ".*_knee_pitch"],
            effort_limit_sim={
                ".*_hip_roll": 117.8,
                ".*_hip_pitch": 72,
                ".*_hip_yaw": 83.5,
                ".*_knee_pitch": 190.8,
            },
            velocity_limit_sim={
                ".*_hip_roll": 90,
                ".*_hip_pitch": 97.5,
                ".*_hip_yaw": 43,
                ".*_knee_pitch": 90.38,
            },
            stiffness={
                ".*_hip_roll": 200.0,
                ".*_hip_pitch": 200.0,
                ".*_hip_yaw": 100.0,
                ".*_knee_pitch": 200.0,
            },
            damping={
                ".*_hip_roll": 4.0,
                ".*_hip_pitch": 4.0,
                ".*_hip_yaw": 2.0,
                ".*_knee_pitch": 4.0,
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
            effort_limit_sim={
                ".*_ankle_pitch": 64.8,
                ".*_ankle_roll": 20.0
            },
            velocity_limit_sim={
                ".*_ankle_pitch": 100,
                ".*_ankle_roll": 100
            },
            stiffness={
                ".*_ankle_pitch": 30.0,
                ".*_ankle_roll": 10.0
            },
            damping={
                ".*_ankle_pitch": 2.0,
                ".*_ankle_roll": 1.0,
            },
            armature={
                ".*_ankle_pitch": 0.01,
                ".*_ankle_roll": 0.01
            },
        ),
    },
)

# E1_CFG = ArticulationCfg(
#     spawn=sim_utils.UsdFileCfg(
#         usd_path=f"{ISAAC_ASSET_DIR}/droid/E1/E1.usd",
#         # asset_path=f"{ISAAC_ASSET_DIR}/droid/E1/E1.urdf",
#         # usd_path=f"{ISAAC_ASSET_DIR}/droid/x2rw10/x2rw10.usd",
#         activate_contact_sensors=True,
#         rigid_props=sim_utils.RigidBodyPropertiesCfg(
#             disable_gravity=False,
#             retain_accelerations=False,
#             linear_damping=0.0,
#             angular_damping=0.0,
#             max_linear_velocity=1000.0,
#             max_angular_velocity=1000.0,
#             max_depenetration_velocity=1.0,
#         ),
#         articulation_props=sim_utils.ArticulationRootPropertiesCfg(
#             enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=4
#         ),
#     ),
#     init_state=ArticulationCfg.InitialStateCfg(
#         pos=(0.0, 0.0, 0.553),
#         joint_pos={
#             ".*_hip_pitch_joint": -0.30,
#             ".*_hip_roll_joint": 0.0,     # 2 deg
#             ".*_hip_yaw_joint": 0.00,    # 12 deg
#             ".*_knee_joint": 0.60,  # -24 deg
#             ".*_ankle_roll_joint": 0.00,  #  13 deg
#             ".*_ankle_pitch_joint": -0.30,
#         },
#         joint_vel={".*": 0.0},
#     ),
#     soft_joint_pos_limit_factor=0.9,
#     actuators={
#         "legs": ImplicitActuatorCfg(
#             joint_names_expr=[".*_hip_roll_joint", ".*_hip_pitch_joint", ".*_hip_yaw_joint", ".*_knee_joint"],
#             effort_limit_sim={
#                 ".*_hip_roll_joint": 60.0,
#                 ".*_hip_pitch_joint": 60.0,
#                 ".*_hip_yaw_joint": 14.0,
#                 ".*_knee_joint": 28.0
#             },
#             velocity_limit_sim={
#                 ".*_hip_roll_joint": 90,
#                 ".*_hip_pitch_joint": 97.5,
#                 ".*_hip_yaw_joint": 43,
#                 ".*_knee_joint": 90.38,
#             },
#             stiffness={
#                 ".*_hip_roll_joint": 100.0,
#                 ".*_hip_pitch_joint": 100.0,
#                 ".*_hip_yaw_joint": 30.0,
#                 ".*_knee_joint": 100.0,
#             },
#             damping={
#                 ".*_hip_roll_joint": 4.0,
#                 ".*_hip_pitch_joint": 4.0,
#                 ".*_hip_yaw_joint": 2.0,
#                 ".*_knee_joint": 4.0,
#             },
#             armature={
#                 ".*_hip_roll_joint": 0.01,
#                 ".*_hip_pitch_joint": 0.01,
#                 ".*_hip_yaw_joint": 0.01,
#                 ".*_knee_joint": 0.01,
#             },
#         ),
#         "feet": ImplicitActuatorCfg(
#             joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
#             effort_limit_sim={
#                 ".*_ankle_pitch_joint": 14.0,
#                 ".*_ankle_roll_joint": 14.0
#             },
#             velocity_limit_sim={
#                 ".*_ankle_pitch_joint": 100,
#                 ".*_ankle_roll_joint": 100
#             },
#             stiffness={
#                 ".*_ankle_pitch_joint": 20.0,
#                 ".*_ankle_roll_joint": 40.0
#             },
#             damping={
#                 ".*_ankle_pitch_joint": 3.0,
#                 ".*_ankle_roll_joint": 2.0,
#             },
#             armature={
#                 ".*_ankle_pitch_joint": 0.01,
#                 ".*_ankle_roll_joint": 0.01
#             },
#         ),
#     },
# )

E1_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        # usd_path=f"{ISAAC_ASSET_DIR}/droid/E1/E1.usd",
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
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=4
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)  # 关节驱动增益(初始化为0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.553),
        joint_pos={
            ".*_hip_pitch_joint": -0.20,
            ".*_hip_roll_joint": 0.0,     # 2 deg
            ".*_hip_yaw_joint": 0.00,    # 12 deg
            ".*_knee_joint": 0.40,  # -24 deg
            ".*_ankle_roll_joint": 0.00,  #  13 deg
            ".*_ankle_pitch_joint": -0.20,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_roll_joint", ".*_hip_pitch_joint", ".*_hip_yaw_joint", ".*_knee_joint"],
            effort_limit_sim={
                ".*_hip_roll_joint": 60.0,
                ".*_hip_pitch_joint": 60.0,
                ".*_hip_yaw_joint": 14.0,
                ".*_knee_joint": 56.0
            },
            velocity_limit_sim={
                ".*_hip_roll_joint": 180.0,
                ".*_hip_pitch_joint": 180.0,
                ".*_hip_yaw_joint": 260.0,
                ".*_knee_joint": 260.0,
            },
            stiffness={
                ".*_hip_roll_joint": 100.0,
                ".*_hip_pitch_joint": 100.0,
                ".*_hip_yaw_joint": 30.0,
                ".*_knee_joint": 100.0,
            },
            damping={
                ".*_hip_roll_joint": 3.0,
                ".*_hip_pitch_joint": 3.0,
                ".*_hip_yaw_joint": 2.0,
                ".*_knee_joint": 3.0,
            },
            armature={
                ".*_hip_roll_joint": 0.01,
                ".*_hip_pitch_joint": 0.01,
                ".*_hip_yaw_joint": 0.01,
                ".*_knee_joint": 0.01,
            },
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            effort_limit_sim={
                ".*_ankle_pitch_joint": 30.0,
                ".*_ankle_roll_joint": 30.0
            },
            velocity_limit_sim={
                ".*_ankle_pitch_joint": 260,
                ".*_ankle_roll_joint": 260
            },
            stiffness={
                ".*_ankle_pitch_joint": 20.0,
                ".*_ankle_roll_joint": 40.0
            },
            damping={
                ".*_ankle_pitch_joint": 3.0,
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
