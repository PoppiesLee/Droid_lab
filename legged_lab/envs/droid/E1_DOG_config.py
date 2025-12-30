import math

from isaaclab.envs.mdp import joint_vel
from sympy.physics.vector.printing import params

from legged_lab.envs.base.base_env_config import (  # noqa:F401
    BaseEnvCfg, BaseAgentCfg, BaseSceneCfg, RobotCfg, DomainRandCfg, CommandsCfg, CommandRangesCfg,
    RewardCfg, HeightScannerCfg, AddRigidBodyMassCfg, PhysxCfg, SimCfg, MLPPolicyCfg, RNNPolicyCfg
)
from legged_lab.assets.droid import E1_DOG_CFG
from legged_lab.terrains import GRAVEL_TERRAINS_CFG, ROUGH_TERRAINS_CFG
from isaaclab.managers import RewardTermCfg as RewTerm
import legged_lab.mdp as mdp
from isaaclab.managers.scene_entity_cfg import SceneEntityCfg
from isaaclab.utils import configclass

@configclass
class E1_DOGRewardCfg(RewardCfg):
    # -- task
    track_lin_vel_xy_exp = RewTerm(func=mdp.track_lin_vel_xy_yaw_frame_exp, weight=1.5, params={"std": 0.5})
    track_ang_vel_z_exp = RewTerm(func=mdp.track_ang_vel_z_world_exp, weight=1.5, params={"std": 0.5})

    # -- base
    base_linear_velocity = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    base_angular_velocity = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-0.001)
    joint_acc = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    joint_torques = RewTerm(func=mdp.joint_torques_l2, weight=-2e-4)
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.1)
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-2.0)
    energy = RewTerm(func=mdp.energy, weight=-1e-2)

    # -- robot
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-2.5)
    # joint_pos = RewTerm(
    #     func=mdp.joint_position_penalty,
    #     weight=-0.7,
    #     params={
    #         "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
    #         "stand_still_scale": 5.0,
    #         "velocity_threshold": 0.3,
    #     },
    # )

    # -- feet
    feet_air_time = RewTerm(func=mdp.feet_air_time_positive_biped, weight=0.1,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*_calf.*"),"threshold": 0.5})

    # air_time_variance = RewTerm(
    #     func=mdp.air_time_variance_penalty,
    #     weight=-1.0,
    #     params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*_calf.*")},
    # )
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.5,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_calf.*"),
            "sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*_calf.*"),
        },
    )
    # feet_contact_forces = RewTerm(
    #     func=mdp.contact_forces,
    #     weight=-0.02,
    #     params={
    #         "threshold": 100.0,
    #         "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
    #     },
    # )

    # -- other
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1,
        params={
            "threshold": 1,
            "sensor_cfg": SceneEntityCfg("contact_sensor", body_names=[".*_hip.*", ".*_thigh.*", ".*_calf.*"]),
        },
    )
@configclass
class E1_DOGFlatEnvCfg(BaseEnvCfg):
    reward = E1_DOGRewardCfg()

    def __post_init__(self):
        super().__post_init__()
        self.scene.height_scanner.prim_body_name = "base"
        self.scene.robot = E1_DOG_CFG
        self.scene.terrain_type = "generator"
        self.scene.terrain_generator = GRAVEL_TERRAINS_CFG
        self.robot.terminate_contacts_body_names = [".*base.*"]
        self.robot.feet_body_names = [".*_calf.*"]
        self.domain_rand.add_rigid_body_mass.params["body_names"] = [".*base.*"]


@configclass
class E1_DOGFlatAgentCfg(BaseAgentCfg):
    experiment_name: str = "E1_DOG_flat"
    wandb_project: str = "E1_DOG_flat"


@configclass
class E1_DOGRoughEnvCfg(E1_DOGFlatEnvCfg):

    def __post_init__(self):
        super().__post_init__()
        self.scene.height_scanner.enable_height_scan = True
        self.scene.terrain_generator = GRAVEL_TERRAINS_CFG
        self.robot.actor_obs_history_length = 1
        self.robot.critic_obs_history_length = 1
        self.reward.feet_air_time.weight = 0.25
        self.reward.track_lin_vel_xy_exp_relax = RewTerm(func=mdp.track_lin_vel_xy_yaw_frame_exp, weight=1.0, params={"std": 0.7})
        self.reward.track_lin_vel_xy_exp.weight = 0.5
        self.reward.track_ang_vel_z_exp.weight = 1.5
        # self.reward.lin_vel_z_l2.weight = -0.25


@configclass
class E1_DOGRoughAgentCfg(BaseAgentCfg):
    experiment_name: str = "E1_DOG_rough"
    wandb_project: str = "E1_DOG_rough"
    policy = RNNPolicyCfg()