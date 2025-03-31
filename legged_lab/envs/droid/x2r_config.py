from legged_lab.envs.base.base_env_config import (  # noqa:F401
    BaseEnvCfg, BaseAgentCfg, BaseSceneCfg, RobotCfg, DomainRandCfg,
    RewardCfg, HeightScannerCfg, AddRigidBodyMassCfg, PhysxCfg, SimCfg, MLPPolicyCfg, RNNPolicyCfg
)
from legged_lab.assets.droid import X2R_CFG
from legged_lab.terrains import GRAVEL_TERRAINS_CFG, ROUGH_TERRAINS_CFG
from isaaclab.managers import RewardTermCfg as RewTerm
import legged_lab.mdp as mdp
from isaaclab.managers.scene_entity_cfg import SceneEntityCfg
from isaaclab.utils import configclass


@configclass
class X2RRewardCfg(RewardCfg):
    track_lin_vel_xy_exp = RewTerm(func=mdp.track_lin_vel_xy_yaw_frame_exp, weight=1.0, params={"std": 0.5})
    track_ang_vel_z_exp = RewTerm(func=mdp.track_ang_vel_z_world_exp, weight=1.0, params={"std": 0.5})
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-1.0)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
    energy = RewTerm(func=mdp.energy, weight=-1e-3)
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    undesired_contacts = RewTerm(func=mdp.undesired_contacts, weight=-1.0, params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names="(?!.*ankle.*).*"), "threshold": 1.0})
    fly = RewTerm(func=mdp.fly, weight=-1.0, params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*ankle_pitch.*"), "threshold": 1.0})
    body_orientation_l2 = RewTerm(func=mdp.body_orientation_l2, params={"asset_cfg": SceneEntityCfg("robot", body_names=".*pelvis.*")}, weight=-2.0)
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-1.0)
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    feet_air_time = RewTerm(func=mdp.feet_air_time_positive_biped, weight=0.25, params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*ankle_pitch.*"), "threshold": 0.4})
    feet_slide = RewTerm(func=mdp.feet_slide, weight=-0.25, params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*ankle_pitch.*"), "asset_cfg": SceneEntityCfg("robot", body_names=".*_ankle_pitch.*")})
    feet_force = RewTerm(func=mdp.body_force, weight=-3e-3, params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*ankle_pitch.*"), "threshold": 400, "max_reward": 300})
    feet_too_near = RewTerm(func=mdp.feet_too_near_humanoid, weight=-3.0, params={"asset_cfg": SceneEntityCfg("robot", body_names=[".*ankle_pitch.*"]), "threshold": 0.25})
    feet_stumble = RewTerm(func=mdp.feet_stumble, weight=-3.0, params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=[".*ankle_pitch.*"])})
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-2.0)
    joint_deviation_hip = RewTerm(func=mdp.joint_deviation_l1, weight=-0.25, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_yaw.*", ".*_hip_roll.*"])})
    joint_deviation_legs = RewTerm(func=mdp.joint_deviation_l1, weight=-0.02, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_pitch.*", ".*_knee.*", ".*_ankle.*"])})
    # feet_swings = RewTerm(func=mdp.feet_swing, weight=0.5, params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=[".*ankle_pitch.*"])})

@configclass
class X2RFlatEnvCfg(BaseEnvCfg):

    reward = X2RRewardCfg()

    def __post_init__(self):
        super().__post_init__()
        self.scene.height_scanner.prim_body_name = "pelvis_link"
        self.scene.robot = X2R_CFG
        self.scene.terrain_type = "generator"
        self.scene.terrain_generator = GRAVEL_TERRAINS_CFG
        self.robot.terminate_contacts_body_names = [".*pelvis.*"]
        self.robot.feet_body_names = [".*ankle_pitch.*"]
        self.domain_rand.add_rigid_body_mass.params["body_names"] = [".*pelvis.*"]


@configclass
class X2RFlatAgentCfg(BaseAgentCfg):
    experiment_name: str = "x2r_flat"
    wandb_project: str = "x2r_flat"


@configclass
class X2RRoughEnvCfg(X2RFlatEnvCfg):

    def __post_init__(self):
        super().__post_init__()
        self.scene.height_scanner.enable_height_scan = True
        self.robot.actor_obs_history_length = 1
        self.robot.critic_obs_history_length = 1
        self.reward.track_lin_vel_xy_exp.weight = 1.5
        self.reward.track_ang_vel_z_exp.weight = 1.5
        self.reward.lin_vel_z_l2.weight = -0.25


@configclass
class X2RRoughAgentCfg(BaseAgentCfg):
    experiment_name: str = "x2r_rough"
    wandb_project: str = "x2r_rough"
    policy = RNNPolicyCfg()
