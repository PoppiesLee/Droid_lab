import math

from isaaclab.envs.mdp import joint_vel
from sympy.physics.vector.printing import params

from legged_lab.envs.base.base_env_config import (  # noqa:F401
    BaseEnvCfg, BaseAgentCfg, BaseSceneCfg, RobotCfg, DomainRandCfg, CommandsCfg, CommandRangesCfg,
    RewardCfg, HeightScannerCfg, AddRigidBodyMassCfg, PhysxCfg, SimCfg, MLPPolicyCfg, RNNPolicyCfg
)
from legged_lab.assets.droid import E1_CFG
from legged_lab.terrains import GRAVEL_TERRAINS_CFG, ROUGH_TERRAINS_CFG
from isaaclab.managers import RewardTermCfg as RewTerm
import legged_lab.mdp as mdp
from isaaclab.managers.scene_entity_cfg import SceneEntityCfg
from isaaclab.utils import configclass

@configclass
class E1RewardCfg(RewardCfg):
    track_lin_vel_xy_exp = RewTerm(func=mdp.track_lin_vel_xy_yaw_frame_exp, weight=1.5, params={"std": 0.5})
    track_ang_vel_z_exp = RewTerm(func=mdp.track_ang_vel_z_world_exp, weight=1.5, params={"std": 0.5})
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-0.15)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.20)
    dof_vel_l2 = RewTerm(func=mdp.joint_vel_l2, weight=-1e-3)
    # === 能量与动作平滑 ===
    energy = RewTerm(func=mdp.energy, weight=-1.0e-3)
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    # === 接触与身体稳定 ===
    undesired_contacts = RewTerm(func=mdp.undesired_contacts, weight=-3.0,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names="(?!.*ankle.*).*"),"threshold": 1.0})
    fly = RewTerm(func=mdp.fly, weight=-1.0,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*ankle_roll.*"),"threshold": 1.0})
    body_orientation_l2 = RewTerm(func=mdp.body_orientation_l2,params={"asset_cfg": SceneEntityCfg("robot", body_names="torso_link")}, weight=-5.0)
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-2.0)
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    # === 脚部接触类 ===
    feet_air_time = RewTerm(func=mdp.feet_air_time_positive_biped, weight=0.50,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*ankle_roll.*"),"threshold": 0.5})
    feet_slide = RewTerm(func=mdp.feet_slide, weight=-0.10,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*ankle_roll.*"),"asset_cfg": SceneEntityCfg("robot", body_names=".*_ankle_roll.*")})
    feet_force = RewTerm(func=mdp.body_force, weight=-3e-3,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*ankle_roll.*"),"threshold": 500, "max_reward": 400})
    feet_too_near = RewTerm(func=mdp.feet_too_near_humanoid, weight=-1.0,params={"asset_cfg": SceneEntityCfg("robot", body_names=[".*ankle_roll.*"]),"threshold": 0.24})
    feet_stumble = RewTerm(func=mdp.feet_stumble, weight=-1.0,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=[".*ankle_roll.*"])})
    # === 关节约束 ===
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-1.5)
    joint_deviation_waist_yaw = RewTerm(func=mdp.joint_deviation_l1, weight=-1.0,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*waist_yaw_joint"])})
    joint_deviation_hip_yaw = RewTerm(func=mdp.joint_deviation_l1, weight=-0.5,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_yaw_joint"])})
    joint_deviation_hip_roll = RewTerm(func=mdp.joint_deviation_l1, weight=-0.1,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_roll_joint"])})
    # joint_deviation_arms = RewTerm(func=mdp.joint_deviation_l1, weight=-0.2, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*waist.*", ".*_shoulder_roll.*", ".*_shoulder_yaw.*", ".*_wrist.*"])})
    joint_deviation_legs = RewTerm(func=mdp.joint_deviation_l1, weight=-0.02,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_pitch_joint"])})
    joint_deviation_legs_knee = RewTerm(func=mdp.joint_deviation_l1, weight=-0.01,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_knee.*"])})
    joint_deviation_feet = RewTerm(func=mdp.joint_deviation_l1, weight=-0.02,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_ankle_pitch.*"])})
    joint_deviation_ankle = RewTerm(func=mdp.joint_deviation_l1, weight=-1.5,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_ankle_roll.*"])})
    feet_swings = RewTerm(func=mdp.feet_swing, weight=1.0,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=[".*_ankle_roll.*"])})
    # feet_height = RewTerm(func=mdp.feet_height, weight=0.5,params={"asset_cfg": SceneEntityCfg("robot", body_names=[".*ankle_roll.*"]),"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=[".*ankle_roll.*"])})
    # dof_torque_limits = RewTerm(func=mdp.torque_limits, weight=-1.0, params={"asset_cfg": SceneEntityCfg("robot",joint_names=[".*waist_yaw_joint",".*_hip_yaw_joint",".*_hip_roll_joint",".*_hip_pitch_joint",".*_knee.*",".*ankle_pitch.*",".*_ankle_roll.*"]),"soft_torque_limit": 0.9})
    dof_torque_limits = RewTerm(func=mdp.torque_limits, weight=-1.0, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_roll_joint", ".*_ankle_roll.*"]), "soft_torque_limit": 0.9})
    dof_vel_limits = RewTerm(func=mdp.dof_vel_limits, weight=-1.0, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*waist_yaw_joint", ".*_hip_yaw_joint", ".*_hip_roll_joint", ".*_hip_pitch_joint", ".*_knee.*", ".*ankle_pitch.*", ".*_ankle_roll.*"]), "soft_dof_vel_limit": 0.9})
@configclass
class E1FlatEnvCfg(BaseEnvCfg):
    reward = E1RewardCfg()

    def __post_init__(self):
        super().__post_init__()
        self.scene.height_scanner.prim_body_name = "pelvis"
        self.scene.robot = E1_CFG
        self.scene.terrain_type = "generator"
        self.scene.terrain_generator = GRAVEL_TERRAINS_CFG
        self.robot.terminate_contacts_body_names = [".*pelvis.*"]
        self.robot.feet_body_names = [".*ankle_roll.*"]
        self.domain_rand.add_rigid_body_mass.params["body_names"] = [".*pelvis.*"]


@configclass
class E1FlatAgentCfg(BaseAgentCfg):
    experiment_name: str = "E1_flat"
    wandb_project: str = "E1_flat"


@configclass
class E1RoughEnvCfg(E1FlatEnvCfg):

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
        self.reward.lin_vel_z_l2.weight = -0.25


@configclass
class E1RoughAgentCfg(BaseAgentCfg):
    experiment_name: str = "E1_rough"
    wandb_project: str = "E1_rough"
    policy = RNNPolicyCfg()