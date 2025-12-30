import math

from isaaclab.envs.mdp import joint_vel
from sympy.physics.vector.printing import params

from legged_lab.envs.base.base_env_config import (  # noqa:F401
    BaseEnvCfg, BaseAgentCfg, BaseSceneCfg, RobotCfg, DomainRandCfg, CommandsCfg, CommandRangesCfg,
    RewardCfg, HeightScannerCfg, AddRigidBodyMassCfg, PhysxCfg, SimCfg, MLPPolicyCfg, RNNPolicyCfg
)
from legged_lab.assets.droid import X03B_CFG
from legged_lab.terrains import GRAVEL_TERRAINS_CFG, ROUGH_TERRAINS_CFG
from isaaclab.managers import RewardTermCfg as RewTerm
import legged_lab.mdp as mdp
from isaaclab.managers.scene_entity_cfg import SceneEntityCfg
from isaaclab.utils import configclass

# @configclass
# class X3BCommand(CommandsCfg):
#     base_velocity = CommandsCfg(
#         resampling_time_range = (10.0, 10.0),
#         rel_standing_envs = 0.3,
#         rel_heading_envs = 1.0,
#         heading_command = True,
#         heading_control_stiffness = 0.5,
#         debug_vis = True,
#         ranges = (CommandRangesCfg(
#                   lin_vel_x=(-0.5, 0.5),
#                   lin_vel_y=(-0.5, 0.5),
#                   ang_vel_z=(-0.5, 0.5),
#                   heading=(-math.pi, math.pi)
#                   )
#         )
#     )

@configclass
class X3BRewardCfg(RewardCfg):
    # --body
    # track_lin_vel_xy_exp = RewTerm(func=mdp.track_lin_vel_xy_yaw_frame_exp,weight=1.0,params={"std": 0.5})
    # track_ang_vel_z_exp = RewTerm(func=mdp.track_ang_vel_z_world_exp,weight=1.0,params={"std": 0.5})
    # lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2,weight=-1.5)
    # ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2,weight=-0.06325)
    # # energy = RewTerm(func=mdp.energy,weight=-1e-3)
    # joint_vel_torque = RewTerm(func=mdp.joint_vel_torque,weight=-5e-5)
    # dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2,weight=-1.5e-8) #-2.5e-7
    # action_rate_l2 = RewTerm(func=mdp.action_rate_l2,weight=-0.03)
    # body_orientation_pitch = RewTerm(func=mdp.body_orientation_pitch_l2,params={"asset_cfg": SceneEntityCfg("robot", body_names=".*pelvis_link")},weight=-3.0)
    # # body_orientation_roll = RewTerm(func=mdp.body_orientation_roll_l2,weight=-1.5)
    # body_stable = RewTerm(func=mdp.body_stable,params={"asset_cfg": SceneEntityCfg("robot", body_names=".*waist_roll_link")},weight=1.0,)
    # flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2,weight=-2.0)
    # termination_penalty = RewTerm(func=mdp.is_terminated,weight=-200.0)
    #
    # # --waist
    # waist_orientation_pitch = RewTerm(func=mdp.body_orientation_pitch_l2,params={"asset_cfg": SceneEntityCfg("robot", body_names=".*waist_roll_link")},weight=-2.0)
    # waist_orientation_roll = RewTerm(func=mdp.body_orientation_roll_l2,params={"asset_cfg": SceneEntityCfg("robot", body_names=".*waist_roll_link")},weight=-8.0)
    #
    # # --feet
    # fly = RewTerm(func=mdp.fly,weight=-1.0,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*_ankle_roll.*"), "threshold": 1.0})
    # undesired_contacts = RewTerm(func=mdp.undesired_contacts,weight=-1.0,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names="(?!.*ankle.*).*"), "threshold": 1.0})
    # feet_air_time = RewTerm(func=mdp.feet_air_time_positive_biped,weight=0.25,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*_ankle_roll.*"), "threshold": 1.8})
    # feet_slide = RewTerm(func=mdp.feet_slide,weight=-0.20,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*_ankle_roll.*"), "asset_cfg": SceneEntityCfg("robot", body_names=".*_ankle_roll.*")})
    # feet_force = RewTerm(func=mdp.body_force,weight=-2e-2,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*_ankle_roll.*"), "threshold": 450, "max_reward": 300})
    # feet_too_near = RewTerm(func=mdp.feet_too_near_humanoid,weight=-7.0,params={"asset_cfg": SceneEntityCfg("robot", body_names=[".*_ankle_roll.*"]), "threshold": 0.24})
    # feet_stumble = RewTerm(func=mdp.feet_stumble,weight=-1.0,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=[".*_ankle_roll.*"])})
    # feet_swings = RewTerm(func=mdp.feet_swing,weight=0.5,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=[".*_ankle_roll.*"])})
    #
    # # --joint
    # dof_pos_limits = RewTerm(func=mdp.joint_pos_limits,weight=-1.5)
    # joint_deviation_waist_roll = RewTerm(func=mdp.joint_deviation_l1,weight=-6.0,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*waist_roll.*"])})
    # joint_deviation_waist_yaw = RewTerm(func=mdp.joint_deviation_l1,weight=-10.0,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*waist_yaw.*"])})
    # # joint_deviation_hip = RewTerm(func=mdp.joint_deviation_l1,weight=-2.0,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_yaw.*", ".*_hip_roll.*"])})
    # joint_deviation_hip_yaw = RewTerm(func=mdp.joint_deviation_l1, weight=-0.80,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_yaw"])})
    # joint_deviation_hip_roll = RewTerm(func=mdp.joint_deviation_l1, weight=-1.0,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_roll"])})
    # joint_deviation_legs = RewTerm(func=mdp.joint_deviation_l1,weight=-0.01,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_pitch.*", ".*_knee.*"])})
    # joint_deviation_feet = RewTerm(func=mdp.joint_deviation_l1,weight=-0.5,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[ ".*_ankle_pitch.*"])})
    # joint_deviation_ankle = RewTerm(func=mdp.joint_deviation_l1,weight=-0.5,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_ankle_roll.*"])})
    #
    # # --task
    # sym_hip_roll_loss = RewTerm(func=mdp.joint_symmetry,weight=-0.5,params={"std": 0.05,"asset_cfg": SceneEntityCfg("robot",joint_names=["L_hip_roll", "R_hip_roll"])},)
    # sym_knee_pitch_loss = RewTerm(func=mdp.joint_symmetry,weight=-0.5,params={"std": 0.05,"asset_cfg": SceneEntityCfg("robot", joint_names=["L_knee_pitch", "R_knee_pitch"])},
    # )
    # sym_hip_pitch_loss = RewTerm(func=mdp.joint_symmetry,weight=-0.5,params={"std": 0.05, "asset_cfg": SceneEntityCfg("robot", joint_names=["L_hip_pitch", "R_hip_pitch"])},
    # )
    #
    # # --stand
    # stand_still_dof_vel = RewTerm(func=mdp.stand_still_joint_vel,weight=-0.5,params={"command_name": "base_velocity"},)
    track_lin_vel_xy_exp = RewTerm(func=mdp.track_lin_vel_xy_yaw_frame_exp, weight=1.5, params={"std": 0.25})
    track_ang_vel_z_exp = RewTerm(func=mdp.track_ang_vel_z_world_exp, weight=1.0, params={"std": 0.5})
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-1.5)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.135)
    energy = RewTerm(func=mdp.energy, weight=-2.0e-3)
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    undesired_contacts = RewTerm(func=mdp.undesired_contacts, weight=-1.0,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names="(?!.*ankle.*).*"),"threshold": 1.0})
    fly = RewTerm(func=mdp.fly, weight=-1.0,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*ankle_pitch.*"),"threshold": 1.0})
    body_orientation_l2 = RewTerm(func=mdp.body_orientation_l2,params={"asset_cfg": SceneEntityCfg("robot", body_names="pelvis_link")}, weight=-5.0)
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-1.0)
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    feet_air_time = RewTerm(func=mdp.feet_air_time_positive_biped, weight=0.15,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*ankle_roll.*"),"threshold": 0.5})
    feet_slide = RewTerm(func=mdp.feet_slide, weight=-0.25,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*ankle_roll.*"),"asset_cfg": SceneEntityCfg("robot", body_names=".*_ankle_roll.*")})
    feet_force = RewTerm(func=mdp.body_force, weight=-3e-3,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*ankle_roll.*"),"threshold": 500, "max_reward": 400})
    feet_too_near = RewTerm(func=mdp.feet_too_near_humanoid, weight=-3.0,params={"asset_cfg": SceneEntityCfg("robot", body_names=[".*ankle_roll.*"]),"threshold": 0.3})
    feet_stumble = RewTerm(func=mdp.feet_stumble, weight=-2.0,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=[".*ankle_roll.*"])})
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-1.5)
    joint_deviation_waist_roll = RewTerm(func=mdp.joint_deviation_l1, weight=-5.0,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*waist_roll"])})
    joint_deviation_waist_yaw = RewTerm(func=mdp.joint_deviation_l1, weight=-5.0,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*waist_yaw"])})
    joint_deviation_hip_yaw = RewTerm(func=mdp.joint_deviation_l1, weight=-0.80,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_yaw"])})
    joint_deviation_hip_roll = RewTerm(func=mdp.joint_deviation_l1, weight=-0.05,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_roll"])})
    # joint_deviation_arms = RewTerm(func=mdp.joint_deviation_l1, weight=-0.2, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*waist.*", ".*_shoulder_roll.*", ".*_shoulder_yaw.*", ".*_wrist.*"])})
    joint_deviation_legs = RewTerm(func=mdp.joint_deviation_l1, weight=-0.01, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_pitch"])})
    joint_deviation_legs_knee = RewTerm(func=mdp.joint_deviation_l1, weight=-0.01, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_knee.*"])})
    joint_deviation_feet = RewTerm(func=mdp.joint_deviation_l1, weight=-0.01,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_ankle_pitch.*"])})
    joint_deviation_ankle = RewTerm(func=mdp.joint_deviation_l1,weight=-0.5,params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_ankle_roll.*"])})
    feet_swings = RewTerm(func=mdp.feet_swing, weight=1.0,params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=[".*_ankle_roll.*"])})
    # feet_height = RewTerm(func=mdp.feet_height, weight=-7.0,params={"asset_cfg": SceneEntityCfg("robot", body_names=[".*ankle_roll.*"]),"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=[".*ankle_roll.*"])})
@configclass
class X03BFlatEnvCfg(BaseEnvCfg):
    # commands = X3BCommand()
    reward = X3BRewardCfg()

    def __post_init__(self):
        super().__post_init__()
        self.scene.height_scanner.prim_body_name = "pelvis_link"
        self.scene.robot = X03B_CFG
        self.scene.terrain_type = "generator"
        self.scene.terrain_generator = GRAVEL_TERRAINS_CFG
        self.robot.terminate_contacts_body_names = ["waist_roll_link"]
        self.robot.feet_body_names = [".*_ankle_roll.*"]
        self.domain_rand.add_rigid_body_mass.params["body_names"] = ["waist_roll_link"]


@configclass
class X03BFlatAgentCfg(BaseAgentCfg):
    experiment_name: str = "X3B_flat"
    wandb_project: str = "X3B_flat"


@configclass
class X03BRoughEnvCfg(X03BFlatEnvCfg):

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
class X03BRoughAgentCfg(BaseAgentCfg):
    experiment_name: str = "X3B_rough"
    wandb_project: str = "X3B_rough"
    policy = RNNPolicyCfg()