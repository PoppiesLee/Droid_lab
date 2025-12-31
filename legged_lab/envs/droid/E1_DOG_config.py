import math
import torch
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


# =================================================================================
#  自定义 Trot 步态奖励函数
# =================================================================================
def reward_track_trot_gait(env, sensor_cfg: SceneEntityCfg):
    """
    奖励机器人跟随 Trot 步态时钟。
    注意：由于 Fixed Joint Merging，PhysX 把 foot 融合进了 calf_link。
    所以这里检测的是 calf_link 的受力，实际上就是脚底受力。
    """
    # 1. 获取接触力数据
    # 这里必须指向 calf_link，因为 foot_link 被合并了
    contact_sensor = env.scene.sensors[sensor_cfg.name]
    force_norm = torch.norm(contact_sensor.data.net_forces_w_history[:, 0, sensor_cfg.body_ids], dim=-1)
    is_contact = force_norm > 1.0

    # 2. 获取步态时钟
    phase = env.gait_process
    sin_phase = torch.sin(2 * torch.pi * phase)

    # 3. 定义期望接触状态
    desired_contact = torch.zeros_like(is_contact, dtype=torch.float)

    # 逻辑：
    # sin > 0: 期望 FL(0) + RR(3) 触地
    # sin < 0: 期望 FR(1) + RL(2) 触地

    mask_group1 = (sin_phase > 0)
    mask_group2 = (sin_phase <= 0)

    # Group 1: FL(0) + RR(3)
    desired_contact[mask_group1, 0] = 1.0
    desired_contact[mask_group1, 3] = 1.0

    # Group 2: FR(1) + RL(2)
    desired_contact[mask_group2, 1] = 1.0
    desired_contact[mask_group2, 2] = 1.0

    # 4. 计算奖励
    match = (is_contact.float() == desired_contact).float()
    return torch.mean(match, dim=1)


# =================================================================================
#  配置类
# =================================================================================

@configclass
class E1_DOGRewardCfg(RewardCfg):
    # -- Task Rewards --
    track_lin_vel_xy_exp = RewTerm(func=mdp.track_lin_vel_xy_yaw_frame_exp, weight=1.5, params={"std": 0.5})
    track_ang_vel_z_exp = RewTerm(func=mdp.track_ang_vel_z_world_exp, weight=1.5, params={"std": 0.5})

    # [关键] Trot 步态同步奖励
    trot_gait_tracking = RewTerm(
        func=reward_track_trot_gait,
        weight=1.0,
        params={
            # 指向 calf_link，因为 foot 被合并进去了
            "sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*_calf_link"),
        },
    )

    # -- Base Penalties --
    base_linear_velocity = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    base_angular_velocity = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)

    # -- Motor Penalties --
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-0.001)
    joint_acc = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    joint_torques = RewTerm(func=mdp.joint_torques_l2, weight=-2e-4)
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.1)
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-2.0)
    energy = RewTerm(func=mdp.energy, weight=-1e-2)

    # -- Robot Body Penalties --
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-2.5)

    # -- Feet Rewards/Penalties --
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.5,
        params={
            # 这里也必须用 calf_link
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_calf_link"),
            "sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*_calf_link"),
        },
    )

    # [关键修改] 非预期接触 (Undesired Contacts)
    # 这里的正则必须要仔细：
    # 我们要惩罚 thigh 和 hip
    # 绝对不能包含 calf，因为 calf 现在代表脚！
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1.0,
        params={
            "threshold": 1.0,
            # 只匹配 thigh 和 hip
            "sensor_cfg": SceneEntityCfg("contact_sensor", body_names=[".*_thigh_link", ".*_hip_link"]),
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

        # 终止条件：基座触地
        self.robot.terminate_contacts_body_names = [".*base.*"]

        # [关键] 定义脚部 Body
        # 告诉 Isaac Lab 我们的“脚”在物理引擎里叫什么名字
        self.robot.feet_body_names = [".*_calf_link"]

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

        self.reward.track_lin_vel_xy_exp_relax = RewTerm(func=mdp.track_lin_vel_xy_yaw_frame_exp, weight=1.0,
                                                         params={"std": 0.7})
        self.reward.track_lin_vel_xy_exp.weight = 0.5
        self.reward.track_ang_vel_z_exp.weight = 1.5


@configclass
class E1_DOGRoughAgentCfg(BaseAgentCfg):
    experiment_name: str = "E1_DOG_rough"
    wandb_project: str = "E1_DOG_rough"
    policy = RNNPolicyCfg()