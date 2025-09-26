from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor
import isaaclab.utils.math as math_utils
from isaaclab.utils.math import quat_apply_inverse, yaw_quat
from isaaclab.assets import Articulation

if TYPE_CHECKING:
    from legged_lab.envs.base.base_env import BaseEnv

# --body
def body_orientation_l2(env: BaseEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    body_orientation = math_utils.quat_apply_inverse(asset.data.body_quat_w[:, asset_cfg.body_ids[0], :], asset.data.GRAVITY_VEC_W)
    return torch.sum(torch.square(body_orientation[:, :2]), dim=1)

def body_orientation_pitch_l2(
    env:BaseEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    body_orientation = math_utils.quat_apply_inverse(asset.data.body_quat_w[:, asset_cfg.body_ids[0], :], asset.data.GRAVITY_VEC_W)
    return torch.sum(torch.square(body_orientation[:, :1]), dim=1)

def body_orientation_roll_l2(
    env:BaseEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    body_orientation = math_utils.quat_apply_inverse(asset.data.body_quat_w[:, asset_cfg.body_ids[0], :], asset.data.GRAVITY_VEC_W)
    return torch.sum(torch.square(body_orientation[:, 1:2]), dim=1)

def track_lin_vel_xy_yaw_frame_exp(
    env:BaseEnv, std: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    vel_yaw = math_utils.quat_apply_inverse(math_utils.yaw_quat(asset.data.root_quat_w), asset.data.root_lin_vel_w[:, :3])
    lin_vel_error = torch.sum(torch.square(env.command_generator.command[:, :2] - vel_yaw[:, :2]), dim=1)
    return torch.exp(-lin_vel_error / std**2)

# def track_lin_vel_xy_yaw_frame_exp(
#     env:BaseEnv, std: float, command_name: str, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
# ) -> torch.Tensor:
#     """Reward tracking of linear velocity commands (xy axes) in the gravity aligned robot frame using exponential kernel."""
#     # extract the used quantities (to enable type-hinting)
#     asset: Articulation = env.scene[asset_cfg.name]
#     vel_yaw = quat_apply_inverse(yaw_quat(asset.data.root_quat_w), asset.data.root_lin_vel_w[:, :3])
#     lin_vel_error = torch.sum(
#         torch.square(env.command_manager.get_command(command_name)[:, :2] - vel_yaw[:, :2]), dim=1
#     )
#     return torch.exp(-lin_vel_error / std**2)

def track_ang_vel_z_world_exp(
    env: BaseEnv, std: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    ang_vel_error = torch.square(env.command_generator.command[:, 2] - asset.data.root_ang_vel_w[:, 2])
    return torch.exp(-ang_vel_error / std**2)

# def track_ang_vel_z_world_exp(
#     env:BaseEnv, command_name: str, std: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
# ) -> torch.Tensor:
#     """Reward tracking of angular velocity commands (yaw) in world frame using exponential kernel."""
#     # extract the used quantities (to enable type-hinting)
#     asset: Articulation = env.scene[asset_cfg.name]
#     ang_vel_error = torch.square(env.command_manager.get_command(command_name)[:, 2] - asset.data.root_ang_vel_w[:, 2])
#     return torch.exp(-ang_vel_error / std**2)

def body_stable(env: BaseEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    quat = asset.data.body_quat_w[:, asset_cfg.body_ids, :].reshape(-1, 4)

    # 提取 roll, pitch
    r, p, _ = math_utils.euler_xyz_from_quat(quat)
    euler_xy = torch.stack([r, p], dim=-1)

    # 限制角度在 [-pi, pi]
    euler_xy[euler_xy > torch.pi] -= 2 * torch.pi
    euler_xy[euler_xy < -torch.pi] += 2 * torch.pi

    # 角度平方和
    deviation_sq = torch.sum(torch.square(euler_xy), dim=1)

    # 超过5度(约0.0873 rad)的部分直接给0
    max_angle_rad = 5 * torch.pi / 180
    out_of_range = (torch.abs(euler_xy) > max_angle_rad).any(dim=1)

    # 指数奖励（靠近0最大为1）
    reward = torch.exp(-deviation_sq / 0.01)  # 0.01 控制敏感度（调得较敏感）
    reward[out_of_range] = 0.0

    return reward
# -------------------------  joint  ---------------------------
def lin_vel_z_l2(
    env: BaseEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.square(asset.data.root_lin_vel_b[:, 2])

def ang_vel_xy_l2(env: BaseEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.sum(torch.square(asset.data.root_ang_vel_b[:, :2]), dim=1)

def energy(env: BaseEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    reward = torch.norm(torch.abs(asset.data.applied_torque * asset.data.joint_vel), dim=-1)
    return reward

def joint_vel_torque(
    env:BaseEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    joint_vel = asset.data.joint_vel[:, asset_cfg.joint_ids]  # 索引指定关节
    applied_torque = asset.data.applied_torque[:, asset_cfg.joint_ids]

    # 逐关节计算两种奖励分量 -----------------------------------------------------------------
    # 速度>0时的奖励分量（保留每个关节的贡献，不降维）
    reward_vel_positive = torch.abs(applied_torque * joint_vel)  # 形状: (batch_size, num_joints)
    # 速度<=0时的奖励分量（每个关节的扭矩平方）
    reward_vel_non_positive = torch.square(applied_torque)  # 形状: (batch_size, num_joints)

    # 若速度>0的位置用reward_vel_positive，否则用reward_vel_non_positive
    reward = torch.where(joint_vel > 0.05, reward_vel_positive, reward_vel_non_positive)
    return torch.sum(reward, dim=1)  # 汇总所有关节的奖励

def joint_acc_l2(env: BaseEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.sum(torch.square(asset.data.joint_acc[:, asset_cfg.joint_ids]), dim=1)

def joint_pos_limits(env: BaseEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize joint positions if they cross the soft limits.

    This is computed as a sum of the absolute value of the difference between the joint position and the soft limits.
    """
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # compute out of limits constraints
    out_of_limits = -(
        asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.soft_joint_pos_limits[:, asset_cfg.joint_ids, 0]
    ).clip(max=0.0)
    out_of_limits += (
        asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.soft_joint_pos_limits[:, asset_cfg.joint_ids, 1]
    ).clip(min=0.0)
    return torch.sum(out_of_limits, dim=1)

def action_rate_l2(env: BaseEnv) -> torch.Tensor:
    return torch.sum(torch.square(env.action_buffer._circular_buffer.buffer[:, -1, :] - env.action_buffer._circular_buffer.buffer[:, -2, :]), dim=1)

def joint_symmetry(env: BaseEnv, std: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    joint_error = torch.abs(asset.data.joint_pos[:, asset_cfg.joint_ids[0]] - asset.data.default_joint_pos[:, asset_cfg.joint_ids[0]] + asset.data.joint_pos[:, asset_cfg.joint_ids[1]] - asset.data.default_joint_pos[:, asset_cfg.joint_ids[1]])
    return torch.exp(-joint_error / std**2)

def flat_orientation_l2(env: BaseEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.sum(torch.square(asset.data.projected_gravity_b[:, :2]), dim=1)

def is_terminated(env: BaseEnv) -> torch.Tensor:
    """Penalize terminated episodes that don't correspond to episodic timeouts."""
    return env.reset_buf * ~env.time_out_buf

def body_force(env: BaseEnv, sensor_cfg: SceneEntityCfg, threshold: float = 400, max_reward: float = 350) -> torch.Tensor:
    """瞬时接触力,仅关注z轴方向的力（net_forces_w[..., 2]），适合垂直方向的接触（如足部着地）。"""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    reward = contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, 2].norm(dim=-1)
    reward[reward < threshold] = 0
    reward[reward > threshold] -= threshold
    reward = reward.clamp(min=0, max=max_reward)
    return reward

def joint_deviation_l1(env: BaseEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    angle = asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    return torch.sum(torch.abs(angle), dim=1)


# ********************************** waist ***********************************
def waist_roll_step_coord(
    env:BaseEnv, std: float, k: float, sensor_cfg: SceneEntityCfg, asset_cfg=SceneEntityCfg("robot")
)-> torch.Tensor:
    #
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]  # 读取接触脚
    in_contact = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids] > 0.0
    # 左脚支撑 → stance = +1，右脚支撑 → –1，其余 0
    stance = torch.where(in_contact[:,0] & ~in_contact[:,1], +1.0,   # 右脚腾空，腰 roll ≈ +θ
             torch.where(in_contact[:,1] & ~in_contact[:,0], -1.0, 0.0))
    # 读取腰 roll 角度
    asset: Articulation = env.scene[asset_cfg.name]
    roll = asset.data.joint_pos[:, asset_cfg.joint_ids[0]]
    desired = k * stance  # 目标 roll = 0.10 rad × stance
    return torch.exp(-((roll - desired)**2) / std**2)

def waist_yaw_heading_alignment(
    env:BaseEnv, command_name: str, std: float, k: float, asset_cfg=SceneEntityCfg("robot")
)-> torch.Tensor:
    """
        Encourage waist-yaw to align (proportionally) with the commanded heading rate
    """
    asset: Articulation = env.scene[asset_cfg.name]
    yaw_cmd = env.command_manager.get_command(command_name)[:, 2]   # 期望 ω_z
    yaw_pos = asset.data.joint_pos[:, asset_cfg.joint_ids[0]]
    # 让腰角度 ≈ ∫ω_cmd·Δt，近似用 proportional：k = 0.25
    desired = k * yaw_cmd
    reward = torch.exp(-((yaw_pos - desired)**2) / std**2)
    return reward

def waist_stability_l2(
    env:BaseEnv, asset_cfg=SceneEntityCfg("robot")
)-> torch.Tensor:
    """
        保持腰部关节尽量接近中立位置， 防止过度剧烈的腰部运动
        奖励 = Σ(关节角度²) + 0.05 × Σ(关节速度²)
    """
    asset: Articulation = env.scene[asset_cfg.name]
    jpos  = asset.data.joint_pos[:, asset_cfg.joint_ids]          # 角度
    jvel  = asset.data.joint_vel[:, asset_cfg.joint_ids]          # 角速度
    reward = torch.sum(jpos**2, dim=1) + 0.05*torch.sum(jvel**2, dim=1)
    return reward


# ----------------------------- feet ----------------------------------
def undesired_contacts(env: BaseEnv, threshold: float, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    net_contact_forces = contact_sensor.data.net_forces_w_history
    is_contact = torch.max(torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0] > threshold
    return torch.sum(is_contact, dim=1)

def fly(env: BaseEnv, threshold: float, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    net_contact_forces = contact_sensor.data.net_forces_w_history
    is_contact = torch.max(torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0] > threshold
    return torch.sum(is_contact, dim=-1) < 0.5

def feet_air_time_positive_biped(env: BaseEnv, threshold: float, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    air_time = contact_sensor.data.current_air_time[:, sensor_cfg.body_ids]
    contact_time = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids]
    in_contact = contact_time > 0.0
    in_mode_time = torch.where(in_contact, contact_time, air_time)
    single_stance = torch.sum(in_contact.int(), dim=1) == 1
    reward = torch.min(torch.where(single_stance.unsqueeze(-1), in_mode_time, 0.0), dim=1)[0]
    reward = torch.clamp(reward, max=threshold)
    # no reward for zero command
    reward *= (torch.norm(env.command_generator.command[:, :2], dim=1) + torch.abs(env.command_generator.command[:, 2])) > 0.1
    return reward

def feet_slide(env: BaseEnv, sensor_cfg: SceneEntityCfg, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    contacts = contact_sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids, :].norm(dim=-1).max(dim=1)[0] > 1.0
    asset: Articulation = env.scene[asset_cfg.name]
    body_vel = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :2]
    reward = torch.sum(body_vel.norm(dim=-1) * contacts, dim=1)
    return reward

def feet_stumble(env: BaseEnv, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    return torch.any(torch.norm(contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, :2], dim=2) > 5 * torch.abs(contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, 2]), dim=1)

def feet_too_near_humanoid(env: BaseEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"), threshold: float = 0.2) -> torch.Tensor:
    assert len(asset_cfg.body_ids) == 2
    asset: Articulation = env.scene[asset_cfg.name]
    feet_pos = asset.data.body_pos_w[:, asset_cfg.body_ids, :]
    distance = torch.norm(feet_pos[:, 0] - feet_pos[:, 1], dim=-1)
    return (threshold - distance).clamp(min=0)

def feet_height(env: BaseEnv, sensor_cfg: SceneEntityCfg, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    assert len(asset_cfg.body_ids) == 2
    asset: Articulation = env.scene[asset_cfg.name]
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]

    # 判断是否在摆动期（未接触地面）
    in_air = contact_sensor.data.current_contact_time[:, asset_cfg.body_ids] <= 0.0
    feet_z = asset.data.body_pos_w[:, asset_cfg.body_ids, 2]

    # 只奖励摆动期的脚抬高程度
    lift_amount = torch.square(feet_z - 0.20)
    reward = (in_air.float() * lift_amount).sum(dim=1)
    return reward

def feet_swing(env: BaseEnv, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    left_swing = (torch.abs(env.gait_process - 0.25) < 0.5 * 0.2) & (env.gait_frequency > 1.0e-8)
    right_swing = (torch.abs(env.gait_process - 0.75) < 0.5 * 0.2) & (env.gait_frequency > 1.0e-8)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    feet_contact = contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, 2] > 3
    return (left_swing & ~feet_contact[:, 0]).float() + (right_swing & ~feet_contact[:, 1]).float()

def feet_clock_vel(
    env:BaseEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """ Reward for the velocity of the feet during the gait cycle """
    stance_mask = (env.leg_phase[:, :] < 0.6).int()
    # print(stance_mask)
    swing_mask = -1 * (1 - stance_mask)
    # stance_mask = -1, swing_mask = 1 (reverse of feet_clock_frc)
    stance_swing_mask = stance_mask + swing_mask
    stance_swing_mask *= -1
    asset = env.scene[asset_cfg.name]
    max_vel = torch.tensor(0.3, device="cuda")
    body_vel = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :]
    normed_vel = torch.min(body_vel.norm(p=2, dim=-1), max_vel) / max_vel
    rew_normed_vel = normed_vel * stance_swing_mask

    # print(stance_swing_mask, normed_vel, rew_normed_vel.mean(dim=1))

    return rew_normed_vel.mean(dim=1)

# ************************************  task  *************************************
def stand_still_joint_vel(
    env:BaseEnv, command_name: str, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    command = env.command_generator.command
    stand_still = (torch.norm(command[:, :3], dim=1) <= 0.1)
    jnt_vel_err = 0.01 * torch.sum(asset.data.joint_vel**2, dim=1)
    reward = torch.where(stand_still, jnt_vel_err, torch.zeros_like(jnt_vel_err))
    return reward

def joint_action_l1(
    env: BaseEnv,asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """
    L1 penalty (‖a‖₁) on the actions of *any* joint subset.
    给稍大一点负权重，它会迅速被“锁”在 0 附近，对小动作敏感
    """
    return torch.sum(torch.abs(env.action[:, asset_cfg.joint_ids]), dim=1)

def torque_limits(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"), soft_torque_limit: float = 0.9) -> torch.Tensor:
    """惩罚扭矩接近限制。"""
    torques = env.scene[asset_cfg.name].data.applied_torque
    torque_limits = env.scene[asset_cfg.name].data.joint_effort_limits
    over_limit = (torch.abs(torques) - torque_limits * soft_torque_limit).clip(min=0.)
    return torch.sum(over_limit, dim=1)

def dof_vel_limits(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"), soft_dof_vel_limit: float = 0.9) -> torch.Tensor:
    """惩罚关节速度接近限制。"""
    dof_vel = env.scene[asset_cfg.name].data.joint_vel
    dof_vel_limits = env.scene[asset_cfg.name].data.joint_vel_limits
    over_limit = (torch.abs(dof_vel) - dof_vel_limits * soft_dof_vel_limit).clip(min=0.)
    return torch.sum(over_limit, dim=1)