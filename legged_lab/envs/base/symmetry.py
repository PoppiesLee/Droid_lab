import torch

@torch.no_grad()
def get_symmetric_states(obs=None, actions=None, env=None, obs_type='policy'):
    """
    Apply symmetry transformation to observations and/or actions.
    Supports:
        - policy obs: [B, 780] (10×78)
        - critic obs: [B, 830] (10×83 = 10×(78 + 3 + 2))
        - actions:    [B, 20]  (interleaved L/R)
    """
    if obs is None and actions is None:
        raise ValueError("At least one of `obs` or `actions` must be provided.")

    device = obs.device if obs is not None else actions.device
    num_stack = 10
    frame_feat_actor  = 50  # per-frame actor features (policy)
    frame_feat_critic = 55  # per-frame critic features: 78 (actor-like) + 3 (root vel) + 2 (contact)

    # L/R permutation for 12-DoF blocks 摆动关节
    swap_joint_idx = [1, 0, 2, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11]

    # === SHARED actor obs permutation + sign ===
    flip_idx_1f = list(range(frame_feat_actor))
    flip_sign_1f = [1.0] * frame_feat_actor
    # waist joints:
    """ 12 = 3(root_lin_vel) + 3(ang_vel) + 3(projected_gravity) + 3(command)"""
    flip_sign_1f[11 + 2] = -1.0  # waist_yaw pos
    # hip_roll, hip_yaw, ankle_roll,shoulder_roll, shoulder_yaw 的旋转方向也需取反
    for idx in [3, 4, 5, 6, 11, 12]:
        flip_sign_1f[11 + idx] = -1.0
    flip_sign_1f[24 + 2] = -1.0  # waist_yaw vel
    for idx in [3, 4, 5, 6, 11, 12]:
        flip_sign_1f[24 + idx] = -1.0

    # (0.2) lin_vel 未使用；下方从 3 开始设置角速度等


    # ang_vel x/yaw
    flip_sign_1f[0] = -1.0
    flip_sign_1f[2] = -1.0
    # project grivaty roll
    flip_sign_1f[4] = -1.0
    # command vx
    flip_sign_1f[6] = 1.0
    flip_sign_1f[7] = -1.0
    flip_sign_1f[8] = -1.0
    # joint swap
    joint_pos_perm = [idx + 11 for idx in swap_joint_idx] # 关节位置左右对调
    joint_vel_perm = [idx + 24 for idx in swap_joint_idx] # 关节速度左右对调

    for i in range(13):
        flip_idx_1f[11 + i] = joint_pos_perm[i]
        flip_idx_1f[24 + i] = joint_vel_perm[i]
    # action block in obs
    action_perm = swap_joint_idx
    for i in range(13):
        flip_idx_1f[37 + i] = 37 + action_perm[i]


    # === 动作符号反转 ===
    # 这些关节的动作在镜像时需要取反（waist、roll/yaw）
    neg_action_idx = [2, 3, 4, 5, 6, 11,12]
    for idx in neg_action_idx:
        flip_sign_1f[37 + idx] = -1.0
    #phase swap; sin/cos phase
    sign_perm = [1+9, 0+9]
    for i in range(2):
        flip_idx_1f[9+i] = sign_perm[i]

    # Stack over time
    flip_idx = []
    flip_sign = []
    # print(flip_idx_1f)
    for t in range(num_stack):
        offset = t * frame_feat_actor
        flip_idx += [offset + i for i in flip_idx_1f]
        flip_sign += flip_sign_1f
    # print(flip_idx)
    flip_idx_tensor = torch.tensor(flip_idx, dtype=torch.long, device=device)
    flip_sign_tensor = torch.tensor(flip_sign, dtype=torch.float32, device=device)
    action_perm_tensor = torch.tensor(action_perm, dtype=torch.long, device=device)

    obs_out = None
    actions_out = None
    # === ONLY ACTIONS (mirror loss)
    if obs is None and actions is not None and obs_type == 'policy':
        # actions_sym = actions[:, action_perm_tensor]
        # 为动作镜像也添加符号反转
        action_sign = torch.ones(13, device=device)
        action_sign[torch.tensor( [2, 3, 4, 5, 6, 11,12], device=device)] = -1.0
        actions_sym = actions[:, action_perm_tensor] * action_sign

        actions_out = torch.cat([actions, actions_sym], dim=0)
        return None, actions_out

    # === POLICY OBS + ACTION
    if obs_type == 'policy' and obs is not None:
        obs_sym = obs[:, flip_idx_tensor] * flip_sign_tensor
        obs_out = torch.cat([obs, obs_sym], dim=0)

        if actions is not None:
            # actions_sym = actions[:, action_perm_tensor]
            # 为动作镜像也添加符号反转
            action_sign = torch.ones(13, device=device)
            action_sign[torch.tensor( [2, 3, 4, 5, 6, 11,12], device=device)] = -1.0
            actions_sym = actions[:, action_perm_tensor] * action_sign

            actions_out = torch.cat([actions, actions_sym], dim=0)

        return obs_out, actions_out

    # === CRITIC OBS ONLY
    if obs_type == 'critic' and obs is not None:
        if obs.shape[1] != frame_feat_critic * num_stack:
            raise ValueError(f"Expected critic obs shape of  = {num_stack}×{frame_feat_critic}, got {obs.shape[1]}")

        B = obs.shape[0]
        obs_reshaped = obs.view(B, num_stack, frame_feat_critic)

        # 拆分：actor-like(78) + root_vel(3) + contact(2)
        actor_like = obs_reshaped[:, :, :frame_feat_actor]  # [B, 10, 78]
        root_vel = obs_reshaped[:, :, frame_feat_actor:frame_feat_actor + 3]  # [B, 10, 3]  -> [vx, vy, vyaw]
        contact = obs_reshaped[:, :, frame_feat_actor + 3:frame_feat_actor + 5]  # [B, 10, 2]  -> [L, R]

        # 1) actor-like 做镜像（逐帧应用 1f 映射）
        actor_like_flat = actor_like.reshape(B, -1)  # [B, 780]
        actor_like_sym = (
                actor_like_flat[:, flip_idx_tensor] * flip_sign_tensor
        ).view(B, num_stack, frame_feat_actor)

        # 2) root vel：vy
        root_vel_sym = root_vel.clone()
        root_vel_sym[..., 1] *= -1.0  # vy
        # root_vel_sym[..., 2] *= -1.0  # vz

        # 3) contact：**交换左右**（而不是 1-contact）
        contact_sym = contact[..., [1, 0]]  # [L, R] -> [R, L]

        # 4) 重新拼接并 reshape 回去
        obs_sym = torch.cat([actor_like_sym ,root_vel_sym, contact_sym ], dim=-1)  # [B, 10, 83]
        obs_sym = obs_sym.view(B, -1)  # [B, 830]

        obs_out = torch.cat([obs, obs_sym], dim=0)  # [2B, 830]
        return obs_out, None


    raise ValueError(f"Unsupported obs_type: {obs_type}. Expected 'policy' or 'critic'.")