# Copyright (c) 2024 Droid AI Institute LLC. All rights reserved.

import json
import os
import re
import numpy as np
from typing import List
from dataclasses import dataclass


def dict_from_lists(keys: List, values: List):
    """construct a dict from two lists where keys and associated values appear at same index"""
    return dict(zip(keys, values))


def dict_to_array(data: dict, keys: List):
    """construct a list of values from dictionary in the order specified by keys"""
    values = [data.get(key) for key in keys]
    return np.array(values)


def set_matching(data: dict, regex, value):
    """set values in dict with keys matching regex"""
    for key in data:
        if regex.match(key):
            if value is None:
                value = 0.0
            data[key] = value


@dataclass
class IsaacLabCfg:
    """dataclass holding data extracted from orbits training configuration"""
    dof_stiffness: np.array(float)
    dof_damping: np.array(float)
    effort_limit: np.array(float)
    default_joints: np.array(float)
    standing_height: float
    action_scale: float
    dt: float
    hist_length: int
    decimation: int


def load_configuration(file: os.PathLike, mujoco_joint_order) -> IsaacLabCfg:
    """parse json file and populate an IsaacLabCfg dataclass"""
    num_actions = len(mujoco_joint_order)
    joint_kp = dict_from_lists(mujoco_joint_order, [None] * num_actions)
    joint_kd = dict_from_lists(mujoco_joint_order, [None] * num_actions)
    effort_limit = dict_from_lists(mujoco_joint_order, [None] * num_actions)
    velocity_limit = dict_from_lists(mujoco_joint_order, [None] * num_actions)
    armature = dict_from_lists(mujoco_joint_order, [None] * num_actions)
    friction = dict_from_lists(mujoco_joint_order, [None] * num_actions)
    joint_offsets = dict_from_lists(mujoco_joint_order, [None] * num_actions)

    with open(file) as f:
        env_config = json.load(f)

        # NOTE: Updated path to actuators for new JSON structure
        actuators = env_config["scene"]["robot"]["actuators"]

        for group in actuators.keys():
            group_joint_num = len(actuators[group]['joint_names_expr'])
            for i in range(group_joint_num):
                temp_name = actuators[group]["joint_names_expr"][i]
                regex = re.compile(temp_name)

                # Using set_matching to map regex keys in JSON to specific joints
                set_matching(joint_kp, regex, actuators[group]["stiffness"][temp_name])
                set_matching(joint_kd, regex, actuators[group]["damping"][temp_name])

                # NOTE: JSON uses _sim suffix for limits
                set_matching(effort_limit, regex, actuators[group]["effort_limit_sim"][temp_name])
                set_matching(velocity_limit, regex, actuators[group]["velocity_limit_sim"][temp_name])
                set_matching(armature, regex, actuators[group]["armature"][temp_name])

                # Friction is sometimes null or flat value, handling varies but strictly following previous logic:
                # If friction is a dict/value inside the group, we try to access it.
                # In provided JSON, friction is null in actuators.
                # If it's null, set_matching handles None -> 0.0 inside itself if passed explicitly,
                # but here we access the key. We use .get to be safe.
                friction_val = actuators[group].get("friction")
                set_matching(friction, regex, friction_val)

        # NOTE: Updated path for init_state
        default_joint_data = env_config["scene"]["robot"]["init_state"]["joint_pos"]
        default_joint_expressions = default_joint_data.keys()
        for expression in default_joint_expressions:
            regex = re.compile(expression)
            set_matching(joint_offsets, regex, default_joint_data[expression])

        # NOTE: Updated path for pos
        standing_height = env_config["scene"]["robot"]["init_state"]["pos"][2]

        # NOTE: Updated path for action scale (moved to actions -> JointPositionAction)
        action_scale = env_config["actions"]["JointPositionAction"]["scale"]

        # NOTE: Updated path for history length (observations -> policy)
        # In the new JSON, history_length is null. We default to 0 if None to satisfy int type.
        hist_length = env_config["observations"]["policy"].get("history_length")
        if hist_length is None:
            hist_length = 0

        dt = env_config["sim"]["dt"]

        # NOTE: Decimation is now at the top level
        decimation = env_config["decimation"]

    print("standing_height:", standing_height)
    print("action_scale:", action_scale)
    print("dt:", dt)
    print("decimation:", decimation)
    print("update Cycle(decimation * dt): %d ms" % (decimation * dt * 1000))

    print("\033[32m>>The robot: Config Loaded with %d dof, DofProperties information as follow:\033[0m" % (num_actions))
    print(
        "\033[32m+--------------------+-----+----------------+--------------+----------------+-----------+---------+----------+----------+\033[0m")
    print(
        "\033[33m|     Joint names    | idx |   default_pos  | effort_limit | velocity_limit | stiffness | damping | armature | friction |\033[0m")
    print(
        "\033[32m+--------------------+-----+----------------+--------------+----------------+-----------+---------+----------+----------+\033[0m")
    for i in range(0, num_actions):
        temp_name = mujoco_joint_order[i]

        # Safety check for None values before printing to avoid formatting errors
        d_pos = joint_offsets[temp_name] if joint_offsets[temp_name] is not None else 0.0
        eff_lim = effort_limit[temp_name] if effort_limit[temp_name] is not None else 0.0
        vel_lim = velocity_limit[temp_name] if velocity_limit[temp_name] is not None else 0.0
        kp = joint_kp[temp_name] if joint_kp[temp_name] is not None else 0.0
        kd = joint_kd[temp_name] if joint_kd[temp_name] is not None else 0.0
        arm = armature[temp_name] if armature[temp_name] is not None else 0.0
        fric = friction[temp_name] if friction[temp_name] is not None else 0.0

        print("| %-19s|  %2d | %5.2f(%7.2f) |   %8.2f   |    %8.2f    | %6.1f    | %5.1f   | %8.4f | %8.4f |" % (
            temp_name, i, d_pos, np.rad2deg(d_pos),
            eff_lim, vel_lim,
            kp, kd,
            arm, fric))
    print(
        "\033[32m+--------------------+-----+----------------+---------+-----------+--------------+----------------+----------+----------+\033[0m")

    return IsaacLabCfg(
        dof_stiffness=dict_to_array(joint_kp, mujoco_joint_order),
        dof_damping=dict_to_array(joint_kd, mujoco_joint_order),
        effort_limit=dict_to_array(effort_limit, mujoco_joint_order),
        default_joints=dict_to_array(joint_offsets, mujoco_joint_order),
        standing_height=standing_height,
        action_scale=action_scale,
        dt=dt,
        hist_length=hist_length,
        decimation=decimation
    )