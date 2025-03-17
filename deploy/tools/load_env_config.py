# Copyright (c) 2024 Droid AI Institute LLC. All rights reserved.

import json
import os
import re
import numpy as np
from typing import List
from dataclasses import dataclass


def dict_from_lists(keys: List, values: List):
    """construct a dict from two lists where keys and associated values appear at same index

    arguments
    keys -- list of keys
    values -- list of values in same order as keys

    return dictionary mapping keys to values
    """

    return dict(zip(keys, values))


def dict_to_array(data: dict, keys: List):
    """construct a list of values from dictionary in the order specified by keys

    arguments
    dict -- dictionary to look up keys in
    keys -- list of keys to retrieve in orer

    return array of values from dict in same order as keys
    """
    values = [data.get(key) for key in keys]
    return np.array(values)


def set_matching(data: dict, regex, value):
    """set values in dict with keys matching regex

    arguments
    dict -- dictionary to set keys in
    regex -- regex to select keys that will be set
    value -- value to set keys matching regex to
    """
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
    """parse json file and populate an IsaacLabCfg dataclass

    arguments
    file -- the path to the json file containing training configuration

    return IsaacLabCfg containing needed training configuration
    """
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
        actuators = env_config["scene"]["robot"]["actuators"]
        for group in actuators.keys():
            group_joint_num = len(actuators[group]['joint_names_expr'])
            for i in range(group_joint_num):
                temp_name = actuators[group]["joint_names_expr"][i]
                regex = re.compile(temp_name)
                set_matching(joint_kp, regex, actuators[group]["stiffness"][temp_name])
                set_matching(joint_kd, regex, actuators[group]["damping"][temp_name])
                set_matching(effort_limit, regex, actuators[group]["effort_limit"][temp_name])
                set_matching(velocity_limit, regex, actuators[group]["velocity_limit"][temp_name])
                set_matching(armature, regex, actuators[group]["armature"][temp_name])
                set_matching(friction, regex, actuators[group]["friction"])

        default_joint_data = env_config["scene"]["robot"]["init_state"]["joint_pos"]
        default_joint_expressions = default_joint_data.keys()
        for expression in default_joint_expressions:
            regex = re.compile(expression)
            set_matching(joint_offsets, regex, default_joint_data[expression])

        standing_height = env_config["scene"]["robot"]["init_state"]["pos"][2]
        action_scale = env_config["robot"]["action_scale"]
        hist_length = env_config["robot"]["actor_obs_history_length"]
        dt = env_config["sim"]["dt"]
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = env_config["sim"]["decimation"]

    print("standing_height:", standing_height)
    print("action_scale:", action_scale)
    print("dt:", dt)
    print("decimation:", decimation)
    print("update Cycle(decimation * dt): %d ms" % (decimation * dt * 1000))

    print("\033[32m>>The robot: X02Lite with %d dof, DofProperties information as follow:\033[0m" % (num_actions))
    print("\033[32m+--------------------+-----+----------------+--------------+----------------+-----------+---------+----------+----------+\033[0m")
    print("\033[33m|     Joint names    | idx |   default_pos  | effort_limit | velocity_limit | stiffness | damping | armature | friction |\033[0m")
    print("\033[32m+--------------------+-----+----------------+--------------+----------------+-----------+---------+----------+----------+\033[0m")
    for i in range(0, num_actions):
        temp_name = mujoco_joint_order[i]
        print("| %-19s|  %2d | %5.2f(%7.2f) |   %8.2f   |    %8.2f    | %6.1f    | %5.1f   | %8.4f | %8.4f |" % (
                temp_name, i, joint_offsets[temp_name], np.rad2deg(joint_offsets[temp_name]),
                effort_limit[temp_name], velocity_limit[temp_name],
                joint_kp[temp_name], joint_kd[temp_name],
                armature[temp_name], friction[temp_name]))
    print("\033[32m+--------------------+-----+----------------+---------+-----------+--------------+----------------+----------+----------+\033[0m")

    return IsaacLabCfg(
        dof_stiffness=dict_to_array(joint_kp, mujoco_joint_order),
        dof_damping=dict_to_array(joint_kd, mujoco_joint_order),
        effort_limit=dict_to_array(effort_limit, mujoco_joint_order),
        default_joints=dict_to_array(joint_offsets, mujoco_joint_order),
        standing_height=standing_height,
        action_scale=action_scale,
        dt=dt,
        hist_length = hist_length,
        decimation=decimation
    )