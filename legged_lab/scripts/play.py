import os
import torch
from legged_lab.utils import task_registry

import argparse

from isaaclab.app import AppLauncher
from rsl_rl.runners import OnPolicyRunner
# local imports
import legged_lab.utils.cli_args as cli_args  # isort: skip
from deploy.tools.update_policies import PolicyManager

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")

# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaaclab_rl.rsl_rl import export_policy_as_jit, export_policy_as_onnx

from legged_lab.envs import *  # noqa:F401, F403
from legged_lab.utils.cli_args import update_rsl_rl_cfg
from isaaclab_tasks.utils import get_checkpoint_path


def play():
    runner: OnPolicyRunner
    env_cfg: BaseEnvCfg  # noqa:F405

    env_class_name = args_cli.task
    env_cfg, agent_cfg = task_registry.get_cfgs(env_class_name)

    env_cfg.noise.add_noise = False
    env_cfg.domain_rand.push_robot.enable = False
    env_cfg.scene.max_episode_length_s = 40.
    env_cfg.scene.num_envs = 50
    env_cfg.scene.env_spacing = 2.5
    env_cfg.commands.ranges.lin_vel_x = (0.6, 0.6)
    env_cfg.commands.ranges.lin_vel_y = (0.0, 0.0)
    env_cfg.commands.ranges.heading = (0.0, 0.0)
    # env_cfg.scene.height_scanner.drift_range = (0.0, 0.0)

    # env_cfg.scene.terrain_generator = None
    # env_cfg.scene.terrain_type = "plane"

    if env_cfg.scene.terrain_generator is not None:
        env_cfg.scene.terrain_generator.num_rows = 5
        env_cfg.scene.terrain_generator.num_cols = 5
        env_cfg.scene.terrain_generator.curriculum = False
        env_cfg.scene.terrain_generator.difficulty_range = (0.4, 0.4)

    if args_cli.num_envs is not None:
        env_cfg.scene.num_envs = args_cli.num_envs

    env_class = task_registry.get_task_class(env_class_name)
    env = env_class(env_cfg, args_cli.headless)

    agent_cfg = update_rsl_rl_cfg(agent_cfg, args_cli)
    log_root_path = os.path.join("logs", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)
    log_dir = os.path.dirname(resume_path)

    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
    runner.load(resume_path, load_optimizer=False)

    policy = runner.get_inference_policy(device=env.device)

    export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")
    export_policy_as_jit(runner.alg.actor_critic, runner.obs_normalizer, path=export_model_dir, filename="policy.pt")
    export_policy_as_onnx(runner.alg.actor_critic, normalizer=runner.obs_normalizer, path=export_model_dir, filename="policy.onnx")
    # 去掉路径末尾的斜杠（如果存在）
    temp_path = export_model_dir.rstrip("/")
    # 去掉路径中的最后一部分
    log_path = os.path.dirname(temp_path)
    policies_destination_path = "legged_lab/../deploy/policies"

    manager = PolicyManager(log_path, policies_destination_path)
    manager.run()

    if not args_cli.headless:
        from legged_lab.utils.keyboard import Keyboard
        keyboard = Keyboard(env)  # noqa:F841

    obs, _ = env.get_observations()

    while simulation_app.is_running():

        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)


if __name__ == '__main__':
    play()
    simulation_app.close()
