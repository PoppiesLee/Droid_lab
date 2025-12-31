from legged_lab.envs.base.base_env import BaseEnv
from legged_lab.envs.base.base_env_config import BaseEnvCfg, BaseAgentCfg
from legged_lab.envs.droid.E1_config import E1FlatEnvCfg, E1RoughEnvCfg, E1FlatAgentCfg, E1RoughAgentCfg
from legged_lab.envs.droid.E1_DOG_config import E1_DOGFlatEnvCfg, E1_DOGRoughEnvCfg, E1_DOGFlatAgentCfg, E1_DOGRoughAgentCfg
from legged_lab.envs.droid.G1_config import G1FlatEnvCfg, G1RoughEnvCfg, G1FlatAgentCfg, G1RoughAgentCfg
from legged_lab.envs.anymal_d.anymal_d_config import AnymalDFlatEnvCfg, AnymalDRoughEnvCfg, AnymalDFlatAgentCfg, AnymalDRoughAgentCfg
from legged_lab.utils.task_registry import task_registry

task_registry.register("E1_flat", BaseEnv, E1FlatEnvCfg(), E1FlatAgentCfg())
task_registry.register("E1_rough", BaseEnv, E1RoughEnvCfg(), E1RoughAgentCfg())
task_registry.register("E1_DOG_flat", BaseEnv, E1_DOGFlatEnvCfg(), E1_DOGFlatAgentCfg())
task_registry.register("E1_DOG_rough", BaseEnv, E1_DOGRoughEnvCfg(), E1_DOGRoughAgentCfg())
task_registry.register("G1_flat", BaseEnv, G1FlatEnvCfg(), G1FlatAgentCfg())
task_registry.register("G1_rough", BaseEnv, G1RoughEnvCfg(), G1RoughAgentCfg())
task_registry.register("anymal_d_flat", BaseEnv, AnymalDFlatEnvCfg(), AnymalDFlatAgentCfg())
task_registry.register("anymal_d_rough", BaseEnv, AnymalDRoughEnvCfg(), AnymalDRoughAgentCfg())
