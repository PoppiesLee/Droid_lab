from legged_lab.envs.base.base_env import BaseEnv
from legged_lab.envs.base.base_env_config import BaseEnvCfg, BaseAgentCfg
from legged_lab.envs.droid.X03B_config import X03BFlatEnvCfg, X03BRoughEnvCfg, X03BFlatAgentCfg, X03BRoughAgentCfg
from legged_lab.envs.droid.x3_config import x3FlatEnvCfg, x3RoughEnvCfg, x3FlatAgentCfg, x3RoughAgentCfg
from legged_lab.envs.droid.X03_config import X03FlatEnvCfg, X03RoughEnvCfg, X03FlatAgentCfg, X03RoughAgentCfg
from legged_lab.envs.droid.x02a_config import X02AFlatEnvCfg, X02ARoughEnvCfg, X02AFlatAgentCfg, X02ARoughAgentCfg
from legged_lab.envs.droid.x2_config import X2FlatEnvCfg, X2RoughEnvCfg, X2FlatAgentCfg, X2RoughAgentCfg
from legged_lab.envs.droid.x2r_config import X2RFlatEnvCfg, X2RRoughEnvCfg, X2RFlatAgentCfg, X2RRoughAgentCfg
from legged_lab.envs.droid.x2c_config import X2CFlatEnvCfg, X2CRoughEnvCfg, X2CFlatAgentCfg, X2CRoughAgentCfg
from legged_lab.envs.droid.E1_config import E1FlatEnvCfg, E1RoughEnvCfg, E1FlatAgentCfg, E1RoughAgentCfg
from legged_lab.envs.droid.G1_config import G1FlatEnvCfg, G1RoughEnvCfg, G1FlatAgentCfg, G1RoughAgentCfg
from legged_lab.envs.anymal_d.anymal_d_config import AnymalDFlatEnvCfg, AnymalDRoughEnvCfg, AnymalDFlatAgentCfg, AnymalDRoughAgentCfg
from legged_lab.utils.task_registry import task_registry

task_registry.register("X03_flat", BaseEnv, X03FlatEnvCfg(), X03FlatAgentCfg())
task_registry.register("X03_rough", BaseEnv, X03RoughEnvCfg(), X03RoughAgentCfg())
task_registry.register("X03B_flat", BaseEnv, X03BFlatEnvCfg(), X03BFlatAgentCfg())
task_registry.register("X03B_rough", BaseEnv, X03BRoughEnvCfg(), X03BRoughAgentCfg())
task_registry.register("x3_flat", BaseEnv, x3FlatEnvCfg(), x3FlatAgentCfg())
task_registry.register("x3_rough", BaseEnv, x3RoughEnvCfg(), x3RoughAgentCfg())
task_registry.register("x02a_flat", BaseEnv, X02AFlatEnvCfg(), X02AFlatAgentCfg())
task_registry.register("x02a_rough", BaseEnv, X02ARoughEnvCfg(), X02ARoughAgentCfg())
task_registry.register("x2_flat", BaseEnv, X2FlatEnvCfg(), X2FlatAgentCfg())
task_registry.register("x2_rough", BaseEnv, X2RoughEnvCfg(), X2RoughAgentCfg())
task_registry.register("x2r_flat", BaseEnv, X2RFlatEnvCfg(), X2RFlatAgentCfg())
task_registry.register("x2r_rough", BaseEnv, X2RRoughEnvCfg(), X2RRoughAgentCfg())
task_registry.register("X2C_flat", BaseEnv, X2CFlatEnvCfg(), X2CFlatAgentCfg())
task_registry.register("X2C_rough", BaseEnv, X2CRoughEnvCfg(), X2CRoughAgentCfg())
task_registry.register("E1_flat", BaseEnv, E1FlatEnvCfg(), E1FlatAgentCfg())
task_registry.register("E1_rough", BaseEnv, E1RoughEnvCfg(), E1RoughAgentCfg())
task_registry.register("G1_flat", BaseEnv, G1FlatEnvCfg(), G1FlatAgentCfg())
task_registry.register("G1_rough", BaseEnv, G1RoughEnvCfg(), G1RoughAgentCfg())
task_registry.register("anymal_d_flat", BaseEnv, AnymalDFlatEnvCfg(), AnymalDFlatAgentCfg())
task_registry.register("anymal_d_rough", BaseEnv, AnymalDRoughEnvCfg(), AnymalDRoughAgentCfg())
