# Droid Lab: Direct IsaacLab Workflow for Legged Robots

[![IsaacSim](https://img.shields.io/badge/IsaacSim-5.1.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-2.3.2-silver)](https://isaac-sim.github.io/IsaacLab)
[![RSL_RK](https://img.shields.io/badge/RSL_RL-2.3.3-silver)](https://github.com/leggedrobotics/rsl_rl)
[![Python](https://img.shields.io/badge/python-3.11-blue.svg)](https://docs.python.org/3/whatsnew/3.11.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://pre-commit.com/)
[![License](https://img.shields.io/badge/license-MIT-yellow.svg)](https://opensource.org/license/mit)

## Overview

This repository provides a direct workflow for training a legged robot using IsaacLab. It provides high transparency and low refactoring difficulty of the direct environment, and uses isaaclab components to simplify the workflow.

It possesses a complete framework from training to deployment, supporting direct deployment of policies via sim2sim and sim2real methods. This framework has been perfectly validated on the DroidUP E1 bipedal robot and the E1 Dog quadrupedal robot.


**Maintainer**: JiaLong Li              
**Contact**: ljialong218@gmail.com    

**Key Features:**

- `Easy to Reorganize` Provides a direct workflow, allowing for fine-grained definition of environment logic.
- `Isolation` Work outside the core Isaac Lab repository, ensuring that the development efforts remain self-contained.
- `Long-term support` This repository will be updated with the updates of isaac sim and isaac lab, and will be supported for a long time.
- `Full process` This framework supports the entire deployment process from robot training to sim2sim and sim2real.

If you use Droid Lab in your research, please cite it as follows:

```bibtex
@software{Droid Lab,
  author = {JiaLong, Li},
  license = {MIT},
  title = {Droid Lab: Direct IsaacLab Workflow for Legged Robots},
  url = {https://github.com/PoppiesLee/Droid_lab},
  version = {1.0.0},
  year = {2026}
}
```

## Installation

- Install Isaac Lab by following the [installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html). We recommend using the conda installation as it simplifies calling Python scripts from the terminal.

- Clone this repository separately from the Isaac Lab installation (i.e. outside the `IsaacLab` directory):

```bash
 # Option 1: HTTPS
git clone https://github.com/PoppiesLee/Droid_lab.git

# Option 2: SSH
git clone git@github.com:PoppiesLee/Droid_lab.git
```

- Using a python interpreter that has Isaac Lab installed, install the library

```bash
cd leggedlab
pip install -e .
```

- Verify that the extension is correctly installed by running the following command:

```bash
python legged_lab/scripts/train.py --task=E1_flat --headless --logger=tensorboard --num_envs=4096
python legged_lab/scripts/play.py --task=E1_flat  --logger=tensorboard --num_envs=1
```


## Use Your Own Robot

You can use any format of robot model file, such as USD, URDF.

## Troubleshooting

### Pylance Missing Indexing of Extensions

In some VsCode versions, the indexing of part of the extensions is missing. In this case, add the path to your extension in `.vscode/settings.json` under the key `"python.analysis.extraPaths"`.

```json
{
    "python.analysis.extraPaths": [
        "${workspaceFolder}/legged_lab",
        "<path-to-IsaacLab>/source/isaaclab_tasks",
        "<path-to-IsaacLab>/source/isaaclab_mimic",
        "<path-to-IsaacLab>/source/extensions",
        "<path-to-IsaacLab>/source/isaaclab_assets",
        "<path-to-IsaacLab>/source/isaaclab_rl",
        "<path-to-IsaacLab>/source/isaaclab",
    ]
}
```

# References and Thanks
This project repository builds upon the shoulders of giants. 
* [IsaacLab](https://github.com/isaac-sim/IsaacLab)   The various reusable practical components in IsaacLab greatly simplify the complexity of LeggedLab.
* [legged_lab](https://github.com/Hellod035/LeggedLab)   We borrowed the code organization and environment definition logic of legged_lab and simplified it as much as possible.
* [Protomotions](https://github.com/NVlabs/ProtoMotions)   The motivation for building this repository comes from protomotions. For the first time, we realized that we could create our own environment using only IsaacLab components without inheriting 'DirectRLEnv' or 'ManagerBasedRLEnv'.
