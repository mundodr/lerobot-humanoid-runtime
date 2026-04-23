# lerobot_humanoid_runtime

Runtime stack for deploying RL locomotion policies on the 12-DOF bipedal humanoid (no arms), with both real-hardware and MuJoCo backends.

## Repository layout

```text
lerobot_humanoid_runtime/
├── apps/
│   └── gamepad_controller.py
├── control/
│   ├── RL_agent_isolated.py
│   └── policy/
├── hardware/
│   ├── mit_codec.py
│   └── robstride_toolkit.py
├── imu/
│   ├── IMU_integration.py
│   ├── IMU_JY901.py
│   └── IMU_BNO055
├── robot/
│   ├── bipedal_robot.py
│   ├── root_constant.py
│   ├── sim_robot.py
│   └── lerobot-humanoid-model/models/bipedal_plateform_no_arms/
├── logs/
├── hip_imu_debug.py
├── measure_actuator_delay.py
├── replay_log.py
├── launch_meshcat_browser.py
├── mock_bus.py
└── ipython_helper.py
```

## Import conventions

Use package imports from repo root:

```python
from robot.bipedal_robot import BipedalRobotController
from robot.sim_robot import SimBipedalRobotController
from control.RL_agent_isolated import RLAgent
from imu.IMU_integration import IMU
from apps.gamepad_controller import GamepadController
```

All internal imports were updated to this layout.

## Core modules

- `robot.bipedal_robot.BipedalRobotController`: real CAN controller.
- `robot.sim_robot.SimBipedalRobotController`: MuJoCo simulator with the same control API.
- `control.RL_agent_isolated.RLAgent`: policy runner (ONNX/Torch), observation assembly, action dispatch.
- `imu.IMU_integration.IMU`: IMU backend selector (`bno085`, `bno055`, `jy901`, or mock).
- `apps.gamepad_controller.GamepadController`: Linux gamepad command source.

## Policies

Policies are stored in `control/policy/<policy_name>/` with:

- `policy.onnx`
- `config.yaml` (or another config file the agent loader can parse)
- `gain.md`

Example:

```python
from control.RL_agent_isolated import RLAgent
from robot.bipedal_robot import BipedalRobotController

robot = BipedalRobotController(control_hz=100.0)
agent = RLAgent.from_files(
    robot,
    config_path="control/policy/tao_iteration/config.yaml",
    policy_path="control/policy/tao_iteration/policy.onnx",
    log_path="logs/tao_iteration_debug_ctrl.csv",
)
```

## Quick start helpers (IPython)

`ipython_helper.py` now provides minimal, maintained helpers:

- `make_real_robot(...)`
- `make_sim_robot(...)`
- `make_gamepad(...)`
- `make_rl_agent(...)`
- `start_sim_policy_session(...)`
- `start_real_policy_session(...)`
- `stop_session(...)`

Example:

```python
from ipython_helper import start_sim_policy_session, stop_session

robot, pad, agent = start_sim_policy_session(policy_name="tao_iteration")
# ...
stop_session(robot=robot, gamepad=pad, agent=agent)
```

## Logs and generated files

- Runtime traces and debug CSVs should go in `logs/`.
- Generated caches (`__pycache__/`) are not part of source and can be deleted safely.

## Naming cleanup suggestions (non-breaking plan)

Current names kept for compatibility (especially model/policy artifacts), but recommended future migration:

1. `robot/bipdeal_config.py` -> `robot/bipedal_config_legacy.py`
2. `robot/lerobot-humanoid-model/models/bipedal_plateform_no_arms` -> `robot/lerobot-humanoid-model/models/bipedal_platform_no_arms`
3. `control/policy/less_noice_*` -> `control/policy/less_noise_*`

If you migrate these names, do it in one commit with a global path update and keep temporary compatibility aliases.

## Notes

- `robot/root_constant.py` now resolves model paths from `robot/lerobot-humanoid-model/models/bipedal_plateform_no_arms/...`.
- `robot/sim_robot.py` now uses the SB model scene at `robot/lerobot-humanoid-model/models/bipedal_plateform_no_arms/mjcf/scene.xml`.
