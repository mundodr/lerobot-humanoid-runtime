# lerobot_humanoid_runtime

Runtime and calibration stack for a 12-DOF bipedal humanoid (no arms), with:
- simulation control (MuJoCo),
- real robot control (CAN + IMU),
- LeRobot integration controller.

This repository is meant to be used by anyone building this robot. Calibration is mandatory before deployment.

## Safety First

This robot can hurt people and damage itself.

- Always start in `state_only` and check state validity before enabling control.
- Keep a clear physical area around the robot.
- Keep a hardware-level power cutoff ready.
- Do not disable controller safety checks.
- If E-STOP triggers, inspect the reason before retrying.

## Supported Platform

- Raspberry Pi 5
- Ubuntu
- Python 3.13 recommended (minimum 3.12 for current LeRobot versions)
- `uv` for environment management

## What Is In This Repo

- `robot/sim_robot.py`: MuJoCo simulation controller.
- `robot/bipedal_robot.py`: real robot controller (CAN MIT protocol + safety).
- `control/RL_agent_isolated.py`: policy inference runner (ONNX/Torch).
- `imu/IMU_integration.py`: IMU backends (`bno055`, `bno085`, `jy901`, `mock`).
- `apps/gamepad_controller.py`: gamepad command source.
- `lerobot_humanoid_lerobot_integration/`: LeRobot robot implementation for this humanoid.
- `ipython_helper.py`: practical copy/paste snippets for sim/real operation.

## Setup

1. Clone and init submodules:
```bash
git clone <your-repo-url>
cd lerobot_humanoid_runtime
git submodule update --init --recursive
```

2. Install dependencies with `uv`:
```bash
uv sync
```

3. Optional extras:
```bash
uv sync --extra sim
uv sync --extra full
```

## Quick Smoke Test (Simulation)

Use this to validate installation without hardware:

```bash
uv run python - <<'PY'
from robot.sim_robot import SimBipedalRobotController

robot = SimBipedalRobotController(control_hz=200.0, fixed_base=False)
robot.start(mode="control", auto_enable=True)
robot.start_viewer()
print("Simulation started. Close viewer / Ctrl+C to stop.")
PY
```

## Real Robot Bring-Up

### Power and CAN sequence

1. Power robot.
2. Power Raspberry Pi.
3. Bring up CAN if not auto-started:
```bash
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on
```
4. Optionally start MeshCat server (if your setup requires separate server):
```bash
meshcat-server
```

### Recommended staged runner

The staged runner enforces pause points:

```bash
uv run python deploy/run_real_policy_sequential.py --policy-dir control/policy/tao_iteration
```

Stages:
1. Create robot in `state_only`.
2. Check MeshCat / limits / IMU.
3. Switch to `control`, enable motors, send zero pose.
4. Apply gains and start policy.

### IPython workflow (from `ipython_helper.py`)

Use `uv run ipython`, then paste the snippets in [`ipython_helper.py`](/home/virgile/devel/lerobot_humanoid_runtime/ipython_helper.py).

Key flow:
1. Create IMU + `BipedalRobotController`.
2. `robot.start(mode="state_only", auto_enable=False)`.
3. Validate MeshCat and orientation.
4. `robot.set_mode("control")`, `robot.enable_all()`.
5. Start `RLAgent`.

## LeRobot Integration Mode

In-repo integration module:
- [`lerobot_humanoid_lerobot_integration/lerobot_humanoid.py`](/home/virgile/devel/lerobot_humanoid_runtime/lerobot_humanoid_lerobot_integration/lerobot_humanoid.py)
- [`lerobot_humanoid_lerobot_integration/config_lerobot_humanoid.py`](/home/virgile/devel/lerobot_humanoid_runtime/lerobot_humanoid_lerobot_integration/config_lerobot_humanoid.py)

Minimal usage pattern:

```python
from lerobot_humanoid_lerobot_integration import LeRobotHumanoid, LeRobotHumanoidConfig

cfg = LeRobotHumanoidConfig()
robot = LeRobotHumanoid(cfg)
robot.connect()

obs = robot.get_observation()
zero = {name: 0.0 for name in robot.action_features}
robot.send_action(zero)
```

Important:
- Start the robot near its reference posture.
- Current LeRobot controller is less permissive than the classical controller.

### Data Acquisition Script

The identification data acquisition workflow is now included in this repo:
- [`tools/data_acquisition.py`](/home/virgile/devel/lerobot_humanoid_runtime/tools/data_acquisition.py)

Example:

```bash
uv run python tools/data_acquisition.py \
  --fps 100 \
  --total-duration-s 5.0 \
  --command-mode step \
  --amplitudes-deg 0 -5 5 \
  --experiment-name experiment_5s_A
```

## Calibration Guide (Mandatory)

Goal: align internal robot state estimation with real hardware state.

Detailed step-by-step guide:
- [`CALIBRATION.md`](/home/virgile/devel/lerobot_humanoid_runtime/CALIBRATION.md)

### 1) Verify wiring and motor IDs

- Confirm all 12 motors respond on expected bus:
  - `can0`: IDs 1..6
  - `can1`: IDs 7..12
- If one side looks mirrored or swapped in visualization, check wiring first.

### 2) Start in read-only mode

- Use `state_only` mode first.
- Inspect joint states in MeshCat before enabling torque.

### 3) Validate signs and offsets

Primary calibration tables:
- [`robot/root_constant.py`](/home/virgile/devel/lerobot_humanoid_runtime/robot/root_constant.py)
  - `MOTOR_SIGN`
  - `MOTOR_OFFSET_DEG`
  - `JOINT_LIMITS_DEG`
- LeRobot-side constants in [`lerobot_humanoid_lerobot_integration/lerobot_humanoid.py`](/home/virgile/devel/lerobot_humanoid_runtime/lerobot_humanoid_lerobot_integration/lerobot_humanoid.py)

If estimated pose does not match real pose, this is usually wiring/sign/offset mismatch.

### 4) Enable control only after checks

- Switch to `control`.
- Enable motors.
- Send zero pose.
- Command very small joint motion and verify correct direction.

### 5) Keep safety constraints active

Do not remove:
- state bounds checks,
- command jump guards,
- ankle guards,
- E-STOP logic.

## Policy Directory Layout

Each policy directory under `control/policy/<name>/` should contain:
- `policy.onnx` (required),
- one config file among:
  - `config.yaml`
  - `config.yml`
  - `model_25000_env.yaml`
  - `config.json`

## Logging and Debugging

- Robot controller and RL agent can both produce CSV logs.
- Common log locations:
  - policy folder debug CSV (`--log-name`, `--log-path`),
  - custom paths passed to `RLAgent.from_files(...)`.

Use logs to inspect:
- command vs observed joints,
- control loop timing,
- E-STOP events.

## Common Failures

1. Internal state does not match real pose.
   - Usually wiring/sign/offset mismatch.
   - Re-check CAN routing and calibration tables.

2. No motor feedback.
   - CAN interfaces not up.
   - Bring up `can0` / `can1` and retry.

3. E-STOP triggers at startup.
   - Robot out of safe bounds or ankle guard violation.
   - Place robot closer to neutral and retry in `state_only`.

4. MeshCat not updating.
   - Missing visualization dependencies or server not reachable.
   - Check `meshcat` install and `tcp://127.0.0.1:6000` path.

5. LeRobot startup inconsistency vs classical controller.
   - See known limitation below.

## Known Limitation and Pending Fix

There is currently a startup mismatch between:
- `robot/bipedal_robot.py` (classical controller),
- `lerobot_humanoid_lerobot_integration/lerobot_humanoid.py` (LeRobot controller).

Desired behavior: wait for response from all motors before enabling torque / applying startup offsets, matching the classical controller behavior.

Until this is unified:
- start LeRobot controller only from a safe, near-reference pose,
- keep initial commands conservative,
- verify state before any motion command.
