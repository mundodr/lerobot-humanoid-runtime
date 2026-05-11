# AGENT.md

This file is a practical runbook for:
- operators who want to run their own policy on the robot,
- coding agents modifying this repository.

It is complementary to [`README.md`](./README.md).  
If information diverges, update both.

## 1) Goal

Operate a 12-DOF biped safely in:
- simulation,
- real hardware with the classical controller + RL agent,
- real hardware through LeRobot integration.

Calibration is mandatory before real deployment:
- [`CALIBRATION.md`](./CALIBRATION.md)

## 2) Current Author Setup (Reference)

- Platform: Raspberry Pi 5, Ubuntu.
- Python target: 3.13.
- Runtime habit: `micromamba` env currently used by author.
- Tip for real deployments: configure `systemd` auto-start for CAN bring-up and MeshCat server.
  - create your own CAN and MeshCat services for your installation,
  - enable them on boot:

```bash
sudo systemctl enable --now <can-service-name>
sudo systemctl enable --now <meshcat-service-name>
```

## 3) Hard Safety Rules (Do Not Violate)

1. Never disable safety layers in controllers.
2. Never bypass controller public APIs to send raw unsafe commands.
3. Always start in `state_only` and validate visualization/state before `control`.
4. If an experiment fails, debug posture/startup conditions first (move robot near reference pose), not safety logic.
5. Keep an immediate power cutoff strategy available during real tests.

Safety-critical files:
- [`robot/bipedal_robot.py`](./robot/bipedal_robot.py)
- [`lerobot_humanoid_lerobot_integration/lerobot_humanoid.py`](./lerobot_humanoid_lerobot_integration/lerobot_humanoid.py)

## 4) Quick Repo Map

- Classical real controller: [`robot/bipedal_robot.py`](./robot/bipedal_robot.py)
- Policy inference: [`control/rl_agent.py`](./control/rl_agent.py)
- Staged deploy runner: [`deploy/run_real_policy_sequential.py`](./deploy/run_real_policy_sequential.py)
- LeRobot integration: [`lerobot_humanoid_lerobot_integration/`](./lerobot_humanoid_lerobot_integration)
- Acquisition: [`tools/data_acquisition.py`](./tools/data_acquisition.py)
- Simulation controller: [`robot/sim_robot.py`](./robot/sim_robot.py)
- Manual helper snippets: [`ipython_helper.py`](./ipython_helper.py)

## 5) Preflight Workflow (Recommended)

### Step A: Mock-first smoke test (no real hardware motion)

```bash
uv run python deploy/run_real_policy_sequential.py \
  --policy-dir control/policy/codex_iteration_6 \
  --use-mock-bus \
  --with-imu \
  --imu-sensor mock \
  --no-with-meshcat \
  --no-with-gamepad \
  --no-pause-between-stages \
  --no-interactive-scale
```

### Step B: Simulation sanity

Use:
- [`robot/sim_robot.py`](./robot/sim_robot.py), and/or
- snippets in [`ipython_helper.py`](./ipython_helper.py).

### Step C: Ipython real bring-up flow

Use copy/paste workflow from [`ipython_helper.py`](./ipython_helper.py).

## 6) Real Deployment Flow (Classical Controller)

Preferred entrypoint:

```bash
uv run python deploy/run_real_policy_sequential.py --policy-dir control/policy/codex_iteration_6
```

Operator checklist during staged pauses:
1. In `state_only`, verify robot in MeshCat looks physically coherent.
2. Verify base orientation and joint directions match real robot.
3. Switch to `control`, enable, send zero.
4. Start policy only after checks pass.

## 7) Real Deployment Flow (LeRobot Integration)

Use:
- [`lerobot_humanoid_lerobot_integration/lerobot_humanoid.py`](./lerobot_humanoid_lerobot_integration/lerobot_humanoid.py)
- [`tools/data_acquisition.py`](./tools/data_acquisition.py) for data collection workflows.

Notes:
- Keep startup posture near reference.
- Startup now waits for first state replies from all motors before full initialization.

## 8) Data Acquisition Notes

Main script:
- [`tools/data_acquisition.py`](./tools/data_acquisition.py)

Safety guidance:
- High gain + large step amplitude can be dangerous.
- Safe amplitude depends on axis and current range-of-motion state.
- Validate ROM and startup posture before aggressive commands.

## 9) MeshCat / Remote Display FAQ

Q: “Robot is running but no MeshCat on my laptop.”  
A:
1. Ensure meshcat server is running on Pi.
2. Use SSH port forwarding from laptop to Pi. Author command:
   ```bash
   ssh -L 7000:localhost:7000 lerobot@172.18.133.90
   ```
3. Verify expected ports are listening on Pi (`6000` for ZMQ path in controller, plus web port from meshcat server setup).
4. Open MeshCat in browser via forwarded endpoint (`http://localhost:7000` in this setup).

Example check on Pi:

```bash
ss -ltnp | rg "6000|7000"
```

## 10) Common Failure Modes (Author Experience)

1. Ankle not correctly oriented.
   - Put foot near zero reference and reboot motors.
2. IMU unplugged / fails silently.
   - Check cabling/power first; then restart process.
3. Unexpected runtime errors.
   - Reboot Raspberry Pi and motor power chain.

This recovery protocol has been sufficient in the author’s tests.

## 11) Policy FAQ

Q: “How do I run my own policy?”  
A:
1. Put policy files under `control/policy/<your_policy>/`.
2. Ensure required artifacts exist (`policy.onnx` + compatible config).
3. Run staged deploy with:
   - `--policy-dir control/policy/<your_policy>`.

Q: “What default policy should I start with?”  
A: Author’s current default is `control/policy/codex_iteration_6`.

## 12) Agent Contribution Rules

When modifying code:
1. Preserve controller safety behavior.
2. Route all commands through existing controller public APIs.
3. Prefer adding diagnostics/tests/config clarity over loosening guards.
4. Keep docs synchronized (`README.md`, `CALIBRATION.md`, and this file).

When validating changes:
1. Run syntax/import checks.
2. Run mock staged deploy.
3. Run mock acquisition.
4. Only then request real-hardware validation.

## 13) Hardware Reference

Hardware/BOM repository:
- <https://github.com/Virgileboat/lerobot-humanoid-hardware>

## 14) Open Items To Fill (When Known)

- Exact `systemd` service names used on Pi (if systemd startup is used) for:
  - CAN bring-up
  - MeshCat server
- Canonical persistent data/log root path on Pi.
- Formal release signoff checklist with pass/fail criteria.
