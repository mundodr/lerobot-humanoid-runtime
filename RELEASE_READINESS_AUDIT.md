# Release Readiness Audit

Date: 2026-05-11
Repo: `lerobot_humanoid_runtime`
Branch: `robot_pi_bundle_lerobot_submodule_test`

## Scope
This file captures:
- The deployment constraints already agreed.
- The blocking issues found for a release-ready Raspberry Pi setup.
- A concrete fix plan, without `try/except` import fallbacks.

## Agreed Constraints
- Target users: anyone building and controlling this robot.
- Platform: Raspberry Pi 5 on Ubuntu.
- Environment tooling: `uv`.
- Python target: `3.13` (for MuJoCo + LeRobot compatibility in this repo context).
- This repo must include calibration usage and data acquisition usage.
- Safety behavior in controllers must be preserved.
- LeRobot dependency is temporary as a submodule pinned to the required branch/commit until upstream merge.

## Current Blocking Issues

### 1) Tao-only absolute import path coupling
Status on 2026-05-11: runtime scripts with hardcoded Tao path were removed from this repo:
- `deploy/run_real_policy_sequential_2.py`
- `control/policy/velocity_v16_iter124750/deploy_v16.py`
- `control/policy/velocity_v16_iter124750/capture_snapshot_on_pi.py`
- `control/policy/velocity_v16_iter124750/go_to_zero_and_hold.py`

Impact:
- Tao-path dependency for these scripts is eliminated.

### 2) Policy artifact configs include machine-specific absolute paths
Several `control/policy/**/config.yaml` files include paths such as:
- `/home/vbatto/...`
- `/home/virgile_batto/...`

Impact:
- These are training metadata and not runtime-critical, but they create confusion and false expectations for reproducibility.

### 3) Submodule health risk (`lerobot`)
Desired state:
- `lerobot` is a clean submodule clone from official repo.
- branch target: `feat/clean-can-bus`.
- pinned superproject commit should be intentional and reproducible.

Current observed issue:
- submodule currently reports mass staged deletions plus untracked duplicates (`D` and `??`), meaning local submodule index/worktree is dirty/corrupted.

Impact:
- Unsafe to trust runtime behavior until cleaned.
- Hard to reproduce exactly on Pi.

### 4) Startup behavior mismatch note (LeRobot vs bipedal controller)
Known behavior difference to align:
- In bipedal controller startup, wait for all motors to respond before startup/offset.
- LeRobot-side startup should match this safety behavior.

Impact:
- Potential wrong initial offsets / unsafe startup edge cases.

## Fix Plan (No Fallback Imports)

### Step 1: Create one runtime package boundary
- Move Tao-dependent runtime code used by deploy scripts into this repo package modules.
- Replace Tao imports with direct imports from local package only.
- Keep one canonical import path; no `try/except import` fallback branch.

### Step 2: Replace hardcoded absolute paths with repo-relative or CLI args
- Default `policy-dir` to repo-relative path.
- Resolve repo root from script location (`Path(__file__)`) where needed.
- Keep optional CLI override flags for custom locations.

### Step 3: Submodule normalization
- Keep `lerobot` as a submodule:
  - URL: `git@github.com:huggingface/lerobot.git`
  - branch: `feat/clean-can-bus`
- Ensure clean state with:
  1. `git -C lerobot reset --hard HEAD`
  2. `git -C lerobot clean -fd`
  3. `git -C lerobot status` must be clean
  4. top repo `git status` must not show `m lerobot`
- Pin the superproject to the exact tested submodule commit.

### Step 4: Startup safety alignment
- Add/verify explicit "wait for all motor replies" phase in LeRobot controller startup before offset enable.
- Re-run startup and acquisition smoke tests on robot.

### Step 5: Documentation stabilization
- Keep `README.md` as operator entrypoint.
- Keep `CALIBRATION.md` as procedural guide.
- Add one short "submodule init/update" section and one "known failure + fix" section.

## Acceptance Criteria
- `uv sync` works on Pi 5 Ubuntu with documented Python version.
- Real robot scripts run without Tao path dependency.
- `lerobot` submodule is clean after clone + submodule init.
- Startup sequence behaves safely and consistently across both controllers.
- Data acquisition path is documented and runnable from this repo only.

## Quick Verification Commands
```bash
git status --short
git submodule status --recursive
git -C lerobot status --short --branch
rg -n "TAO_ROOT|/home/lerobot/devel/Tao|sys.path.insert\\(0, str\\(TAO_ROOT\\)\\)" deploy control
```
