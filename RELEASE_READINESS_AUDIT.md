# Release Readiness Audit

Date: 2026-05-11  
Repo: `lerobot_humanoid_runtime`  
Branch: `robot_pi_bundle_lerobot_submodule_test`

## Scope
This audit tracks what still blocks a release-ready Raspberry Pi deployment for:
- simulation control,
- real robot control (classical controller),
- real robot control (LeRobot integration),
- calibration + acquisition workflows.

## Target Constraints
- Platform: Raspberry Pi 5 on Ubuntu.
- Python: 3.13.
- Environment tool: `uv`.
- Safety behavior must remain enabled (E-STOP, bounds checks, jump guards).
- LeRobot stays as a pinned submodule until upstream merge.

## Verified Completed Items

### 1) Tao runtime path coupling removed
- Tao-dependent runtime scripts were removed from deploy path.
- No active deploy/runtime script requires `TAO_ROOT`.

### 2) LeRobot startup safety alignment implemented
- LeRobot startup now waits for initial motor replies before startup wrap/offset logic.
- This aligns startup behavior with classical controller intent.

### 3) Submodule health is currently clean
- `lerobot` submodule is clean and points to Hugging Face upstream.
- Current checkout:
  - branch: `feat/clean-can-bus`
  - commit: `6d69cfb95`
  - status: clean (up-to-date with remote branch)

### 4) Data acquisition dependency fixed in project definition
- `pyproject.toml` now uses:
  - `lerobot[robstride,dataset]`
- `uv.lock` refreshed.
- `uv run python tools/data_acquisition.py --help` now works.

### 5) Documentation consistency pass completed
- README stale startup-mismatch note removed.
- README now documents dependency profiles for simulation, real runtime, and tools.
- Absolute machine-local doc links replaced by repo-relative links in:
  - `README.md`
  - `CALIBRATION.md`
  - `ARCHITECTURE_CLASS_DIAGRAM.md`

## Remaining Release Blockers

### A) Policy artifact absolute paths remain in training metadata
- Several policy `config.yaml` files contain machine-specific absolute paths.
- Not runtime-blocking, but should be documented as training metadata only.

### B) Hardware validation sign-off still required
- Final release requires on-robot validation pass with:
  - startup in `state_only`,
  - staged control enable,
  - short policy run,
  - short data acquisition run.

## Updated Fix Plan

### Step 1: Clarify policy config path semantics
- Add one short note in README:
  - absolute paths inside `control/policy/**/config.yaml` are training artifacts,
  - deploy uses `--policy-dir` + local ONNX/config files.

### Step 2: Run final hardware validation and record results
- Capture command logs and pass/fail checklist for:
  - classical controller startup path,
  - LeRobot startup path,
  - data acquisition smoke test.

## Acceptance Criteria
- `uv sync` succeeds with documented Python version.
- `uv run` commands for real deploy and acquisition match docs and start cleanly.
- `lerobot` submodule clone + init yields clean status.
- Startup behavior is consistently safe across classical + LeRobot controllers.
- Calibration guide is complete and executable without placeholders.
- Final hardware validation checklist is captured and passes.

## Quick Verification Commands
```bash
git status --short
git submodule status --recursive
git -C lerobot status --short --branch
git -C lerobot rev-parse --short HEAD
uv run python deploy/run_real_policy_sequential.py --help
uv run python tools/data_acquisition.py --help
rg -n "TAO_ROOT|/home/lerobot/devel/Tao" deploy control tools
```
