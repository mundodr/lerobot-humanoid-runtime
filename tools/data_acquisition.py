#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Mapping

import yaml
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.pipeline_features import aggregate_pipeline_dataset_features, create_initial_features
from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts
from lerobot.processor import make_default_processors
from lerobot.utils.constants import ACTION, OBS_STR

# Prefer local imports when launched as script.
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from lerobot_humanoid_lerobot_integration import LeRobotHumanoid, LeRobotHumanoidConfig  # noqa: E402
from lerobot_humanoid_lerobot_integration.lerobot_humanoid import JOINT_ORDER  # noqa: E402


ROBOT_NAME = "lerobot_humanoid"
EXPECTED_MOTOR_IDS = tuple(range(1, len(JOINT_ORDER) + 1))

# Imported from identification_2 baseline (kept as source-of-truth defaults).
POSITION_KP_BY_MOTOR_ID_BASELINE: dict[int, float] = {
    1: 10.0,
    2: 20.0,
    3: 2.0,
    4: 2.0,
    5: 10.0,
    6: 10.0,
    7: 10.0,
    8: 20.0,
    9: 2.0,
    10: 2.0,
    11: 10.0,
    12: 10.0,
}
POSITION_KD_BY_MOTOR_ID_BASELINE: dict[int, float] = {
    1: 2.0,
    2: 2.0,
    3: 0.1,
    4: 0.1,
    5: 0.5,
    6: 0.5,
    7: 2.0,
    8: 2.0,
    9: 0.1,
    10: 0.1,
    11: 0.5,
    12: 0.5,
}


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Collect per-joint datasets for humanoid identification.")
    p.add_argument(
        "--robot",
        type=str,
        default=ROBOT_NAME,
        choices=[ROBOT_NAME],
        help="Robot model key. Only lerobot_humanoid is supported for now.",
    )
    p.add_argument("--fps", type=int, default=100)
    p.add_argument("--total-duration-s", type=float, default=5.0)
    p.add_argument(
        "--command-mode",
        type=str,
        choices=["step", "sinus"],
        default="step",
        help="Command profile used for each joint over --total-duration-s.",
    )
    p.add_argument(
        "--amplitudes-deg",
        nargs="+",
        type=float,
        default=[0.0, -5.0, 5.0],
        help="Step mode only: N plateau values in degrees over the full recording duration.",
    )
    p.add_argument(
        "--sinus-amp-deg",
        type=float,
        default=5.0,
        help="Sinus mode only: sine amplitude in degrees.",
    )
    p.add_argument(
        "--sinus-freq-hz",
        type=float,
        default=0.5,
        help="Sinus mode only: sine frequency in Hz.",
    )
    p.add_argument(
        "--experiment-name",
        type=str,
        default="",
        help="Optional experiment folder name. Default: experiment_<timestamp>.",
    )
    p.add_argument(
        "--datasets-root",
        type=str,
        default="",
        help="Optional override for output root. Default: identification/models/<robot_name>/datasets",
    )
    p.add_argument("--use-mock-bus", action=argparse.BooleanOptionalAction, default=False)
    p.add_argument("--imu-mock", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--ankle-guard-enabled", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--pre-roll-s", type=float, default=1.0)
    p.add_argument("--between-joints-s", type=float, default=0.5)
    return p.parse_args()


def _zero_action() -> dict[str, float]:
    return {f"{joint}.pos": 0.0 for joint in JOINT_ORDER}


def _safe_send_zero(robot: LeRobotHumanoid) -> None:
    try:
        robot.send_action(_zero_action())
    except Exception as e:
        print(f"[warn] could not send zero action during teardown: {e}")


def _validate_motor_gain_ids(
    *,
    kp_by_motor_id: Mapping[int, float],
    kd_by_motor_id: Mapping[int, float],
) -> None:
    kp_ids = tuple(sorted(int(k) for k in kp_by_motor_id.keys()))
    kd_ids = tuple(sorted(int(k) for k in kd_by_motor_id.keys()))
    if kp_ids != EXPECTED_MOTOR_IDS:
        raise RuntimeError(f"Invalid kp motor IDs: got {kp_ids}, expected {EXPECTED_MOTOR_IDS}")
    if kd_ids != EXPECTED_MOTOR_IDS:
        raise RuntimeError(f"Invalid kd motor IDs: got {kd_ids}, expected {EXPECTED_MOTOR_IDS}")


def _load_position_gains_from_baseline() -> tuple[dict[int, float], dict[int, float]]:
    kp = {int(k): float(v) for k, v in POSITION_KP_BY_MOTOR_ID_BASELINE.items()}
    kd = {int(k): float(v) for k, v in POSITION_KD_BY_MOTOR_ID_BASELINE.items()}
    _validate_motor_gain_ids(kp_by_motor_id=kp, kd_by_motor_id=kd)
    return kp, kd


def _apply_gains_from_baseline(
    robot: LeRobotHumanoid,
) -> tuple[dict[int, float], dict[int, float]]:
    kp_by_motor_id, kd_by_motor_id = _load_position_gains_from_baseline()
    print("Applying baseline gains")
    print(f"  position_kp_by_motor_id: {kp_by_motor_id}")
    print(f"  position_kd_by_motor_id: {kd_by_motor_id}")
    robot.config.position_kp.update({int(k): float(v) for k, v in kp_by_motor_id.items()})
    robot.config.position_kd.update({int(k): float(v) for k, v in kd_by_motor_id.items()})
    robot.configure()
    return kp_by_motor_id, kd_by_motor_id


def _gains_payload(
    *,
    robot_key: str,
    gains_source_tag: str,
    kp_by_motor_id: Mapping[int, float],
    kd_by_motor_id: Mapping[int, float],
) -> dict[str, Any]:
    _validate_motor_gain_ids(kp_by_motor_id=kp_by_motor_id, kd_by_motor_id=kd_by_motor_id)
    return {
        "robot_key": str(robot_key),
        "gains_source": str(gains_source_tag),
        "position_kp_by_motor_id": {str(int(k)): float(v) for k, v in kp_by_motor_id.items()},
        "position_kd_by_motor_id": {str(int(k)): float(v) for k, v in kd_by_motor_id.items()},
    }


def _build_command_profile(
    *,
    command_mode: str,
    total_duration_s: float,
    fps: int,
    amplitudes_deg: tuple[float, ...],
    sinus_amp_deg: float,
    sinus_freq_hz: float,
) -> tuple[list[float], dict[str, Any]]:
    total_steps = int(round(float(total_duration_s) * float(fps)))
    if total_steps < 1:
        raise ValueError(f"Invalid total duration/fps: total_steps={total_steps}.")

    mode = str(command_mode).strip().lower()
    if mode == "step":
        values = [float(v) for v in amplitudes_deg]
        if len(values) < 1:
            raise ValueError("Step mode requires at least one value in --amplitudes-deg.")
        cmd = [values[int(i * len(values) / total_steps)] for i in range(total_steps)]
        meta = {
            "command_mode": "step",
            "total_steps": int(total_steps),
            "total_duration_s": float(total_duration_s),
            "fps": int(fps),
            "step_values_deg": [float(v) for v in values],
        }
        return cmd, meta

    if mode == "sinus":
        amp = float(sinus_amp_deg)
        freq = float(sinus_freq_hz)
        dt = 1.0 / float(fps)
        cmd = [amp * math.sin(2.0 * math.pi * freq * (i * dt)) for i in range(total_steps)]
        meta = {
            "command_mode": "sinus",
            "total_steps": int(total_steps),
            "total_duration_s": float(total_duration_s),
            "fps": int(fps),
            "sinus_amp_deg": float(amp),
            "sinus_freq_hz": float(freq),
        }
        return cmd, meta

    raise ValueError(f"Unsupported command mode '{command_mode}'.")


def _write_experiment_acquisition_context(
    *,
    experiment_root: Path,
    payload: Mapping[str, Any],
) -> None:
    experiment_root.mkdir(parents=True, exist_ok=True)
    out = experiment_root / "acquisition_context.json"
    out.write_text(json.dumps(dict(payload), indent=2) + "\n")


def _write_experiment_manifest_yaml(
    *,
    experiment_root: Path,
    timestamp: str,
    frames: int,
    joint_order: list[str],
    joint_roots: Mapping[str, Path],
) -> None:
    manifest = {
        "timestamp": str(timestamp),
        "frames": int(frames),
        "joint_order": list(joint_order),
        "joints": {str(j): str(Path(p)) for j, p in joint_roots.items()},
    }
    (experiment_root / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))


def _resolve_default_datasets_root(robot_name: str) -> Path:
    return REPO_ROOT / "identification" / "models" / robot_name / "datasets"


def collect_per_joint_step_datasets(
    *,
    robot: LeRobotHumanoid,
    robot_key: str,
    gains_source_tag: str,
    kp_by_motor_id: Mapping[int, float],
    kd_by_motor_id: Mapping[int, float],
    fps: int,
    command_mode: str,
    amplitudes_deg: tuple[float, ...],
    sinus_amp_deg: float,
    sinus_freq_hz: float,
    total_duration_s: float,
    experiment_name: str = "",
    datasets_root: Path | None = None,
    pre_roll_s: float = 1.0,
    between_joints_s: float = 0.5,
) -> Path:
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()
    features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            teleop_action_processor,
            create_initial_features(action=robot.action_features),
            use_videos=False,
        ),
        aggregate_pipeline_dataset_features(
            robot_observation_processor,
            create_initial_features(observation=robot.observation_features),
            use_videos=False,
        ),
    )

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    exp_name = str(experiment_name).strip() or f"experiment_{run_id}"

    root = datasets_root if datasets_root is not None else _resolve_default_datasets_root(robot.name)
    root.mkdir(parents=True, exist_ok=True)
    experiment_root = Path(root) / exp_name
    experiment_root.mkdir(parents=True, exist_ok=True)

    dt = 1.0 / float(fps)
    cmd_profile_deg, cmd_profile_meta = _build_command_profile(
        command_mode=command_mode,
        total_duration_s=float(total_duration_s),
        fps=int(fps),
        amplitudes_deg=tuple(float(a) for a in amplitudes_deg),
        sinus_amp_deg=float(sinus_amp_deg),
        sinus_freq_hz=float(sinus_freq_hz),
    )
    total_steps = int(len(cmd_profile_deg))

    zero = _zero_action()
    robot.send_action(zero)
    time.sleep(float(pre_roll_s))

    gain_meta = _gains_payload(
        robot_key=robot_key,
        gains_source_tag=gains_source_tag,
        kp_by_motor_id=kp_by_motor_id,
        kd_by_motor_id=kd_by_motor_id,
    )
    gain_meta["command_profile"] = cmd_profile_meta
    _write_experiment_acquisition_context(experiment_root=experiment_root, payload=gain_meta)

    frames_per_joint = int(total_steps)
    manifest_joint_roots: dict[str, Path] = {}

    for joint in JOINT_ORDER:
        joint_root = experiment_root / joint
        ds = LeRobotDataset.create(
            repo_id=f"local/{robot.name}_{exp_name}_{joint}",
            fps=int(fps),
            root=joint_root,
            robot_type=robot.name,
            features=features,
            use_videos=False,
        )
        print(f"[record] joint={joint} root={joint_root}")
        print(
            f"  profile={cmd_profile_meta['command_mode']} "
            f"duration={cmd_profile_meta['total_duration_s']:.2f}s "
            f"steps={cmd_profile_meta['total_steps']}"
        )

        next_tick = time.perf_counter()
        overruns = 0
        late_time_s_acc = 0.0
        for cmd_deg in cmd_profile_deg:
            now = time.perf_counter()
            sleep_s = next_tick - now
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                overruns += 1
                late_time_s_acc += -sleep_s
                # Re-sync on overrun to avoid cumulative drift.
                next_tick = now

            cmd_raw = dict(zero)
            cmd_raw[f"{joint}.pos"] = float(cmd_deg)
            obs = robot.get_observation()
            obs_p = robot_observation_processor(obs)
            act_p = robot_action_processor((cmd_raw, obs))
            robot.send_action(act_p)

            act_full = {f"{j}.pos": float(cmd_raw[f"{j}.pos"]) for j in JOINT_ORDER}
            obs_frame = build_dataset_frame(ds.features, obs_p, prefix=OBS_STR)
            act_frame = build_dataset_frame(ds.features, act_full, prefix=ACTION)
            ds.add_frame(
                {
                    **obs_frame,
                    **act_frame,
                    "task": f"single_joint_{cmd_profile_meta['command_mode']}:{joint}",
                }
            )
            next_tick += dt

        if overruns > 0:
            mean_late_ms = 1000.0 * late_time_s_acc / float(overruns)
            overrun_pct = 100.0 * float(overruns) / float(total_steps)
            print(
                f"  [warn] control-loop overruns: {overruns}/{total_steps} "
                f"({overrun_pct:.1f}%), mean lateness={mean_late_ms:.2f} ms"
            )

        ds.save_episode()
        ds.finalize()
        manifest_joint_roots[joint] = joint_root
        robot.send_action(zero)
        time.sleep(float(between_joints_s))

    _write_experiment_manifest_yaml(
        experiment_root=experiment_root,
        timestamp=run_id,
        frames=frames_per_joint,
        joint_order=list(JOINT_ORDER),
        joint_roots=manifest_joint_roots,
    )
    print(f"[done] per-joint datasets saved under {experiment_root}")
    return experiment_root


def main() -> None:
    args = _parse_args()
    if str(args.robot) != ROBOT_NAME:
        raise ValueError(f"Unsupported robot '{args.robot}'. Supported: {ROBOT_NAME}")

    cfg = LeRobotHumanoidConfig(
        use_mock_bus=bool(args.use_mock_bus),
        imu_mock=bool(args.imu_mock),
        ankle_guard_enabled=bool(args.ankle_guard_enabled),
    )
    robot = LeRobotHumanoid(cfg)

    print(f"[setup] connecting robot='{robot.name}'")
    robot.connect()
    try:
        gains_source_tag = "baseline:POSITION_*_BY_MOTOR_ID_BASELINE"
        kp_by_motor_id, kd_by_motor_id = _apply_gains_from_baseline(robot)
        print(f"[setup] applied gains from {gains_source_tag}")
        try:
            robot.send_action(_zero_action())
        except Exception as e:
            print(f"[error] unable to send initial action. Clear E-STOP and retry: {e}")
            return
        time.sleep(float(args.pre_roll_s))

        datasets_root = Path(args.datasets_root) if str(args.datasets_root).strip() else None
        collect_per_joint_step_datasets(
            robot=robot,
            robot_key=str(args.robot),
            gains_source_tag=gains_source_tag,
            kp_by_motor_id=kp_by_motor_id,
            kd_by_motor_id=kd_by_motor_id,
            fps=int(args.fps),
            command_mode=str(args.command_mode),
            amplitudes_deg=tuple(float(a) for a in args.amplitudes_deg),
            sinus_amp_deg=float(args.sinus_amp_deg),
            sinus_freq_hz=float(args.sinus_freq_hz),
            total_duration_s=float(args.total_duration_s),
            experiment_name=str(args.experiment_name),
            datasets_root=datasets_root,
            pre_roll_s=float(args.pre_roll_s),
            between_joints_s=float(args.between_joints_s),
        )
    finally:
        _safe_send_zero(robot)
        try:
            robot.disconnect()
        except Exception as e:
            print(f"[warn] robot disconnect raised: {e}")
        print("[teardown] robot disconnected")


if __name__ == "__main__":
    main()
