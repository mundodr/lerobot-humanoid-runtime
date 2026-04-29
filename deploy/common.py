from __future__ import annotations

from pathlib import Path


ZERO_LEFT = {
    "hipz": 0.0,
    "hipx": 0.0,
    "hipy": 0.0,
    "knee": 0.0,
    "ankle_pitch": 0.0,
    "ankle_roll": 0.0,
}

ZERO_RIGHT = {
    "hipz": 0.0,
    "hipx": 0.0,
    "hipy": 0.0,
    "knee": 0.0,
    "ankle_pitch": 0.0,
    "ankle_roll": 0.0,
}


def resolve_policy_files(policy_dir: Path) -> tuple[Path, Path]:
    policy_dir = Path(policy_dir)
    if not policy_dir.exists():
        raise FileNotFoundError(f"Policy directory not found: {policy_dir}")

    config_candidates = (
        policy_dir / "config.yaml",
        policy_dir / "config.yml",
        policy_dir / "model_25000_env.yaml",
        policy_dir / "config.json",
    )
    config_path = next((p for p in config_candidates if p.exists()), None)
    if config_path is None:
        raise FileNotFoundError(
            f"No config file found in {policy_dir}. "
            "Expected one of config.yaml/config.yml/model_25000_env.yaml/config.json"
        )

    policy_path = policy_dir / "policy.onnx"
    if not policy_path.exists():
        raise FileNotFoundError(f"Policy ONNX not found: {policy_path}")
    return config_path, policy_path


def set_zero_pose(robot: object) -> None:
    robot.set_action(left=ZERO_LEFT, right=ZERO_RIGHT)  # type: ignore[attr-defined]


def apply_base_gain_profile(robot: object) -> None:
    # Mirrored from the practical values used in the old ipython helper.
    for mid in (1, 7):
        robot.set_joint_gains(mid, kp=30.0, kd=3.0)  # type: ignore[attr-defined]

    for mid in (5, 6, 11, 12):
        robot.set_joint_gains(mid, kp=10.0, kd=0.75)  # type: ignore[attr-defined]

    robot.set_joint_gains(2, kp=40.0, kd=3.0)   # type: ignore[attr-defined]
    robot.set_joint_gains(3, kp=6.0, kd=0.2)    # type: ignore[attr-defined]
    robot.set_joint_gains(4, kp=12.0, kd=0.4)   # type: ignore[attr-defined]  # kp/kd x2 from 6/0.2 for knee tracking (2026-04-24)
    robot.set_joint_gains(8, kp=40.0, kd=5.0)   # type: ignore[attr-defined]  # kd raised 3->5 for overshoot damping (2026-04-24, still in training distribution: damping x[0.8, 2.0])
    robot.set_joint_gains(9, kp=6.0, kd=0.2)    # type: ignore[attr-defined]
    robot.set_joint_gains(10, kp=12.0, kd=0.4)  # type: ignore[attr-defined]  # kp/kd x2 from 6/0.2 for knee tracking (2026-04-24)

