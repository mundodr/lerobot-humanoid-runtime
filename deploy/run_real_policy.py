#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from deploy.common import apply_base_gain_profile, resolve_policy_files, set_zero_pose


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run real robot with isolated RL policy.")
    p.add_argument("--policy-dir", type=Path, default=Path("control/policy/tao_iteration"))
    p.add_argument("--log-path", type=Path, default=None)
    p.add_argument(
        "--log-name",
        type=str,
        default="runtime_debug_ctrl.csv",
        help="Log filename used inside --policy-dir when --log-path is not provided.",
    )
    p.add_argument("--control-hz", type=float, default=100.0)
    p.add_argument("--action-scale", type=float, default=0.0)
    p.add_argument("--joint-vel-source", type=str, default="auto")

    p.add_argument("--with-imu", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--imu-sensor", type=str, default="bno055", choices=("bno055", "jy901", "bno085"))
    p.add_argument("--i2c-bus", type=int, default=1)
    p.add_argument("--imu-address", type=lambda x: int(x, 0), default=0x28)
    p.add_argument("--frame-yaw-deg", type=float, default=-180.0)
    p.add_argument("--jy901-port", type=str, default="/dev/ttyAMA0")
    p.add_argument("--jy901-baudrate", type=int, default=9600)

    p.add_argument("--with-meshcat", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--viz-hz", type=float, default=20.0)
    p.add_argument("--max-command-delta-deg", type=float, default=60.0)

    p.add_argument("--zero-pose", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--enable-motors", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--gain-profile", type=str, default="tao", choices=("none", "tao"))

    p.add_argument("--with-gamepad", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--gamepad-name", type=str, default="8bitdo")
    p.add_argument("--deadzone", type=float, default=0.12)
    p.add_argument("--max-lin-x", type=float, default=0.75)
    p.add_argument("--max-lin-y", type=float, default=0.5)
    p.add_argument("--max-yaw-rate", type=float, default=0.8)

    p.add_argument("--cmd-lin-x", type=float, default=0.0)
    p.add_argument("--cmd-lin-y", type=float, default=0.0)
    p.add_argument("--cmd-yaw-rate", type=float, default=0.0)
    return p.parse_args()


def build_imu(args: argparse.Namespace) -> Any | None:
    if not args.with_imu:
        return None
    from imu.IMU_integration import IMU

    if args.imu_sensor == "bno055":
        return IMU(
            sensor="bno055",
            i2c_bus=int(args.i2c_bus),
            address=int(args.imu_address),
            rate_hz=100.0,
            frame_yaw_deg=float(args.frame_yaw_deg),
        )
    if args.imu_sensor == "jy901":
        return IMU(
            sensor="jy901",
            port=str(args.jy901_port),
            baudrate=int(args.jy901_baudrate),
        )
    return IMU(sensor="bno085", address=int(args.imu_address))


def main() -> int:
    args = parse_args()
    config_path, policy_path = resolve_policy_files(Path(args.policy_dir))
    from apps.gamepad_controller import GamepadController
    from control.RL_agent_isolated import RLAgent
    from robot.bipedal_robot import BipedalRobotController

    robot = None
    pad = None
    agent = None

    try:
        robot = BipedalRobotController(control_hz=float(args.control_hz), imu=build_imu(args))
        if args.with_meshcat:
            robot.attach_default_meshcat()
            robot._viz_hz = float(args.viz_hz)
        robot.set_max_command_delta(float(args.max_command_delta_deg))

        robot.start(mode="state_only", auto_enable=False)
        robot.request_state_once()

        if args.zero_pose:
            set_zero_pose(robot)

        robot.set_mode("control")
        if args.enable_motors:
            robot.enable_all()

        if args.gain_profile == "tao":
            apply_base_gain_profile(robot)

        if args.with_gamepad:
            pad = GamepadController(
                name_substring=str(args.gamepad_name),
                deadzone=float(args.deadzone),
                max_lin_x=float(args.max_lin_x),
                max_lin_y=float(args.max_lin_y),
                max_yaw_rate=float(args.max_yaw_rate),
            )
            pad.connect()
            pad.start()

        default_log_path = Path(args.policy_dir) / str(args.log_name)
        log_path = Path(args.log_path) if args.log_path is not None else default_log_path
        agent = RLAgent.from_files(
            robot,
            config_path=str(config_path),
            policy_path=str(policy_path),
            log_path=str(log_path),
            log_observation=True,
            log_action=True,
            log_every_n=1,
        )
        agent.spec.joint_vel_source = str(args.joint_vel_source)
        agent.spec.action_scale = float(args.action_scale)
        if pad is not None:
            agent.set_command_source(pad)
        else:
            agent.set_command_twist(float(args.cmd_lin_x), float(args.cmd_lin_y), float(args.cmd_yaw_rate))
        agent.start()

        print(f"[deploy] real policy running: {policy_path}")
        print(f"[deploy] config: {config_path}")
        print(f"[deploy] log: {log_path}")
        print("[deploy] press Ctrl+C to stop")
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        if agent is not None:
            try:
                agent.stop()
            except Exception:
                pass
        if pad is not None:
            try:
                pad.stop()
            except Exception:
                pass
        if robot is not None:
            try:
                robot.stop()
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
