#!/usr/bin/env python3
"""Deploy velocity_v16_iter124750 on the real LeRobot humanoid using Tao's
RL_agent_isolated (which supports the isaaclab obs layout that this policy
was trained with).

Safety: defaults to SHADOW mode (read state, run policy inference, log CSV —
but DO NOT enable motors). Pass --enable-motors to actually drive the robot.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Tao's code path — this is where we pick up RL_agent_isolated, bipedal_robot, IMU, etc.
TAO_ROOT = Path("/home/lerobot/devel/Tao/lerobot-humanoid-design/to_real_robot")
if str(TAO_ROOT) not in sys.path:
    sys.path.insert(0, str(TAO_ROOT))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Deploy v16 policy on real robot via Tao bridge.")
    p.add_argument("--policy-dir", type=Path,
                   default=Path("/home/lerobot/devel/lerobot-humanoid-runtime/control/policy/velocity_v16_iter124750"))
    p.add_argument("--log-name", type=str, default="v16_real_debug.csv",
                   help="CSV log filename inside --policy-dir")

    p.add_argument("--control-hz", type=float, default=100.0)
    p.add_argument("--max-command-delta-deg", type=float, default=60.0)

    p.add_argument("--with-imu", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--imu-sensor", type=str, default="bno055", choices=("bno055", "jy901", "bno085"))
    p.add_argument("--i2c-bus", type=int, default=1)
    p.add_argument("--imu-address", type=lambda x: int(x, 0), default=0x28)
    p.add_argument("--frame-yaw-deg", type=float, default=-180.0)
    p.add_argument("--jy901-port", type=str, default="/dev/ttyAMA0")
    p.add_argument("--jy901-baudrate", type=int, default=9600)

    # SAFETY — default is NOT to enable motors.
    p.add_argument("--enable-motors", action="store_true",
                   help="If set, enable motors and actually drive the robot.  Without this, "
                        "we read state + run policy inference + log CSV but never turn motors on.")
    p.add_argument("--zero-pose-first", action="store_true",
                   help="Before running the policy, move motors to all-zero pose (only if --enable-motors).")
    p.add_argument("--duration-s", type=float, default=15.0,
                   help="How long to run (seconds). Ctrl+C also stops early.")

    # Command source — default zeros (stand still).
    p.add_argument("--cmd-vx", type=float, default=0.0)
    p.add_argument("--cmd-vy", type=float, default=0.0)
    p.add_argument("--cmd-wz", type=float, default=0.0)
    return p.parse_args()


def build_imu(args: argparse.Namespace):
    if not args.with_imu:
        return None
    from IMU_integration import IMU  # Tao's IMU module
    if args.imu_sensor == "bno055":
        return IMU(sensor="bno055",
                   i2c_bus=int(args.i2c_bus),
                   address=int(args.imu_address),
                   rate_hz=100.0,
                   frame_yaw_deg=float(args.frame_yaw_deg))
    if args.imu_sensor == "jy901":
        return IMU(sensor="jy901",
                   port=str(args.jy901_port),
                   baudrate=int(args.jy901_baudrate))
    return IMU(sensor="bno085", address=int(args.imu_address))


def main() -> int:
    args = parse_args()

    pdir = Path(args.policy_dir)
    config_path = pdir / "config.yaml"
    policy_path = pdir / "policy.onnx"
    if not config_path.is_file() or not policy_path.is_file():
        print(f"[deploy][ERR] missing config.yaml or policy.onnx in {pdir}")
        return 2

    from bipedal_robot import BipedalRobotController   # Tao's real-hardware controller
    from RL_agent_isolated import RLAgent              # Tao's policy runner (isaaclab layout aware)

    robot = None
    agent = None
    try:
        print(f"[deploy] policy_dir = {pdir}")
        print(f"[deploy] shadow mode = {not args.enable_motors}")
        print(f"[deploy] control_hz = {args.control_hz}")

        imu = build_imu(args)
        robot = BipedalRobotController(control_hz=float(args.control_hz), imu=imu)
        robot.set_max_command_delta(float(args.max_command_delta_deg))

        # Read state first; never command motors until we explicitly switch.
        robot.start(mode="state_only", auto_enable=False)
        robot.request_state_once()

        if args.enable_motors:
            if args.zero_pose_first:
                try:
                    from deploy.common import set_zero_pose  # if available
                    set_zero_pose(robot)
                except Exception as exc:
                    print(f"[deploy][WARN] zero-pose helper not available: {exc}")
            robot.set_mode("control")
            robot.enable_all()
            print("[deploy] MOTORS ENABLED — robot is live")
        else:
            print("[deploy] shadow mode — motors remain OFF")

        log_path = pdir / str(args.log_name)
        agent = RLAgent.from_files(
            robot,
            config_path=str(config_path),
            policy_path=str(policy_path),
            log_path=str(log_path),
            log_observation=True,
            log_action=True,
            log_every_n=1,
        )
        agent.set_command_twist(float(args.cmd_vx), float(args.cmd_vy), float(args.cmd_wz))
        agent.start()
        print(f"[deploy] RLAgent running, logging to {log_path}")
        print(f"[deploy] command twist = ({args.cmd_vx}, {args.cmd_vy}, {args.cmd_wz})")
        print(f"[deploy] duration = {args.duration_s}s  (Ctrl+C to stop early)")

        t_end = time.monotonic() + float(args.duration_s)
        while time.monotonic() < t_end:
            time.sleep(0.5)
        print("[deploy] duration elapsed — stopping")
    except KeyboardInterrupt:
        print("\n[deploy] interrupted — stopping")
    finally:
        if agent is not None:
            try:
                agent.stop()
            except Exception as exc:
                print(f"[deploy][WARN] agent.stop: {exc}")
        if robot is not None:
            try:
                robot.stop()
            except Exception as exc:
                print(f"[deploy][WARN] robot.stop: {exc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
