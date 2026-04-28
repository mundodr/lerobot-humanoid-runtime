#!/usr/bin/env python3
"""Runs on the LeRobot Pi. Captures one RLAgent-style snapshot of the real
robot state.

On Tao's bipedal_robot.py, state_only mode auto-disables the motors (hard
safety), so motors don't stream position data. To actually get real joint
positions, we briefly use mode='control' with auto_enable=True — the control
loop then latches the action target to the current measured position (so the
robot holds still), motors report state back, we snapshot it, we stop.

Since we never call robot.set_action(), the motors only ever receive
'hold current position' commands."""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

TAO_ROOT = Path("/home/lerobot/devel/Tao/lerobot-humanoid-design/to_real_robot")
if str(TAO_ROOT) not in sys.path:
    sys.path.insert(0, str(TAO_ROOT))


def build_imu(imu_sensor, i2c_bus, imu_address, frame_yaw_deg, jy901_port, jy901_baudrate):
    from IMU_integration import IMU
    if imu_sensor == "bno055":
        return IMU(sensor="bno055", i2c_bus=int(i2c_bus), address=int(imu_address),
                   rate_hz=100.0, frame_yaw_deg=float(frame_yaw_deg))
    if imu_sensor == "jy901":
        return IMU(sensor="jy901", port=str(jy901_port), baudrate=int(jy901_baudrate))
    return IMU(sensor="bno085", address=int(imu_address))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--imu-sensor", choices=("bno055", "jy901", "bno085"), default="bno055")
    ap.add_argument("--i2c-bus", type=int, default=1)
    ap.add_argument("--imu-address", type=lambda x: int(x, 0), default=0x28)
    ap.add_argument("--frame-yaw-deg", type=float, default=-180.0)
    ap.add_argument("--jy901-port", type=str, default="/dev/ttyAMA0")
    ap.add_argument("--jy901-baudrate", type=int, default=9600)
    ap.add_argument("--warmup-s", type=float, default=2.0,
                    help="Seconds of control-mode 'hold' before snapshotting.")
    args = ap.parse_args()

    from bipedal_robot import BipedalRobotController

    imu = build_imu(args.imu_sensor, args.i2c_bus, args.imu_address,
                    args.frame_yaw_deg, args.jy901_port, args.jy901_baudrate)

    robot = None
    try:
        robot = BipedalRobotController(control_hz=100.0, imu=imu)
        # Start in state_only first so request_state_once can populate the
        # measured state, then flip to control mode — this makes the control
        # loop latch its action target to the CURRENT measured position (so
        # the robot is commanded 'hold where you are').
        robot.start(mode="state_only", auto_enable=False)
        robot.request_state_once()
        time.sleep(0.2)

        # Switch to control + enable motors — motors will stream state, but
        # because we never call robot.set_action() the target stays at the
        # latched 'current position' (hold).
        robot.set_mode("control")
        robot.enable_all()

        time.sleep(float(args.warmup_s))   # let CAN state stream in

        snap = robot.get_combined_state_snapshot(include_joint_state=True)

        payload = {
            "time_s": snap.get("time_s"),
            "joint_state_deg": snap.get("joint_state_deg"),
            "joint_velocity_rad_s": snap.get("joint_velocity_rad_s"),
            "joint_velocity_deg_s": snap.get("joint_velocity_deg_s"),
            "projected_gravity": snap.get("projected_gravity"),
            "orientation_quaternion_xyzw": snap.get("orientation_quaternion_xyzw"),
            "imu": snap.get("imu"),
        }
        text = json.dumps(payload, indent=2, default=str)
        if args.out:
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.out.write_text(text)
            print(f"[capture] wrote {args.out}", file=sys.stderr)
        else:
            print(text)
    finally:
        if robot is not None:
            try:
                robot.stop()
            except Exception as exc:
                print(f"[capture][WARN] robot.stop: {exc}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
