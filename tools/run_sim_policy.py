#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from control.rl_agent import RLAgent
from deploy.common import resolve_policy_files, set_zero_pose
from robot.sim_robot import SimBipedalRobotController


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run a walking policy in MuJoCo simulation.")
    p.add_argument("--policy-dir", type=Path, default=Path("control/policy/codex_iteration_8"))
    p.add_argument("--control-hz", type=float, default=50.0)
    p.add_argument("--sim-dt", type=float, default=0.005)
    p.add_argument("--initial-height-m", type=float, default=0.72)
    p.add_argument("--duration-s", type=float, default=60.0, help="Use <=0 to run until Ctrl+C.")
    p.add_argument("--cmd-lin-x", type=float, default=0.20, help="Forward command in m/s.")
    p.add_argument("--cmd-lin-y", type=float, default=0.0, help="Sideways command in m/s.")
    p.add_argument("--cmd-yaw-rate", type=float, default=0.0, help="Yaw-rate command in rad/s.")
    p.add_argument("--action-scale", type=float, default=0.25, help="Start small; try 0.1 to 1.0.")
    p.add_argument(
        "--policy-action-clip",
        type=float,
        default=1.0,
        help="Clip raw policy outputs before using them as actions. Use <=0 to disable.",
    )
    p.add_argument("--fixed-base", action=argparse.BooleanOptionalAction, default=False)
    p.add_argument("--hardcode-mjlab-spawn", action=argparse.BooleanOptionalAction, default=False)
    p.add_argument(
        "--warmup-fixed-base-s",
        type=float,
        default=1.0,
        help="Hold the floating base fixed for this many seconds before release. Use 0 to disable.",
    )
    p.add_argument("--viewer", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--joint-vel-source", type=str, default="finite_diff")
    p.add_argument("--log-path", type=Path, default=Path("logs/sim_policy_debug.csv"))
    p.add_argument("--log", action=argparse.BooleanOptionalAction, default=False)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    config_path, policy_path = resolve_policy_files(args.policy_dir)

    warmup_fixed = (not bool(args.fixed_base)) and float(args.warmup_fixed_base_s) > 0.0
    robot = SimBipedalRobotController(
        control_hz=float(args.control_hz),
        sim_dt=float(args.sim_dt),
        initial_height_m=float(args.initial_height_m),
        fixed_base=bool(args.fixed_base) or warmup_fixed,
        hardcode_mjlab_spawn=bool(args.hardcode_mjlab_spawn),
        auto_reset_on_flip=True,
        reset_hold_s=0.3,
    )
    agent = None

    try:
        robot.start(mode="control", auto_enable=True)
        set_zero_pose(robot)
        if args.viewer:
            robot.start_viewer()

        agent = RLAgent.from_files(
            robot,
            config_path=str(config_path),
            policy_path=str(policy_path),
            log_path=str(args.log_path),
            log_observation=bool(args.log),
            log_action=bool(args.log),
            log_every_n=1,
        )
        agent.spec.joint_vel_source = str(args.joint_vel_source)
        agent.spec.action_scale = float(args.action_scale)
        agent.spec.policy_action_clip = float(args.policy_action_clip) if float(args.policy_action_clip) > 0.0 else None
        agent.set_command_twist(float(args.cmd_lin_x), float(args.cmd_lin_y), float(args.cmd_yaw_rate))
        agent.start()

        print(f"[sim-policy] policy: {policy_path}", flush=True)
        print(f"[sim-policy] config: {config_path}", flush=True)
        print(
            "[sim-policy] command "
            f"lin_x={args.cmd_lin_x:.3f} lin_y={args.cmd_lin_y:.3f} yaw={args.cmd_yaw_rate:.3f} "
            f"action_scale={args.action_scale:.3f} fixed_base={args.fixed_base}",
            flush=True,
        )
        print("[sim-policy] running; close viewer or press Ctrl+C to stop.", flush=True)

        start = time.perf_counter()
        released = not warmup_fixed
        next_status_s = 0.0
        while True:
            elapsed = time.perf_counter() - start
            if warmup_fixed and not released and elapsed >= float(args.warmup_fixed_base_s):
                robot.set_fixed_base(False)
                released = True
                print("[sim-policy] released fixed-base warmup", flush=True)
            if elapsed >= next_status_s:
                snap = robot.get_combined_state_snapshot(include_joint_state=False)
                print(
                    "[sim-policy] "
                    f"t={elapsed:5.1f}s z={float(robot.data.qpos[2]) if robot.data.qpos.size >= 3 else 0.0:.3f} "
                    f"resets={snap['sim_reset_count']} fixed_base={snap['fixed_base']}",
                    flush=True,
                )
                next_status_s = elapsed + 1.0
            time.sleep(0.2)
            if args.duration_s > 0 and elapsed >= float(args.duration_s):
                break
    except KeyboardInterrupt:
        print("[sim-policy] stopping...", flush=True)
    finally:
        if agent is not None:
            agent.stop()
        robot.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
