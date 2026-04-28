#!/usr/bin/env python3
"""Runs on the LeRobot Pi.  Enables motors, reads the current pose, then
linearly ramps joint-position commands from that pose to model-frame zero
over --ramp-s seconds, and holds zero for --hold-s seconds.  After holding,
optionally writes a snapshot JSON and stops the motors.

This is the 'park to zero then start the policy' workflow the v16 policy
was trained for.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

TAO_ROOT = Path("/home/lerobot/devel/Tao/lerobot-humanoid-design/to_real_robot")
if str(TAO_ROOT) not in sys.path:
    sys.path.insert(0, str(TAO_ROOT))

# Per-side joint keys used by bipedal_robot.set_action
KEYS = ("hipz", "hipx", "hipy", "knee", "ankle_pitch", "ankle_roll")

# Motor id (1..12) -> human-readable name, matching CSV column order.
MOTOR_NAMES = [
    "L_hipz", "L_hipx", "L_hipy", "L_knee", "L_apitch", "L_aroll",
    "R_hipz", "R_hipx", "R_hipy", "R_knee", "R_apitch", "R_aroll",
]


def check_motor_torque_health(
    log_path: Path,
    window_s: float = 2.0,
    *,
    std_threshold_nm: float = 5e-4,
    unique_threshold: int = 2,
) -> tuple[bool, list[int]]:
    """Read the tail of ``log_path`` and flag motors with dead-motor signature.

    A motor is DEAD if its torque samples over the last ``window_s`` have both
    ``std < std_threshold_nm`` AND ``n_unique_values <= unique_threshold``.
    Thresholds are chosen so a healthy motor at rest (noise floor ~5-10 mN·m,
    dozens of unique values per second) clears by >10x.
    Returns ``(all_ok, failing_motor_ids)``.
    """
    if not log_path.exists():
        print(f"[torque_check][WARN] log file missing: {log_path}")
        return True, []

    with log_path.open("r", newline="") as fh:
        lines = fh.readlines()
    if len(lines) < 3:
        print(f"[torque_check][WARN] log has <3 rows; skipping check.")
        return True, []

    header = lines[0].rstrip("\n").split(",")
    try:
        time_col = header.index("time_s")
        tau_cols = {i: header.index(f"m{i}_tau_nm") for i in range(1, 13)}
    except ValueError as exc:
        print(f"[torque_check][WARN] unexpected CSV header ({exc}); skipping check.")
        return True, []

    taus: dict[int, list[float]] = {i: [] for i in range(1, 13)}
    times: list[float] = []
    # Only walk back as many lines as we could conceivably need (100Hz default).
    max_rows = max(400, int(window_s * 200) + 50)
    for ln in lines[-max_rows:]:
        row = ln.rstrip("\n").split(",")
        if len(row) < len(header):
            continue
        try:
            t = float(row[time_col])
        except ValueError:
            continue
        times.append(t)
        for i in range(1, 13):
            try:
                taus[i].append(float(row[tau_cols[i]]))
            except ValueError:
                taus[i].append(float("nan"))

    if len(times) < 20:
        print(f"[torque_check][WARN] only {len(times)} samples available; skipping check.")
        return True, []

    t_end = times[-1]
    t_start = t_end - window_s
    idxs = [k for k, t in enumerate(times) if t >= t_start]
    if len(idxs) < 20:
        print(f"[torque_check][WARN] only {len(idxs)} samples in {window_s:.1f}s window; skipping check.")
        return True, []

    n = len(idxs)
    failing: list[int] = []
    print(f"[torque_check] window: last {window_s:.1f}s ({n} samples); thresholds: std<{std_threshold_nm} Nm AND unique<={unique_threshold}")
    print(f"[torque_check]   {'mot':>4}  {'name':>8}  {'mean_Nm':>9}  {'std_Nm':>9}  {'uniq':>4}  result")
    for i in range(1, 13):
        xs = [taus[i][k] for k in idxs]
        if any(x != x for x in xs):  # NaN check
            xs = [x for x in xs if x == x]
            if len(xs) < 20:
                continue
        mu = sum(xs) / len(xs)
        var = sum((x - mu) ** 2 for x in xs) / len(xs)
        sigma = var ** 0.5
        uniq = len({round(x, 6) for x in xs})
        is_dead = (sigma < std_threshold_nm) and (uniq <= unique_threshold)
        status = "DEAD" if is_dead else "ok"
        name = MOTOR_NAMES[i - 1]
        print(f"[torque_check]    m{i:<2}  {name:>8}  {mu:+9.4f}  {sigma:9.5f}  {uniq:>4}  {status}")
        if is_dead:
            failing.append(i)

    if failing:
        parts = ", ".join(f"m{i} ({MOTOR_NAMES[i - 1]})" for i in failing)
        print(f"[torque_check] ❌ FAIL: dead-motor signature on: {parts}")
        print(f"[torque_check]        check cable / CAN / motor ID / driver enable before running the policy.")
    else:
        print(f"[torque_check] ✓ all 12 motors produced variable torque during hold.")
    return (not failing), failing


def build_imu(imu_sensor, i2c_bus, imu_address, frame_yaw_deg, jy901_port, jy901_baudrate):
    from IMU_integration import IMU
    if imu_sensor == "bno055":
        return IMU(sensor="bno055", i2c_bus=int(i2c_bus), address=int(imu_address),
                   rate_hz=100.0, frame_yaw_deg=float(frame_yaw_deg))
    if imu_sensor == "jy901":
        return IMU(sensor="jy901", port=str(jy901_port), baudrate=int(jy901_baudrate))
    return IMU(sensor="bno085", address=int(imu_address))


def snap_to_sides(joint_state_deg):
    """Snapshot joint order: [left(6), right(6)] in (hipz, hipx, hipy, knee, ankle_pitch, ankle_roll) order."""
    left = dict(zip(KEYS, joint_state_deg[0:6]))
    right = dict(zip(KEYS, joint_state_deg[6:12]))
    return left, right


def interp_sides(left0, right0, left1, right1, alpha):
    """alpha in [0, 1].  alpha=0 returns (left0, right0), alpha=1 returns (left1, right1)."""
    lefti = {k: left0[k] + alpha * (left1[k] - left0[k]) for k in KEYS}
    righti = {k: right0[k] + alpha * (right1[k] - right0[k]) for k in KEYS}
    return lefti, righti


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ramp-s", type=float, default=4.0,
                    help="Seconds to ramp from current pose to zero.")
    ap.add_argument("--hold-s", type=float, default=2.0,
                    help="Seconds to hold zero before stopping / snapshotting.")
    ap.add_argument("--tick-hz", type=float, default=100.0,
                    help="Ramp command tick rate.")
    ap.add_argument("--max-command-delta-deg", type=float, default=60.0,
                    help="Per-step command delta limit on the controller (safety).")
    ap.add_argument("--snapshot-out", type=Path, default=None,
                    help="Write final snapshot JSON to this path after holding.")
    ap.add_argument("--stop-after", action=argparse.BooleanOptionalAction, default=True,
                    help="Stop the robot (disable motors) after the hold.")
    ap.add_argument("--skip-torque-check", action="store_true",
                    help="Skip the post-hold torque-health check.")
    ap.add_argument("--torque-check-window-s", type=float, default=2.0,
                    help="Seconds at the end of the hold to use for torque check.")
    ap.add_argument("--imu-sensor", choices=("bno055", "jy901", "bno085"), default="bno055")
    ap.add_argument("--i2c-bus", type=int, default=1)
    ap.add_argument("--imu-address", type=lambda x: int(x, 0), default=0x28)
    ap.add_argument("--frame-yaw-deg", type=float, default=-180.0)
    ap.add_argument("--jy901-port", type=str, default="/dev/ttyAMA0")
    ap.add_argument("--jy901-baudrate", type=int, default=9600)
    args = ap.parse_args()

    from bipedal_robot import BipedalRobotController
    from deploy.common import apply_base_gain_profile  # low gains suitable for holding

    imu = build_imu(args.imu_sensor, args.i2c_bus, args.imu_address,
                    args.frame_yaw_deg, args.jy901_port, args.jy901_baudrate)

    # Per-run timestamp so successive calls don't overwrite each other.
    run_ts = time.strftime("%Y%m%d_%H%M%S")
    state_log_path = Path.cwd() / f"bipedal_state_log_{run_ts}.csv"
    print(f"[ramp] run_ts: {run_ts}")
    print(f"[ramp] state_log: {state_log_path}")

    robot = None
    torque_check_ok = True
    try:
        robot = BipedalRobotController(
            control_hz=float(args.tick_hz),
            imu=imu,
            log_path=str(state_log_path),
        )
        robot.set_max_command_delta(float(args.max_command_delta_deg))

        # Read state first (motors off), then control+enable.
        robot.start(mode="state_only", auto_enable=False)
        robot.request_state_once()
        time.sleep(0.3)

        # Enable motors in control mode. Control loop auto-latches current position
        # as the target so the robot doesn't jump when we enable.
        robot.set_mode("control")
        robot.enable_all()

        # Apply gentle gains for the ramp.
        apply_base_gain_profile(robot)

        time.sleep(0.5)   # let state stream fully
        snap = robot.get_combined_state_snapshot(include_joint_state=True)
        js = snap["joint_state_deg"]
        left0, right0 = snap_to_sides(js)
        left_zero = {k: 0.0 for k in KEYS}
        right_zero = {k: 0.0 for k in KEYS}

        print(f"[ramp] starting joint positions (deg):")
        print(f"        left:  {left0}")
        print(f"        right: {right0}")
        print(f"[ramp] ramping to zero over {args.ramp_s}s at {args.tick_hz} Hz ...")

        tick_dt = 1.0 / float(args.tick_hz)
        t0 = time.monotonic()
        ramp_end = t0 + float(args.ramp_s)
        next_tick = t0
        while time.monotonic() < ramp_end:
            now = time.monotonic()
            alpha = (now - t0) / float(args.ramp_s)
            alpha = max(0.0, min(1.0, alpha))
            l, r = interp_sides(left0, right0, left_zero, right_zero, alpha)
            robot.set_action(left=l, right=r)
            next_tick += tick_dt
            sleep = next_tick - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_tick = time.monotonic()
        # Final zero setpoint
        robot.set_action(left=left_zero, right=right_zero)
        print(f"[ramp] at zero. holding {args.hold_s}s ...")

        # Hold zero
        hold_end = time.monotonic() + float(args.hold_s)
        while time.monotonic() < hold_end:
            robot.set_action(left=left_zero, right=right_zero)
            time.sleep(tick_dt)

        # Post-hold torque-health preflight: flags motors stuck at a constant
        # (dead) torque reading. Cheaper than finding out mid-policy.
        if not args.skip_torque_check:
            log_path = Path(getattr(robot, "log_path", "bipedal_state_log.csv"))
            window_s = min(float(args.torque_check_window_s), max(0.5, float(args.hold_s) - 0.2))
            torque_check_ok, _ = check_motor_torque_health(log_path, window_s=window_s)

        # Final snapshot (what the policy would see if it started now)
        snap_final = robot.get_combined_state_snapshot(include_joint_state=True)
        js_final = snap_final["joint_state_deg"]
        print(f"[ramp] settled joint positions (deg):")
        for i, name in enumerate(
            ["hipz_L","hipx_L","hipy_L","knee_L","ankley_L","anklex_L",
             "hipz_R","hipx_R","hipy_R","knee_R","ankley_R","anklex_R"]):
            print(f"   {name:>10}  {js_final[i]:+7.2f}°")
        if args.snapshot_out:
            payload = {
                "time_s": snap_final.get("time_s"),
                "joint_state_deg": snap_final.get("joint_state_deg"),
                "joint_velocity_rad_s": snap_final.get("joint_velocity_rad_s"),
                "joint_velocity_deg_s": snap_final.get("joint_velocity_deg_s"),
                "projected_gravity": snap_final.get("projected_gravity"),
                "orientation_quaternion_xyzw": snap_final.get("orientation_quaternion_xyzw"),
                "imu": snap_final.get("imu"),
            }
            args.snapshot_out.parent.mkdir(parents=True, exist_ok=True)
            args.snapshot_out.write_text(json.dumps(payload, indent=2, default=str))
            print(f"[ramp] wrote snapshot -> {args.snapshot_out}")

        if not args.stop_after:
            print(f"[ramp] --no-stop-after: holding zero indefinitely (Ctrl+C to stop)")
            while True:
                robot.set_action(left=left_zero, right=right_zero)
                time.sleep(tick_dt)

    except KeyboardInterrupt:
        print("\n[ramp] interrupted")
    finally:
        if robot is not None:
            try:
                robot.stop()
            except Exception as exc:
                print(f"[ramp][WARN] robot.stop: {exc}", file=sys.stderr)

    return 0 if torque_check_ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
