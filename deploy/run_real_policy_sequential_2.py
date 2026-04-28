#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional


class ActionScaleLogger:
    """Event-stream logger for the split hip / knee / ankle action scales.

    Writes one row on open (the initial values) plus one row every time any
    value changes. Forward-fill by ``time_s`` in analysis to align with the
    RL agent's obs/action log.

    Column name ``upper_scale`` now specifically means "hip/above-knee
    joints" (hipz, hipx, hipy) -- no longer includes knee -- because the
    knee group has its own scale.
    """

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self._fh: Optional[Any] = None
        self._writer: Optional[Any] = None

    def open(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = self.path.open("w", newline="")
        self._writer = csv.writer(self._fh)
        self._writer.writerow(["time_s", "upper_scale", "knee_scale", "ankle_scale", "source"])
        self._fh.flush()

    def log(self, upper_scale: float, knee_scale: float, ankle_scale: float, source: str = "") -> None:
        if self._writer is None or self._fh is None:
            return
        self._writer.writerow([
            f"{time.time():.6f}",
            f"{float(upper_scale):.6f}",
            f"{float(knee_scale):.6f}",
            f"{float(ankle_scale):.6f}",
            str(source),
        ])
        self._fh.flush()

    def close(self) -> None:
        if self._fh is not None:
            try:
                self._fh.flush()
                self._fh.close()
            except Exception:
                pass
        self._fh = None
        self._writer = None


# Added by run_real_policy_sequential_2.py: use Tao's obs-layout-aware RL stack
# so history is assembled in Isaac Lab term-major order (not chronological).
from pathlib import Path as _P
TAO_ROOT = _P("/home/lerobot/devel/Tao/lerobot-humanoid-design/to_real_robot")
if str(TAO_ROOT) not in sys.path:
    sys.path.insert(0, str(TAO_ROOT))

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from deploy.common import apply_base_gain_profile, resolve_policy_files, set_zero_pose


class ScaleConsole:
    def __init__(self, *, step: float, enabled: bool) -> None:
        self.step = float(step)
        self.enabled = bool(enabled)
        self._queue: queue.SimpleQueue[tuple[str, float | None]] = queue.SimpleQueue()
        self._stop = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if not self.enabled:
            return
        self._thread = threading.Thread(target=self._run, daemon=True, name="scale-console")
        self._thread.start()

    def stop(self) -> None:
        self._stop = True

    def pop(self) -> Optional[tuple[str, float | None]]:
        try:
            return self._queue.get_nowait()
        except queue.Empty:
            return None

    def _run(self) -> None:
        print(
            "[scale-console] Commands:\n"
            "  `5` / `6`  : hip (above-knee) scale up/down\n"
            "  `3` / `4`  : knee scale up/down\n"
            "  `1` / `2`  : ankle scale up/down\n"
            "  `0`        : zero all three scales (panic)\n"
            "  `save`     : bookmark current scales\n"
            "  `r`        : restore last bookmark (auto-bookmarked before tilt/accel safety)\n"
            "  `upper <v>` / `knee <v>` / `ankle <v>` : set one group\n"
            "  `scale <v>` : set all three groups\n"
            "  `status` / `stop`",
            flush=True,
        )
        while not self._stop:
            try:
                line = input().strip()
            except EOFError:
                return
            except KeyboardInterrupt:
                self._queue.put(("stop", None))
                return
            if not line:
                continue
            low = line.lower()
            if low in {"stop", "quit", "exit"}:
                self._queue.put(("stop", None))
                return
            if low in {"status", "s"}:
                self._queue.put(("status", None))
                continue
            if low == "0":
                self._queue.put(("set", 0.0))
                continue
            if low in {"save", "bookmark"}:
                self._queue.put(("save", None))
                continue
            if low in {"r", "restore"}:
                self._queue.put(("restore", None))
                continue
            if low == "5":
                self._queue.put(("upper_delta", +self.step))
                continue
            if low == "6":
                self._queue.put(("upper_delta", -self.step))
                continue
            if low == "1":
                self._queue.put(("ankle_delta", +self.step))
                continue
            if low == "2":
                self._queue.put(("ankle_delta", -self.step))
                continue
            if low == "3":
                self._queue.put(("knee_delta", +self.step))
                continue
            if low == "4":
                self._queue.put(("knee_delta", -self.step))
                continue
            if low.startswith("upper "):
                parts = low.split(maxsplit=1)
                try:
                    value = float(parts[1])
                except Exception:
                    print("[scale-console] Invalid value. Example: upper 0.25", flush=True)
                    continue
                self._queue.put(("upper_set", value))
                continue
            if low.startswith("knee "):
                parts = low.split(maxsplit=1)
                try:
                    value = float(parts[1])
                except Exception:
                    print("[scale-console] Invalid value. Example: knee 0.25", flush=True)
                    continue
                self._queue.put(("knee_set", value))
                continue
            if low.startswith("ankle "):
                parts = low.split(maxsplit=1)
                try:
                    value = float(parts[1])
                except Exception:
                    print("[scale-console] Invalid value. Example: ankle 0.25", flush=True)
                    continue
                self._queue.put(("ankle_set", value))
                continue
            if low.startswith("scale "):
                parts = low.split(maxsplit=1)
                try:
                    value = float(parts[1])
                except Exception:
                    print("[scale-console] Invalid value. Example: scale 0.25", flush=True)
                    continue
                self._queue.put(("set", value))
                continue
            print("[scale-console] Unknown command.", flush=True)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Sequential real-robot deployment script with staged pauses:\n"
            "1) robot creation\n"
            "2) control+enable twice+zero action\n"
            "3) gains\n"
            "4) policy start"
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
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
    p.add_argument(
        "--max-command-delta-deg",
        type=float,
        default=15.0,
        help="Per-tick command clamp (deg). At 100 Hz, 15 deg/tick = 1500 deg/s, "
             "below motor vmax ~1900 deg/s. Bipedal controller now CLAMPS rather "
             "than rejects on breach (was 60 with reject behavior).",
    )
    p.add_argument("--viz-hz", type=float, default=20.0)

    p.add_argument("--with-imu", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--imu-sensor", type=str, default="bno055", choices=("bno055", "jy901", "bno085"))
    p.add_argument("--i2c-bus", type=int, default=1)
    p.add_argument("--imu-address", type=lambda x: int(x, 0), default=0x28)
    p.add_argument("--frame-yaw-deg", type=float, default=-180.0)
    p.add_argument("--jy901-port", type=str, default="/dev/ttyAMA0")
    p.add_argument("--jy901-baudrate", type=int, default=9600)

    p.add_argument("--with-meshcat", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--with-gamepad", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--gamepad-name", type=str, default="8bitdo")
    p.add_argument("--deadzone", type=float, default=0.12)
    p.add_argument("--max-lin-x", type=float, default=0.75)
    p.add_argument("--max-lin-y", type=float, default=0.5)
    p.add_argument("--max-yaw-rate", type=float, default=0.8)
    p.add_argument("--cmd-lin-x", type=float, default=0.0)
    p.add_argument("--cmd-lin-y", type=float, default=0.0)
    p.add_argument("--cmd-yaw-rate", type=float, default=0.0)

    p.add_argument("--pause-between-stages", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--auto-pause-seconds", type=float, default=0.0)
    p.add_argument("--enable-repeat-count", type=int, default=2)
    p.add_argument("--enable-repeat-delay-s", type=float, default=0.25)
    p.add_argument("--gain-profile", type=str, default="tao", choices=("none", "tao"))

    p.add_argument("--interactive-scale", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--scale-step", type=float, default=0.05)

    p.add_argument(
        "--live-status-hz",
        type=float,
        default=1.0,
        help="Rate (Hz) to print base tilt + scales in the main loop. 0 to disable.",
    )
    p.add_argument(
        "--tilt-safety-deg",
        type=float,
        default=25.0,
        help="If base tilt exceeds this angle (deg), auto-reduce upper/ankle scales. 0 to disable.",
    )
    p.add_argument(
        "--tilt-safety-action",
        type=str,
        default="zero",
        choices=("zero", "halve"),
        help="On tilt-safety trigger: 'zero' sets both scales to 0, 'halve' multiplies both by 0.5.",
    )
    p.add_argument(
        "--accel-safety-g",
        type=float,
        default=2.0,
        help="If base linear-acceleration magnitude (g, gravity-removed) exceeds this, "
             "zero all action scales. 0 to disable.",
    )
    p.add_argument(
        "--accel-safety-action",
        type=str,
        default="zero",
        choices=("zero", "halve"),
        help="On accel-safety trigger: 'zero' sets all three scales to 0, 'halve' multiplies them by 0.5.",
    )
    return p.parse_args()


def _compute_tilt_deg(robot: Any) -> Optional[tuple[float, float, float, float]]:
    """Tilt (deg) plus roll/pitch/yaw (deg) from the IMU quaternion.

    Returns ``(tilt, roll, pitch, yaw)`` or ``None``. ``tilt`` is the angle
    between the robot's body-up axis and world-up (derived from
    projected_gravity_z: pgz=-1 upright, 0 at 90 deg, +1 inverted).
    Roll/pitch/yaw are extracted as standard XYZ Tait-Bryan angles from the
    quaternion (x, y, z, w).
    """
    getter = getattr(robot, "get_orientation_quaternion", None)
    if getter is None:
        return None
    q = getter()
    if q is None or len(q) != 4:
        return None
    import math
    try:
        x, y, z, w = (float(v) for v in q)
    except Exception:
        return None
    pgz = 2.0 * (x * x + y * y) - 1.0
    pgz = max(-1.0, min(1.0, pgz))
    tilt = math.degrees(math.acos(-pgz))

    # roll (x), pitch (y), yaw (z) -- ZYX intrinsic / XYZ extrinsic Tait-Bryan
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    return tilt, roll, pitch, yaw


_ACCEL_G_MS2 = 9.80665


def _compute_accel_g(robot: Any) -> Optional[float]:
    """Magnitude of base linear acceleration (in g, gravity-removed) from the IMU.

    Reads the same ``linear_acceleration_mps2`` field that the state log uses.
    Returns ``None`` if the IMU isn't attached or hasn't published yet.
    """
    getter = getattr(robot, "get_imu_snapshot", None)
    if getter is None:
        return None
    snap = getter()
    if not isinstance(snap, dict):
        return None
    vec = None
    for key in ("linear_acceleration_mps2", "lin_acc_m_s2", "linear_acceleration"):
        v = snap.get(key)
        if v is None:
            continue
        try:
            seq = list(v)
        except TypeError:
            continue
        if len(seq) < 3:
            continue
        try:
            vec = (float(seq[0]), float(seq[1]), float(seq[2]))
        except Exception:
            continue
        break
    if vec is None:
        return None
    import math
    mag = math.sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2])
    return mag / _ACCEL_G_MS2


def stage_pause(stage: str, args: argparse.Namespace) -> None:
    if not args.pause_between_stages:
        return
    if float(args.auto_pause_seconds) > 0.0:
        t = float(args.auto_pause_seconds)
        print(f"[stage] {stage} | auto pause {t:.1f}s", flush=True)
        time.sleep(t)
        return
    input(f"[stage] {stage} | Press Enter to continue...")


def build_imu(args: argparse.Namespace) -> Any | None:
    if not args.with_imu:
        return None
    from IMU_integration import IMU  # Tao

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


def create_gamepad(args: argparse.Namespace) -> Any:
    from apps.gamepad_controller import GamepadController

    pad = GamepadController(
        name_substring=str(args.gamepad_name),
        deadzone=float(args.deadzone),
        max_lin_x=float(args.max_lin_x),
        max_lin_y=float(args.max_lin_y),
        max_yaw_rate=float(args.max_yaw_rate),
    )
    pad.connect()
    pad.start()
    return pad


def main() -> int:
    args = parse_args()
    config_path, policy_path = resolve_policy_files(Path(args.policy_dir))
    from RL_agent_isolated import RLAgent  # Tao
    from bipedal_robot import BipedalRobotController  # Tao

    # Per-run timestamp used for every log file this script writes so back-to-back
    # runs don't overwrite each other.
    run_ts = time.strftime("%Y%m%d_%H%M%S")
    state_log_path = Path(args.policy_dir) / f"bipedal_state_log_{run_ts}.csv"
    print(f"[deploy] run_ts: {run_ts}", flush=True)
    print(f"[deploy] state_log: {state_log_path}", flush=True)

    robot = None
    pad = None
    agent = None
    console = None
    scale_logger: Optional[ActionScaleLogger] = None

    try:
        print("[stage] 1/4 create robot in state_only", flush=True)
        robot = BipedalRobotController(
            control_hz=float(args.control_hz),
            imu=build_imu(args),
            log_path=str(state_log_path),
        )
        if args.with_meshcat:
            robot.attach_default_meshcat()
            robot._viz_hz = float(args.viz_hz)
        robot.set_max_command_delta(float(args.max_command_delta_deg))
        robot.start(mode="state_only", auto_enable=False)
        robot.request_state_once()
        stage_pause("check meshcat / limits / imu", args)

        print("[stage] 2/4 control mode + enable twice + zero action", flush=True)
        robot.set_mode("control")
        repeat = max(1, int(args.enable_repeat_count))
        for _ in range(repeat):
            robot.enable_all()
            time.sleep(max(0.0, float(args.enable_repeat_delay_s)))
        set_zero_pose(robot)
        stage_pause("check each joint reacts to zero action", args)

        print("[stage] 3/4 set gains", flush=True)
        if args.gain_profile == "tao":
            apply_base_gain_profile(robot)
        stage_pause("check gains and posture", args)

        print("[stage] 4/4 start policy", flush=True)
        if args.with_gamepad:
            pad = create_gamepad(args)
        # Append the run timestamp to the ctrl log filename (keeps the --log-name
        # stem so tooling still recognises it) unless --log-path is given explicitly.
        if args.log_path is not None:
            log_path = Path(args.log_path)
        else:
            stem = Path(args.log_name).stem
            suffix = Path(args.log_name).suffix or ".csv"
            log_path = Path(args.policy_dir) / f"{stem}_{run_ts}{suffix}"
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

        # Split the single user-facing action_scale into per-group scales
        # (hip/above-knee vs knee vs ankle) by driving the per-joint
        # ``action_scales_rad`` array directly. The policy-config baseline is
        # preserved so that upper=knee=ankle=1.0 reproduces the policy's
        # natural scales. Group assignment is by substring match on the
        # action key (IsaacLab uses keys like "knee_left", "ankley_right",
        # "hipx_right", etc. -- both POLICY and ISAACLAB orderings include
        # the literal substrings "knee" and "ankle").
        action_keys = list(agent.spec.action_keys)
        baseline_scales_rad = [
            float(agent.spec.action_scales_rad[i]) if i < len(agent.spec.action_scales_rad) else 1.0
            for i in range(len(action_keys))
        ]
        ankle_mask = ["ankle" in k.lower() for k in action_keys]
        knee_mask = [("knee" in k.lower()) and not ankle_mask[i] for i, k in enumerate(action_keys)]
        # upper = everything else (hipz / hipx / hipy)
        upper_mask = [not (ankle_mask[i] or knee_mask[i]) for i in range(len(action_keys))]
        upper_scale = float(args.action_scale)
        knee_scale = float(args.action_scale)
        ankle_scale = float(args.action_scale)

        def _apply_split_scales() -> None:
            new = []
            for i in range(len(action_keys)):
                if ankle_mask[i]:
                    s = ankle_scale
                elif knee_mask[i]:
                    s = knee_scale
                else:
                    s = upper_scale
                new.append(baseline_scales_rad[i] * s)
            agent.spec.action_scales_rad = new

        # action_scale is now a dummy multiplier; per-group scaling lives in
        # action_scales_rad so the policy thread picks it up without special
        # cooperation.
        agent.spec.action_scale = 1.0
        _apply_split_scales()

        n_ankle = sum(ankle_mask)
        n_knee = sum(knee_mask)
        n_upper = sum(upper_mask)
        print(
            f"[deploy] split-scale: {n_upper} upper (hip), {n_knee} knee, {n_ankle} ankle "
            f"(of {len(action_keys)} total)",
            flush=True,
        )

        scale_logger = ActionScaleLogger(Path(log_path).parent / f"action_scale_log_{run_ts}.csv")
        scale_logger.open()
        scale_logger.log(upper_scale, knee_scale, ankle_scale, source="init")
        if pad is not None:
            agent.set_command_source(pad)
        else:
            agent.set_command_twist(float(args.cmd_lin_x), float(args.cmd_lin_y), float(args.cmd_yaw_rate))
        agent.start()

        print(f"[deploy] running policy: {policy_path}", flush=True)
        print(f"[deploy] config: {config_path}", flush=True)
        print(f"[deploy] log: {log_path}", flush=True)
        print(
            f"[deploy] upper_scale: {upper_scale:.3f}  knee_scale: {knee_scale:.3f}  "
            f"ankle_scale: {ankle_scale:.3f}",
            flush=True,
        )

        interactive = bool(args.interactive_scale and sys.stdin.isatty())
        console = ScaleConsole(step=float(args.scale_step), enabled=interactive)
        console.start()
        if interactive:
            print("[deploy] interactive action scale enabled.", flush=True)
        else:
            print("[deploy] interactive action scale disabled.", flush=True)

        # Tilt-safety + live-status setup. We tick the main loop at 10 Hz
        # (existing time.sleep(0.1)), so we derive print/check cadence from
        # wall-clock intervals rather than a counter.
        live_status_period = (1.0 / float(args.live_status_hz)) if float(args.live_status_hz) > 0 else None
        tilt_threshold = float(args.tilt_safety_deg)
        tilt_safety_enabled = tilt_threshold > 0.0
        accel_threshold_g = float(args.accel_safety_g)
        accel_safety_enabled = accel_threshold_g > 0.0
        last_status_print_s = 0.0
        tilt_safety_tripped = False  # one-shot per breach; resets when tilt drops below threshold
        accel_safety_tripped = False  # one-shot per breach; resets when accel drops back near 1g
        saved_state: Optional[tuple[float, float, float]] = None  # last (upper, knee, ankle) bookmark

        while True:
            time.sleep(0.1)
            now_s = time.time()

            # Tilt + accel read (cheap) -- used for live-status + safety.
            tilt_tuple = _compute_tilt_deg(robot)
            if tilt_tuple is not None:
                tilt_deg, roll_deg, pitch_deg, yaw_deg = tilt_tuple
            else:
                tilt_deg = roll_deg = pitch_deg = yaw_deg = None
            accel_g = _compute_accel_g(robot)

            # Live status line.
            if live_status_period is not None and (now_s - last_status_print_s) >= live_status_period:
                last_status_print_s = now_s
                tilt_txt = f"{tilt_deg:6.2f}" if tilt_deg is not None else "   n/a"
                roll_txt = f"{roll_deg:7.2f}" if roll_deg is not None else "    n/a"
                pitch_txt = f"{pitch_deg:7.2f}" if pitch_deg is not None else "    n/a"
                yaw_txt = f"{yaw_deg:7.2f}" if yaw_deg is not None else "    n/a"
                accel_txt = f"{accel_g:5.2f}" if accel_g is not None else "  n/a"
                tilt_flag = (tilt_deg is not None and tilt_safety_enabled and tilt_deg >= tilt_threshold)
                accel_flag = (accel_g is not None and accel_safety_enabled and accel_g >= accel_threshold_g)
                flag = "!" if (tilt_flag or accel_flag) else " "
                print(
                    f"[live]{flag} tilt={tilt_txt} deg  "
                    f"roll={roll_txt} pitch={pitch_txt} yaw={yaw_txt}  "
                    f"acc={accel_txt} g  "
                    f"upper={upper_scale:.2f}  knee={knee_scale:.2f}  ankle={ankle_scale:.2f}",
                    flush=True,
                )

            # Tilt safety: if we breach the threshold and haven't already
            # tripped, slam the scales down. User must manually ramp back up.
            if tilt_safety_enabled and tilt_deg is not None:
                if tilt_deg >= tilt_threshold and not tilt_safety_tripped:
                    tilt_safety_tripped = True
                    old_upper, old_knee, old_ankle = upper_scale, knee_scale, ankle_scale
                    saved_state = (old_upper, old_knee, old_ankle)
                    if args.tilt_safety_action == "halve":
                        upper_scale = 0.5 * upper_scale
                        knee_scale = 0.5 * knee_scale
                        ankle_scale = 0.5 * ankle_scale
                    else:  # "zero"
                        upper_scale = 0.0
                        knee_scale = 0.0
                        ankle_scale = 0.0
                    _apply_split_scales()
                    print(
                        f"[tilt-safety] tilt={tilt_deg:.2f} deg >= {tilt_threshold:.2f} deg, "
                        f"action={args.tilt_safety_action}: "
                        f"upper {old_upper:.2f}->{upper_scale:.2f}  "
                        f"knee {old_knee:.2f}->{knee_scale:.2f}  "
                        f"ankle {old_ankle:.2f}->{ankle_scale:.2f}",
                        flush=True,
                    )
                    if scale_logger is not None:
                        scale_logger.log(upper_scale, knee_scale, ankle_scale, source=f"tilt_safety_{args.tilt_safety_action}")
                elif tilt_deg < tilt_threshold - 2.0 and tilt_safety_tripped:
                    # Re-arm the safety once tilt drops >=2 deg below threshold
                    # (hysteresis) so a later breach will re-trigger.
                    tilt_safety_tripped = False

            # Accel safety: zero (or halve) all action scales on a high-g event
            # (impacts, falls). One-shot with hysteresis so the user has to
            # ramp scales back up manually after the spike.
            if accel_safety_enabled and accel_g is not None:
                if accel_g >= accel_threshold_g and not accel_safety_tripped:
                    accel_safety_tripped = True
                    old_upper, old_knee, old_ankle = upper_scale, knee_scale, ankle_scale
                    saved_state = (old_upper, old_knee, old_ankle)
                    if args.accel_safety_action == "halve":
                        upper_scale = 0.5 * upper_scale
                        knee_scale = 0.5 * knee_scale
                        ankle_scale = 0.5 * ankle_scale
                    else:  # "zero"
                        upper_scale = 0.0
                        knee_scale = 0.0
                        ankle_scale = 0.0
                    _apply_split_scales()
                    print(
                        f"[accel-safety] |a|={accel_g:.2f} g >= {accel_threshold_g:.2f} g, "
                        f"action={args.accel_safety_action}: "
                        f"upper {old_upper:.2f}->{upper_scale:.2f}  "
                        f"knee {old_knee:.2f}->{knee_scale:.2f}  "
                        f"ankle {old_ankle:.2f}->{ankle_scale:.2f}",
                        flush=True,
                    )
                    if scale_logger is not None:
                        scale_logger.log(
                            upper_scale, knee_scale, ankle_scale,
                            source=f"accel_safety_{args.accel_safety_action}",
                        )
                elif accel_g < max(0.5, accel_threshold_g - 0.5) and accel_safety_tripped:
                    # Re-arm once acceleration drops well below threshold
                    # (>=0.5 g of hysteresis) so a later spike re-triggers.
                    accel_safety_tripped = False

            if console is None:
                continue
            cmd = console.pop()
            while cmd is not None:
                kind, value = cmd
                if kind == "stop":
                    raise KeyboardInterrupt
                if kind == "status":
                    saved_txt = (
                        f"  saved=(upper={saved_state[0]:.3f}, knee={saved_state[1]:.3f}, ankle={saved_state[2]:.3f})"
                        if saved_state is not None else "  saved=<none>"
                    )
                    print(
                        f"[deploy] upper_scale={upper_scale:.3f}  knee_scale={knee_scale:.3f}  "
                        f"ankle_scale={ankle_scale:.3f}{saved_txt}",
                        flush=True,
                    )
                elif kind == "save":
                    saved_state = (upper_scale, knee_scale, ankle_scale)
                    print(
                        f"[deploy] bookmark saved: upper={saved_state[0]:.3f}  "
                        f"knee={saved_state[1]:.3f}  ankle={saved_state[2]:.3f}",
                        flush=True,
                    )
                elif kind == "restore":
                    if saved_state is None:
                        print("[deploy] no bookmark saved yet (use `save` first).", flush=True)
                    else:
                        upper_scale, knee_scale, ankle_scale = saved_state
                        _apply_split_scales()
                        print(
                            f"[deploy] restored bookmark: upper={upper_scale:.3f}  "
                            f"knee={knee_scale:.3f}  ankle={ankle_scale:.3f}",
                            flush=True,
                        )
                        if scale_logger is not None:
                            scale_logger.log(upper_scale, knee_scale, ankle_scale, source="restore")
                elif kind == "set" and value is not None:
                    v = float(max(0.0, value))
                    upper_scale = v
                    knee_scale = v
                    ankle_scale = v
                    _apply_split_scales()
                    print(
                        f"[deploy] upper_scale -> {upper_scale:.3f}  knee_scale -> {knee_scale:.3f}  "
                        f"ankle_scale -> {ankle_scale:.3f}",
                        flush=True,
                    )
                    if scale_logger is not None:
                        scale_logger.log(upper_scale, knee_scale, ankle_scale, source="set")
                elif kind == "upper_set" and value is not None:
                    upper_scale = float(max(0.0, value))
                    _apply_split_scales()
                    print(f"[deploy] upper_scale -> {upper_scale:.3f}", flush=True)
                    if scale_logger is not None:
                        scale_logger.log(upper_scale, knee_scale, ankle_scale, source="upper_set")
                elif kind == "knee_set" and value is not None:
                    knee_scale = float(max(0.0, value))
                    _apply_split_scales()
                    print(f"[deploy] knee_scale -> {knee_scale:.3f}", flush=True)
                    if scale_logger is not None:
                        scale_logger.log(upper_scale, knee_scale, ankle_scale, source="knee_set")
                elif kind == "ankle_set" and value is not None:
                    ankle_scale = float(max(0.0, value))
                    _apply_split_scales()
                    print(f"[deploy] ankle_scale -> {ankle_scale:.3f}", flush=True)
                    if scale_logger is not None:
                        scale_logger.log(upper_scale, knee_scale, ankle_scale, source="ankle_set")
                elif kind == "upper_delta" and value is not None:
                    upper_scale = float(max(0.0, upper_scale + float(value)))
                    _apply_split_scales()
                    print(f"[deploy] upper_scale -> {upper_scale:.3f}", flush=True)
                    if scale_logger is not None:
                        scale_logger.log(upper_scale, knee_scale, ankle_scale, source="upper_delta")
                elif kind == "knee_delta" and value is not None:
                    knee_scale = float(max(0.0, knee_scale + float(value)))
                    _apply_split_scales()
                    print(f"[deploy] knee_scale -> {knee_scale:.3f}", flush=True)
                    if scale_logger is not None:
                        scale_logger.log(upper_scale, knee_scale, ankle_scale, source="knee_delta")
                elif kind == "ankle_delta" and value is not None:
                    ankle_scale = float(max(0.0, ankle_scale + float(value)))
                    _apply_split_scales()
                    print(f"[deploy] ankle_scale -> {ankle_scale:.3f}", flush=True)
                    if scale_logger is not None:
                        scale_logger.log(upper_scale, knee_scale, ankle_scale, source="ankle_delta")
                cmd = console.pop()
    except KeyboardInterrupt:
        print("[deploy] stopping...", flush=True)
    finally:
        if scale_logger is not None:
            scale_logger.close()
        if console is not None:
            console.stop()
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
