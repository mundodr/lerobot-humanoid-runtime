#!/usr/bin/env python3
from __future__ import annotations

import argparse
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional


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
            "[scale-console] Commands: `scale <v>`, `+`, `-`, `status`, `stop`.",
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
            if low in {"+", "up"}:
                self._queue.put(("delta", +self.step))
                continue
            if low in {"-", "down"}:
                self._queue.put(("delta", -self.step))
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
    p.add_argument("--max-command-delta-deg", type=float, default=60.0)
    p.add_argument("--viz-hz", type=float, default=20.0)
    p.add_argument("--use-mock-bus", action=argparse.BooleanOptionalAction, default=False)
    p.add_argument("--use-zqwl-bus", action=argparse.BooleanOptionalAction, default=False)
    p.add_argument("--zqwl-port", type=str, default=None, help="ZQWL 串口路径，默认自动检测 /dev/ttyACM*")
    p.add_argument("--zqwl-serial-baudrate", type=int, default=460_800)
    p.add_argument("--mock-default-temp-c", type=float, default=30.0)
    p.add_argument("--mock-send-sleep-s", type=float, default=0.0)

    p.add_argument("--with-imu", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--imu-sensor", type=str, default="bno055", choices=("bno055", "jy901", "bno085", "mock"))
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
    return p.parse_args()


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
    from imu.IMU_integration import IMU

    if args.imu_sensor == "mock":
        return IMU(sensor="mock")
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


def build_mock_buses(args: argparse.Namespace) -> tuple[Any, Any]:
    from lerobot.motors import Motor, MotorNormMode
    from lerobot_humanoid_lerobot_integration.lerobot_humanoid import HUMANOID_MOTOR_TYPE_BY_ID
    from lerobot_humanoid_lerobot_integration.robstride_mock_bus import RobstrideMockBus
    from robot.root_constant import CAN0_MOTOR_IDS, CAN1_MOTOR_IDS, JOINT_LIMITS_DEG

    def _build_motors(ids: tuple[int, ...]) -> dict[str, Motor]:
        out: dict[str, Motor] = {}
        for mid in ids:
            out[f"m{mid}"] = Motor(
                id=int(mid),
                model="robstride",
                norm_mode=MotorNormMode.DEGREES,
                motor_type_str=str(HUMANOID_MOTOR_TYPE_BY_ID.get(int(mid), "o0")),
                recv_id=int(mid),
            )
        return out

    # Start each mock motor in the middle of its configured raw limit range so
    # controller-side state guards pass on startup.
    initial_position_deg_by_motor_id = {
        int(mid): 0.5 * (float(lim[0]) + float(lim[1])) for mid, lim in JOINT_LIMITS_DEG.items()
    }
    # Coupled ankles should start with equal motor values to keep pitch near 0.
    for a, b in ((5, 6), (11, 12)):
        lim_a = JOINT_LIMITS_DEG[int(a)]
        lim_b = JOINT_LIMITS_DEG[int(b)]
        lo = max(float(lim_a[0]), float(lim_b[0]))
        hi = min(float(lim_a[1]), float(lim_b[1]))
        shared = 0.0 if lo > hi else 0.5 * (lo + hi)
        initial_position_deg_by_motor_id[int(a)] = shared
        initial_position_deg_by_motor_id[int(b)] = shared

    bus_can0 = RobstrideMockBus(
        motors=_build_motors(CAN0_MOTOR_IDS),
        default_temp_c=float(args.mock_default_temp_c),
        send_sleep_s=float(args.mock_send_sleep_s),
        initial_position_deg_by_motor_id=initial_position_deg_by_motor_id,
    )
    bus_can1 = RobstrideMockBus(
        motors=_build_motors(CAN1_MOTOR_IDS),
        default_temp_c=float(args.mock_default_temp_c),
        send_sleep_s=float(args.mock_send_sleep_s),
        initial_position_deg_by_motor_id=initial_position_deg_by_motor_id,
    )
    return bus_can0, bus_can1


def build_zqwl_buses(args: argparse.Namespace) -> tuple[Any, Any]:
    from robot.zqwl_serial_bus import open_zqwl_can_buses

    return open_zqwl_can_buses(
        port=args.zqwl_port,
        serial_baudrate=int(args.zqwl_serial_baudrate),
    )


def main() -> int:
    args = parse_args()
    config_path, policy_path = resolve_policy_files(Path(args.policy_dir))
    from control.rl_agent import RLAgent
    from robot.bipedal_robot import BipedalRobotController

    robot = None
    pad = None
    agent = None
    console = None

    try:
        print("[stage] 1/4 create robot in state_only", flush=True)
        mock_bus_can0 = None
        mock_bus_can1 = None
        if args.use_mock_bus and args.use_zqwl_bus:
            raise SystemExit("[error] --use-mock-bus 与 --use-zqwl-bus 不能同时使用")
        if args.use_mock_bus:
            print("[stage] using mock CAN buses (no hardware CAN access)", flush=True)
            mock_bus_can0, mock_bus_can1 = build_mock_buses(args)
        elif args.use_zqwl_bus:
            print("[stage] using ZQWL serial CAN buses (ttyACM)", flush=True)
            mock_bus_can0, mock_bus_can1 = build_zqwl_buses(args)
        robot = BipedalRobotController(
            control_hz=float(args.control_hz),
            imu=build_imu(args),
            bus_can0=mock_bus_can0,
            bus_can1=mock_bus_can1,
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

        print(f"[deploy] running policy: {policy_path}", flush=True)
        print(f"[deploy] config: {config_path}", flush=True)
        print(f"[deploy] log: {log_path}", flush=True)
        print(f"[deploy] action_scale: {agent.spec.action_scale:.3f}", flush=True)

        interactive = bool(args.interactive_scale and sys.stdin.isatty())
        console = ScaleConsole(step=float(args.scale_step), enabled=interactive)
        console.start()
        if interactive:
            print("[deploy] interactive action scale enabled.", flush=True)
        else:
            print("[deploy] interactive action scale disabled.", flush=True)

        while True:
            time.sleep(0.1)
            if console is None:
                continue
            cmd = console.pop()
            while cmd is not None:
                kind, value = cmd
                if kind == "stop":
                    raise KeyboardInterrupt
                if kind == "status":
                    print(f"[deploy] action_scale={agent.spec.action_scale:.3f}", flush=True)
                elif kind == "set" and value is not None:
                    agent.spec.action_scale = float(max(0.0, value))
                    print(f"[deploy] action_scale -> {agent.spec.action_scale:.3f}", flush=True)
                elif kind == "delta" and value is not None:
                    agent.spec.action_scale = float(max(0.0, agent.spec.action_scale + float(value)))
                    print(f"[deploy] action_scale -> {agent.spec.action_scale:.3f}", flush=True)
                cmd = console.pop()
    except KeyboardInterrupt:
        print("[deploy] stopping...", flush=True)
    finally:
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
