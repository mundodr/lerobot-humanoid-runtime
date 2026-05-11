#!/usr/bin/env python

from __future__ import annotations

import logging
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from functools import cached_property
from typing import Any
import math

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.robstride import RobstrideMotorsBus
from lerobot.motors.robstride.tables import MOTOR_LIMIT_PARAMS, MotorType
from lerobot.processor import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from lerobot.robots import Robot
from .config_lerobot_humanoid import LeRobotHumanoidConfig
from .robstride_mock_bus import RobstrideMockBus

logger = logging.getLogger(__name__)


JOINT_ORDER = [
    "left_hipz",
    "left_hipx",
    "left_hipy",
    "left_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_hipz",
    "right_hipx",
    "right_hipy",
    "right_knee",
    "right_ankle_pitch",
    "right_ankle_roll",
]

# Keep humanoid constants local to this module so LeRobot hardware control does
# not depend on repository-level `robots.*` constants packages.
HUMANOID_MOTOR_IDS: tuple[int, ...] = tuple(range(1, 13))
HUMANOID_MOTORS: dict[int, str] = {
    1: "left_hipz",
    2: "left_hipx",
    3: "left_hipy",
    4: "left_knee",
    5: "left_ankle1",
    6: "left_ankle2",
    7: "right_hipz",
    8: "right_hipx",
    9: "right_hipy",
    10: "right_knee",
    11: "right_ankle1",
    12: "right_ankle2",
}

# Current routing: right IDs on can0, left IDs on can1.
HUMANOID_CAN1_MOTOR_IDS: tuple[int, ...] = (7, 8, 9, 10, 11, 12)
HUMANOID_CAN0_MOTOR_IDS: tuple[int, ...] = (1, 2, 3, 4, 5, 6)

HUMANOID_MOTOR_RECV_ID_BY_ID: dict[int, int] = {mid: mid for mid in HUMANOID_MOTOR_IDS}
HUMANOID_MOTOR_SIGN: dict[int, float] = {
    1: +1.0,
    2: +1.0,
    3: +1.0,
    4: -1.0,
    5: -1.0,
    6: -1.0,
    7: +1.0,
    8: +1.0,
    9: +1.0,
    10: +1.0,
    11: -1.0,
    12: -1.0,
}
HUMANOID_MOTOR_OFFSET_DEG: dict[int, float] = {
    1: 132.68,
    2: 19.394,
    3: 88.096,
    4: 57.352,
    5: 0.0,
    6: 0.0,
    7: -132.68,
    8: -19.394,
    9: -88.096,
    10: 57.352,
    11: 0.0,
    12: 0.0,
}
HUMANOID_MOTOR_TYPE_BY_ID: dict[int, str] = {
    1: "o0",
    2: "o2",
    3: "o3",
    4: "o3",
    5: "o5",
    6: "o5",
    7: "o0",
    8: "o2",
    9: "o3",
    10: "o3",
    11: "o5",
    12: "o5",
}
HUMANOID_ANKLE_COUPLING_CALIBRATION_LEFT: dict[str, Any] = {
    "motors": (5, 6),
    "pitch": {"sign": -1.0, "offset_deg": 0.0},
    "roll": {"sign": +1.0, "offset_deg": 0.0},
}
HUMANOID_ANKLE_COUPLING_CALIBRATION_RIGHT: dict[str, Any] = {
    "motors": (11, 12),
    "pitch": {"sign": -1.0, "offset_deg": 0.0},
    "roll": {"sign": +1.0, "offset_deg": 0.0},
}


class LeRobotHumanoid(Robot):
    config_class = LeRobotHumanoidConfig
    name = "lerobot_humanoid"

    def __init__(self, config: LeRobotHumanoidConfig):
        super().__init__(config)
        self.config = config

        self._motor_ids: tuple[int, ...] = HUMANOID_MOTOR_IDS
        self._can0_ids: tuple[int, ...] = HUMANOID_CAN0_MOTOR_IDS
        self._can1_ids: tuple[int, ...] = HUMANOID_CAN1_MOTOR_IDS
        self._motor_name_by_id = {int(k): str(v) for k, v in HUMANOID_MOTORS.items()}
        self._motor_recv_id_by_id = {int(k): int(v) for k, v in HUMANOID_MOTOR_RECV_ID_BY_ID.items()}
        self._motor_sign = {int(k): float(v) for k, v in HUMANOID_MOTOR_SIGN.items()}
        self._motor_offset_deg = {int(k): float(v) for k, v in HUMANOID_MOTOR_OFFSET_DEG.items()}
        self._joint_limits_deg = {int(k): (float(v[0]), float(v[1])) for k, v in config.joint_limits_deg.items()}
        self._motor_type_by_id = {int(k): str(v).lower() for k, v in HUMANOID_MOTOR_TYPE_BY_ID.items()}
        self._ankle_left_cfg = HUMANOID_ANKLE_COUPLING_CALIBRATION_LEFT
        self._ankle_right_cfg = HUMANOID_ANKLE_COUPLING_CALIBRATION_RIGHT

        self._joint_to_motor_direct = {
            "left_hipz": 1,
            "left_hipx": 2,
            "left_hipy": 3,
            "left_knee": 4,
            "right_hipz": 7,
            "right_hipx": 8,
            "right_hipy": 9,
            "right_knee": 10,
        }

        self._bus_can0 = RobstrideMotorsBus(
            port=config.can0_port,
            motors=self._build_bus_motors(self._can0_ids),
            calibration=self.calibration,
            can_interface=config.can_interface,
            use_can_fd=config.use_can_fd,
            bitrate=config.can_bitrate,
            data_bitrate=config.can_data_bitrate if config.use_can_fd else None,
        )
        self._bus_can1 = RobstrideMotorsBus(
            port=config.can1_port,
            motors=self._build_bus_motors(self._can1_ids),
            calibration=self.calibration,
            can_interface=config.can_interface,
            use_can_fd=config.use_can_fd,
            bitrate=config.can_bitrate,
            data_bitrate=config.can_data_bitrate if config.use_can_fd else None,
        )
        self._buses = [self._bus_can0, self._bus_can1]

        self._connected = False
        self._control_enabled = bool(config.auto_enable_control)
        self._stop_event = threading.Event()
        self._control_thread: threading.Thread | None = None
        # Send commands to both CAN buses in parallel to reduce control-loop latency.
        self._tx_executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="humanoid_tx")
        self._state_lock = threading.Lock()
        self._action_lock = threading.Lock()
        self._imu_lock = threading.Lock()
        self._safety_lock = threading.Lock()

        self._latest_states: dict[int, dict[str, float]] = {
            mid: {"position": 0.0, "velocity": 0.0, "torque": 0.0, "stamp": 0.0} for mid in self._motor_ids
        }
        self._desired_raw_positions: dict[int, float] = {mid: 0.0 for mid in self._motor_ids}
        self._imu_state: dict[str, Any] = {"available": False}
        self._imu_device: Any = None
        self._imu_thread: threading.Thread | None = None
        self._imu_stop_event = threading.Event()
        self._estop = False
        self._estop_reason = ""
        self._startup_wrap_checked = not bool(config.startup_wrap_correction_enabled)
        self._last_jump_warn_s: dict[int, float] = {mid: 0.0 for mid in self._motor_ids}

    def _build_bus_motors(self, motor_ids: tuple[int, ...]) -> dict[str, Motor]:
        out: dict[str, Motor] = {}
        for mid in motor_ids:
            name = self._motor_name_by_id[int(mid)]
            motor_type = self._motor_type_by_id.get(int(mid), "o0")
            out[name] = Motor(
                id=int(mid),
                model="robstride",
                norm_mode=MotorNormMode.DEGREES,
                motor_type_str=motor_type,
                recv_id=int(self._motor_recv_id_by_id.get(int(mid), int(mid))),
            )
        return out

    def _start_imu_thread(self) -> None:
        if not self.config.imu_enabled:
            with self._imu_lock:
                self._imu_state = {"available": False, "error": "IMU disabled"}
            return

        self._imu_stop_event.clear()
        self._imu_thread = threading.Thread(target=self._imu_loop, daemon=True)
        self._imu_thread.start()

    def _stop_imu_thread(self) -> None:
        self._imu_stop_event.set()
        if self._imu_thread is not None:
            self._imu_thread.join(timeout=2.0)
        self._imu_thread = None

        if self._imu_device is not None and hasattr(self._imu_device, "stop"):
            try:
                self._imu_device.stop()
            except Exception as e:
                logger.warning("Failed to stop BNO055 device: %s", e)
        self._imu_device = None

    def _imu_loop(self) -> None:
        if self.config.imu_mock:
            self._imu_loop_mock()
            return

        try:
            # Reuse the same extensionless BNO055 loader used by imu.IMU_integration.
            from imu.IMU_integration import BNO055I2CIMU

            BNO055IMU = BNO055I2CIMU._load_impl_class()
            self._imu_device = BNO055IMU(
                i2c_bus=int(self.config.imu_bno055_i2c_bus),
                address=int(self.config.imu_bno055_address),
                rate_hz=float(self.config.imu_poll_hz),
            )
            self._imu_device.start()
        except Exception as e:
            with self._imu_lock:
                self._imu_state = {"available": False, "error": f"{type(e).__name__}: {e}"}
            return

        period = 1.0 / max(1.0, float(self.config.imu_poll_hz))
        while not self._imu_stop_event.is_set():
            try:
                obs = self._imu_device.get_observation()
                quat = obs.quat
                ang = obs.ang_vel_rad_s
                lin_vel = obs.lin_vel_m_s
                lin_acc = obs.lin_acc_m_s2

                q_xyzw = None
                if quat is not None:
                    q_xyzw = (float(quat.x), float(quat.y), float(quat.z), float(quat.w))

                gyro = None
                if ang is not None:
                    gyro = (float(ang.x), float(ang.y), float(ang.z))

                linear_velocity = None
                if lin_vel is not None:
                    linear_velocity = (float(lin_vel.x), float(lin_vel.y), float(lin_vel.z))

                linear_acc = None
                if lin_acc is not None:
                    linear_acc = (float(lin_acc.x), float(lin_acc.y), float(lin_acc.z))

                with self._imu_lock:
                    self._imu_state = {
                        "timestamp_s": float(obs.timestamp),
                        "quaternion_xyzw": q_xyzw,
                        "gyro_rads": gyro,
                        "linear_velocity_mps": linear_velocity,
                        "linear_acceleration_mps2": linear_acc,
                        "sensor": "bno055_direct",
                        "available": True,
                    }
            except Exception as e:
                with self._imu_lock:
                    self._imu_state = {"available": False, "error": f"{type(e).__name__}: {e}"}

            time.sleep(period)

    def _imu_loop_mock(self) -> None:
        period = 1.0 / max(1.0, float(self.config.imu_poll_hz))
        q = tuple(float(v) for v in self.config.imu_mock_quaternion_xyzw)
        w = tuple(float(v) for v in self.config.imu_mock_gyro_rads)
        lv = tuple(float(v) for v in self.config.imu_mock_linear_velocity_mps)
        la = tuple(float(v) for v in self.config.imu_mock_linear_acceleration_mps2)
        while not self._imu_stop_event.is_set():
            with self._imu_lock:
                self._imu_state = {
                    "timestamp_s": time.time(),
                    "quaternion_xyzw": q,
                    "gyro_rads": w,
                    "linear_velocity_mps": lv,
                    "linear_acceleration_mps2": la,
                    "sensor": "mock",
                    "available": True,
                }
            time.sleep(period)

    @property
    def is_connected(self) -> bool:
        return self._connected and all(bus.is_connected for bus in self._buses)

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{joint}.pos": float for joint in JOINT_ORDER}

    @cached_property
    def observation_features(self) -> dict[str, type]:
        obs = {}
        for joint in JOINT_ORDER:
            obs[f"{joint}.pos"] = float
            obs[f"{joint}.vel"] = float
            obs[f"{joint}.torque"] = float
        obs["base.orientation.x"] = float
        obs["base.orientation.y"] = float
        obs["base.orientation.z"] = float
        obs["base.orientation.w"] = float
        obs["base.ang_vel.x"] = float
        obs["base.ang_vel.y"] = float
        obs["base.ang_vel.z"] = float
        return obs

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        with self._safety_lock:
            self._estop = False
            self._estop_reason = ""
        try:
            if self.config.use_mock_bus:
                mock_positions_by_id = {
                    int(mid): 0.5 * (float(lim[0]) + float(lim[1]))
                    for mid, lim in self._joint_limits_deg.items()
                }
                # Keep coupled ankles in a safe mock startup pose:
                # both motors of one ankle start equal so pitch/roll are near zero.
                for ankle_cfg in (self._ankle_left_cfg, self._ankle_right_cfg):
                    m_a, m_b = (int(ankle_cfg["motors"][0]), int(ankle_cfg["motors"][1]))
                    lim_a = self._joint_limits_deg.get(m_a)
                    lim_b = self._joint_limits_deg.get(m_b)
                    if lim_a is None or lim_b is None:
                        continue
                    lo = max(float(lim_a[0]), float(lim_b[0]))
                    hi = min(float(lim_a[1]), float(lim_b[1]))
                    shared = 0.0 if lo > hi else 0.5 * (lo + hi)
                    mock_positions_by_id[m_a] = shared
                    mock_positions_by_id[m_b] = shared
                for bus in self._buses:
                    bus.canbus = RobstrideMockBus(
                        motors=bus.motors,
                        default_temp_c=float(self.config.mock_bus_default_temp_c),
                        send_sleep_s=float(self.config.mock_bus_send_sleep_s),
                        initial_position_deg_by_motor_id=mock_positions_by_id,
                    )
                    bus._is_connected = True
                    if self.config.handshake:
                        bus._handshake()
            else:
                for bus in self._buses:
                    bus.connect(handshake=self.config.handshake)

            self.configure()

            if self.config.enable_torque_on_connect:
                for bus in self._buses:
                    bus.enable_torque()

            self._start_imu_thread()

            self._refresh_states_once()
            self._wait_for_all_motor_states_on_startup()
            if not self._startup_wrap_checked:
                self._startup_wrap_checked = self._auto_correct_startup_wrap()
            self._check_observed_state_safety()
            self._check_ankle_configuration_guard()
            with self._state_lock, self._action_lock:
                for mid in self._motor_ids:
                    self._desired_raw_positions[mid] = float(self._latest_states[mid]["position"])

            self._stop_event.clear()
            self._control_thread = threading.Thread(target=self._run_control_loop, daemon=True)
            self._control_thread.start()
            self._connected = True
            logger.info("%s connected", self)
        except Exception:
            self._stop_event.set()
            if self._control_thread is not None:
                self._control_thread.join(timeout=1.0)
            self._control_thread = None
            self._stop_imu_thread()
            if self.config.disable_torque_on_disconnect:
                for bus in self._buses:
                    try:
                        bus.disable_torque()
                    except Exception as e:
                        logger.warning("Failed to disable torque during connect rollback: %s", e)
            for bus in self._buses:
                try:
                    bus.disconnect(disable_torque=False)
                except Exception:
                    pass
            self._connected = False
            raise

    def configure(self) -> None:
        for bus in self._buses:
            kp_values: dict[str, float] = {}
            kd_values: dict[str, float] = {}
            for motor_name, motor in bus.motors.items():
                mid = int(motor.id)
                kp_values[motor_name] = float(self.config.position_kp.get(mid, 10.0))
                kd_values[motor_name] = float(self.config.position_kd.get(mid, 0.5))
            if kp_values:
                bus.sync_write("Kp", kp_values)
            if kd_values:
                bus.sync_write("Kd", kd_values)

    def _refresh_states_once(self) -> None:
        all_states = {}
        stamp = time.time()
        for bus in self._buses:
            states = bus.sync_read_all_states()
            for motor_name, state in states.items():
                mid = int(bus.motors[motor_name].id)
                all_states[mid] = {
                    "position": float(state["position"]),
                    "velocity": float(state["velocity"]),
                    "torque": float(state["torque"]),
                    "stamp": stamp,
                }
        with self._state_lock:
            for mid, state in all_states.items():
                self._latest_states[mid] = state

    def _missing_startup_state_motor_ids(self) -> list[int]:
        with self._state_lock:
            return [mid for mid in self._motor_ids if float(self._latest_states[mid].get("stamp", 0.0)) <= 0.0]

    def _wait_for_all_motor_states_on_startup(self) -> None:
        if not self.config.startup_wait_all_motors:
            return
        timeout_s = max(0.0, float(self.config.startup_wait_all_motors_timeout_s))
        poll_s = max(0.001, float(self.config.startup_wait_all_motors_poll_s))
        t0 = time.monotonic()
        missing = self._missing_startup_state_motor_ids()
        if not missing:
            return
        logger.warning("Waiting for startup state from motors: %s", ",".join(str(mid) for mid in missing))
        while missing:
            elapsed_s = time.monotonic() - t0
            if elapsed_s >= timeout_s:
                missing_txt = ", ".join(str(mid) for mid in missing)
                raise RuntimeError(
                    f"Startup aborted: missing initial state from motors [{missing_txt}] "
                    f"after {elapsed_s:.2f}s"
                )
            time.sleep(min(poll_s, max(0.0, timeout_s - elapsed_s)))
            self._refresh_states_once()
            missing = self._missing_startup_state_motor_ids()

    def _run_control_loop(self) -> None:
        period = 1.0 / max(1.0, float(self.config.control_hz))
        next_tick = time.perf_counter()
        stats_t0 = next_tick
        stats_iters = 0
        stats_overruns = 0
        stats_refresh_s = 0.0
        stats_checks_s = 0.0
        stats_send_s = 0.0
        while not self._stop_event.is_set():
            refresh_t = 0.0
            checks_t = 0.0
            send_t = 0.0
            try:
                t0 = time.perf_counter()
                self._refresh_states_once()
                t1 = time.perf_counter()
                refresh_t = t1 - t0

                if not self._startup_wrap_checked:
                    self._startup_wrap_checked = self._auto_correct_startup_wrap()
                t2 = time.perf_counter()
                self._check_observed_state_safety()
                self._check_ankle_configuration_guard()
                t3 = time.perf_counter()
                checks_t = (t2 - t1) + (t3 - t2)
                if self._control_enabled and not self._is_estop():
                    t4 = time.perf_counter()
                    self._send_desired_positions_once()
                    t5 = time.perf_counter()
                    send_t = t5 - t4
            except Exception as e:
                logger.warning("Control loop iteration failed: %s", e)

            stats_iters += 1
            stats_refresh_s += refresh_t
            stats_checks_s += checks_t
            stats_send_s += send_t
            next_tick += period
            sleep_s = next_tick - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                stats_overruns += 1
                next_tick = time.perf_counter()

            now = time.perf_counter()
            if now - stats_t0 >= 1.0:
                elapsed = now - stats_t0
                achieved_hz = float(stats_iters) / elapsed if elapsed > 1e-9 else 0.0
                target_hz = float(self.config.control_hz)
                avg_refresh_ms = (stats_refresh_s / max(1, stats_iters)) * 1000.0
                avg_checks_ms = (stats_checks_s / max(1, stats_iters)) * 1000.0
                avg_send_ms = (stats_send_s / max(1, stats_iters)) * 1000.0
                overrun_pct = 100.0 * float(stats_overruns) / max(1, stats_iters)
                if achieved_hz < 0.95 * target_hz:
                    logger.warning(
                        "Control loop below target: target=%.1fHz achieved=%.1fHz "
                        "(avg ms: refresh=%.2f checks=%.2f send=%.2f, overruns=%.1f%%)",
                        target_hz,
                        achieved_hz,
                        avg_refresh_ms,
                        avg_checks_ms,
                        avg_send_ms,
                        overrun_pct,
                    )
                else:
                    logger.debug(
                        "Control loop: target=%.1fHz achieved=%.1fHz "
                        "(avg ms: refresh=%.2f checks=%.2f send=%.2f, overruns=%.1f%%)",
                        target_hz,
                        achieved_hz,
                        avg_refresh_ms,
                        avg_checks_ms,
                        avg_send_ms,
                        overrun_pct,
                    )
                stats_t0 = now
                stats_iters = 0
                stats_overruns = 0
                stats_refresh_s = 0.0
                stats_checks_s = 0.0
                stats_send_s = 0.0

    def _send_desired_positions_once(self) -> None:
        with self._action_lock:
            desired = dict(self._desired_raw_positions)
        with self._state_lock:
            snapshot = {mid: st for mid, st in self._latest_states.items()}

        bus_batches: list[tuple[RobstrideMotorsBus, dict[str, tuple[float, float, float, float, float]]]] = []
        for bus in self._buses:
            commands = {}
            for motor_name, motor in bus.motors.items():
                mid = int(motor.id)
                st = snapshot[mid]
                if float(st.get("stamp", 0.0)) <= 0.0:
                    now = time.time()
                    if now - self._last_jump_warn_s[mid] > 0.5:
                        logger.warning("m%d: no valid state yet, skipping command send", mid)
                        self._last_jump_warn_s[mid] = now
                    continue

                cur_raw = float(st["position"])
                req_raw = float(desired[mid])
                tgt_raw = self._resolve_raw_target_near_current(mid, req_raw, cur_raw)
                if not self._raw_command_in_limits(mid, tgt_raw, cur_raw):
                    now = time.time()
                    if now - self._last_jump_warn_s[mid] > 0.5:
                        lo, hi = self._raw_limits_near_ref(mid, self._joint_limits_deg.get(mid, (-float("inf"), float("inf"))), cur_raw)
                        logger.warning(
                            "m%d: command out of limits, req_raw=%.1f, tgt_raw=%.1f, allowed=(%.1f, %.1f), skipping send",
                            mid,
                            req_raw,
                            tgt_raw,
                            lo + float(self.config.command_margin_deg),
                            hi - float(self.config.command_margin_deg),
                        )
                        self._last_jump_warn_s[mid] = now
                    continue

                max_delta = float(self.config.max_command_delta_deg)
                delta = tgt_raw - cur_raw
                if abs(delta) > max_delta:
                    now = time.time()
                    if now - self._last_jump_warn_s[mid] > 0.5:
                        logger.warning(
                            "m%d: command jump too large (%.1f deg > %.1f deg), cur_raw=%.1f, req_raw=%.1f, tgt_raw=%.1f, skipping send",
                            mid,
                            abs(delta),
                            max_delta,
                            cur_raw,
                            req_raw,
                            tgt_raw,
                        )
                        self._last_jump_warn_s[mid] = now
                    continue
                commands[motor_name] = (
                    float(self.config.position_kp.get(mid, 10.0)),
                    float(self.config.position_kd.get(mid, 0.5)),
                    float(tgt_raw),
                    0.0,
                    0.0,
                )
            if commands:
                bus_batches.append((bus, commands))

        if not bus_batches:
            return

        futures = [self._tx_executor.submit(bus._mit_control_batch, commands) for bus, commands in bus_batches]
        for future in futures:
            try:
                future.result()
            except Exception as e:
                logger.warning("MIT batch send failed: %s", e)

    def _raw_to_cal(self, motor_id: int, raw_deg: float) -> float:
        return float(self._motor_sign[motor_id] * raw_deg + self._motor_offset_deg[motor_id])

    def _cal_to_raw(self, motor_id: int, cal_deg: float) -> float:
        sign = float(self._motor_sign[motor_id])
        if abs(sign) < 1e-9:
            raise ValueError(f"Invalid sign for motor {motor_id}: {sign}")
        return float((cal_deg - self._motor_offset_deg[motor_id]) / sign)

    def _joint_from_cal_ankle(self, a1: float, a2: float, side: str) -> tuple[float, float]:
        cfg = self._ankle_left_cfg if side == "left" else self._ankle_right_cfg
        sign_p = float(cfg["pitch"]["sign"])
        off_p = float(cfg["pitch"]["offset_deg"])
        sign_r = float(cfg["roll"]["sign"])
        off_r = float(cfg["roll"]["offset_deg"])
        pitch = sign_p * ((a1 - a2) / 2.0) + off_p
        roll = sign_r * ((a1 + a2) / 2.0) + off_r
        return float(pitch), float(roll)

    def _cal_ankle_from_joint(self, pitch: float, roll: float, side: str) -> tuple[float, float]:
        cfg = self._ankle_left_cfg if side == "left" else self._ankle_right_cfg
        sign_p = float(cfg["pitch"]["sign"])
        off_p = float(cfg["pitch"]["offset_deg"])
        sign_r = float(cfg["roll"]["sign"])
        off_r = float(cfg["roll"]["offset_deg"])
        p = (pitch - off_p) / sign_p
        r = (roll - off_r) / sign_r
        return float(p + r), float(r - p)

    def _joint_observation_from_raw_states(
        self, raw_pos: dict[int, float], raw_vel: dict[int, float], raw_tau: dict[int, float]
    ) -> dict[str, float]:
        cal_pos = {mid: self._raw_to_cal(mid, raw_pos[mid]) for mid in self._motor_ids}
        cal_vel = {mid: self._motor_sign[mid] * raw_vel[mid] for mid in self._motor_ids}
        cal_tau = {
            mid: (raw_tau[mid] / self._motor_sign[mid]) if abs(self._motor_sign[mid]) > 1e-9 else 0.0
            for mid in self._motor_ids
        }

        out: dict[str, float] = {}
        for joint, mid in self._joint_to_motor_direct.items():
            out[f"{joint}.pos"] = float(cal_pos[mid])
            out[f"{joint}.vel"] = float(cal_vel[mid])
            out[f"{joint}.torque"] = float(cal_tau[mid])

        l_pitch, l_roll = self._joint_from_cal_ankle(cal_pos[5], cal_pos[6], "left")
        l_pitch_vel, l_roll_vel = self._joint_from_cal_ankle(cal_vel[5], cal_vel[6], "left")
        out["left_ankle_pitch.pos"] = l_pitch
        out["left_ankle_roll.pos"] = l_roll
        out["left_ankle_pitch.vel"] = l_pitch_vel
        out["left_ankle_roll.vel"] = l_roll_vel
        out["left_ankle_pitch.torque"] = float((cal_tau[5] - cal_tau[6]) / float(self._ankle_left_cfg["pitch"]["sign"]))
        out["left_ankle_roll.torque"] = float((cal_tau[5] + cal_tau[6]) / float(self._ankle_left_cfg["roll"]["sign"]))

        r_pitch, r_roll = self._joint_from_cal_ankle(cal_pos[11], cal_pos[12], "right")
        r_pitch_vel, r_roll_vel = self._joint_from_cal_ankle(cal_vel[11], cal_vel[12], "right")
        out["right_ankle_pitch.pos"] = r_pitch
        out["right_ankle_roll.pos"] = r_roll
        out["right_ankle_pitch.vel"] = r_pitch_vel
        out["right_ankle_roll.vel"] = r_roll_vel
        out["right_ankle_pitch.torque"] = float(
            (cal_tau[11] - cal_tau[12]) / float(self._ankle_right_cfg["pitch"]["sign"])
        )
        out["right_ankle_roll.torque"] = float((cal_tau[11] + cal_tau[12]) / float(self._ankle_right_cfg["roll"]["sign"]))
        return out

    def _joint_targets_from_raw_targets(self, raw_targets: dict[int, float]) -> dict[str, float]:
        raw_vel = {mid: 0.0 for mid in self._motor_ids}
        raw_tau = {mid: 0.0 for mid in self._motor_ids}
        obs = self._joint_observation_from_raw_states(raw_targets, raw_vel, raw_tau)
        return {joint: float(obs[f"{joint}.pos"]) for joint in JOINT_ORDER}

    def _raw_targets_from_joint_targets(self, joint_targets_deg: dict[str, float]) -> dict[int, float]:
        cal_targets: dict[int, float] = {}
        for joint, mid in self._joint_to_motor_direct.items():
            cal_targets[mid] = float(joint_targets_deg[joint])

        l_a1, l_a2 = self._cal_ankle_from_joint(
            float(joint_targets_deg["left_ankle_pitch"]),
            float(joint_targets_deg["left_ankle_roll"]),
            "left",
        )
        cal_targets[5], cal_targets[6] = l_a1, l_a2

        r_a1, r_a2 = self._cal_ankle_from_joint(
            float(joint_targets_deg["right_ankle_pitch"]),
            float(joint_targets_deg["right_ankle_roll"]),
            "right",
        )
        cal_targets[11], cal_targets[12] = r_a1, r_a2

        return {mid: self._cal_to_raw(mid, cal_targets[mid]) for mid in self._motor_ids}

    def _clamp_raw_target(self, motor_id: int, target_raw: float, ref_raw_deg: float | None = None) -> float:
        if motor_id not in self._joint_limits_deg:
            return float(target_raw)
        lim = self._joint_limits_deg[motor_id]
        if ref_raw_deg is None:
            lo, hi = float(lim[0]), float(lim[1])
        else:
            lo, hi = self._raw_limits_near_ref(int(motor_id), lim, float(ref_raw_deg))
        lo_safe = lo + float(self.config.command_margin_deg)
        hi_safe = hi - float(self.config.command_margin_deg)
        if lo_safe > hi_safe:
            lo_safe, hi_safe = lo, hi
        return max(lo_safe, min(hi_safe, float(target_raw)))

    def _raw_limits_near_ref(
        self,
        motor_id: int,
        lim_raw: tuple[float, float],
        ref_raw_deg: float,
    ) -> tuple[float, float]:
        lo, hi = float(lim_raw[0]), float(lim_raw[1])
        ref = float(ref_raw_deg)
        best = (lo, hi)
        best_dist = float("inf")
        for k in range(-4, 5):
            cand_lo = lo + 360.0 * k
            cand_hi = hi + 360.0 * k
            center = 0.5 * (cand_lo + cand_hi)
            dist = abs(center - ref)
            if dist < best_dist:
                best = (cand_lo, cand_hi)
                best_dist = dist
        return best

    def _resolve_raw_target_near_current(self, motor_id: int, raw_target_deg: float, raw_current_deg: float) -> float:
        mtype_name = self._motor_type_by_id.get(int(motor_id), "o0").upper()
        mtype = getattr(MotorType, mtype_name, MotorType.O0)
        pmax_rad, _, _ = MOTOR_LIMIT_PARAMS[mtype]
        pmax_deg = float(math.degrees(float(pmax_rad)))
        cur = float(raw_current_deg)
        tgt = float(raw_target_deg)
        best = tgt
        best_dist = abs(tgt - cur)
        found_in_range = (-pmax_deg <= tgt <= pmax_deg)
        for k in range(-4, 5):
            cand = tgt + 360.0 * k
            dist = abs(cand - cur)
            in_range = (-pmax_deg <= cand <= pmax_deg)
            if found_in_range:
                if in_range and dist < best_dist:
                    best = cand
                    best_dist = dist
            else:
                if in_range:
                    best = cand
                    best_dist = dist
                    found_in_range = True
                elif dist < best_dist:
                    best = cand
                    best_dist = dist
        return float(best)

    def _raw_command_in_limits(self, motor_id: int, pos_raw_deg: float, ref_raw_deg: float) -> bool:
        lim_raw = self._joint_limits_deg.get(int(motor_id))
        if lim_raw is None:
            return True
        lo, hi = self._raw_limits_near_ref(int(motor_id), lim_raw, float(ref_raw_deg))
        lo_safe = lo + float(self.config.command_margin_deg)
        hi_safe = hi - float(self.config.command_margin_deg)
        if lo_safe > hi_safe:
            lo_safe, hi_safe = lo, hi
        x = float(pos_raw_deg)
        return lo_safe <= x <= hi_safe

    def _passes_action_update_jump_guard(self, target_raw: dict[int, float]) -> bool:
        with self._state_lock:
            snapshot = {mid: st for mid, st in self._latest_states.items()}
        max_delta = float(self.config.max_command_delta_deg)
        for mid in self._motor_ids:
            if float(snapshot[mid].get("stamp", 0.0)) <= 0.0:
                continue
            cur_raw = float(snapshot[mid]["position"])
            tgt_raw = self._resolve_raw_target_near_current(mid, float(target_raw[mid]), cur_raw)
            if not self._raw_command_in_limits(mid, tgt_raw, cur_raw):
                return False
            if abs(tgt_raw - cur_raw) > max_delta:
                return False
        return True

    def _action_update_guard_failures(self, target_raw: dict[int, float]) -> list[str]:
        with self._state_lock:
            snapshot = {mid: st for mid, st in self._latest_states.items()}
        max_delta = float(self.config.max_command_delta_deg)
        failures: list[str] = []
        for mid in self._motor_ids:
            st = snapshot[mid]
            if float(st.get("stamp", 0.0)) <= 0.0:
                continue
            cur_raw = float(st["position"])
            req_raw = float(target_raw[mid])
            tgt_raw = self._resolve_raw_target_near_current(mid, req_raw, cur_raw)
            if not self._raw_command_in_limits(mid, tgt_raw, cur_raw):
                lim_raw = self._joint_limits_deg.get(int(mid), (-float("inf"), float("inf")))
                lo, hi = self._raw_limits_near_ref(int(mid), lim_raw, cur_raw)
                lo_safe = lo + float(self.config.command_margin_deg)
                hi_safe = hi - float(self.config.command_margin_deg)
                if lo_safe > hi_safe:
                    lo_safe, hi_safe = lo, hi
                failures.append(
                    f"m{mid} limits: cur={cur_raw:.2f} req={req_raw:.2f} tgt={tgt_raw:.2f} "
                    f"allowed=[{lo_safe:.2f},{hi_safe:.2f}]"
                )
                continue
            delta = abs(tgt_raw - cur_raw)
            if delta > max_delta:
                failures.append(
                    f"m{mid} jump: cur={cur_raw:.2f} req={req_raw:.2f} tgt={tgt_raw:.2f} "
                    f"delta={delta:.2f}>{max_delta:.2f}"
                )
        return failures

    def _raw_to_cal_near_interval(self, motor_id: int, raw_deg: float, lo: float, hi: float) -> float:
        base = self._raw_to_cal(motor_id, raw_deg)
        step = 360.0 * float(self._motor_sign[int(motor_id)])
        center = 0.5 * (float(lo) + float(hi))
        best = base
        best_dist = float("inf")
        best_center = float("inf")
        for k in range(-4, 5):
            cand = base + step * k
            if cand < lo:
                dist = lo - cand
            elif cand > hi:
                dist = cand - hi
            else:
                dist = 0.0
            cdist = abs(cand - center)
            if (dist < best_dist) or (dist == best_dist and cdist < best_center):
                best = cand
                best_dist = dist
                best_center = cdist
        return float(best)

    def _state_in_bounds(self, motor_id: int, pos_raw_deg: float) -> bool:
        lim_raw = self._joint_limits_deg.get(int(motor_id))
        if lim_raw is None:
            return True
        lo_raw, hi_raw = float(lim_raw[0]), float(lim_raw[1])
        lo_cal = self._raw_to_cal(int(motor_id), lo_raw)
        hi_cal = self._raw_to_cal(int(motor_id), hi_raw)
        lo, hi = min(lo_cal, hi_cal), max(lo_cal, hi_cal)
        value = self._raw_to_cal_near_interval(int(motor_id), float(pos_raw_deg), lo, hi)
        margin = float(self.config.state_margin_deg)
        return (lo - margin) <= value <= (hi + margin)

    def _auto_correct_startup_wrap(self) -> bool:
        with self._state_lock:
            snapshot = {mid: st for mid, st in self._latest_states.items()}
        if not any(float(st.get("stamp", 0.0)) > 0.0 for st in snapshot.values()):
            return False
        updates: list[tuple[int, float]] = []
        for mid, st in snapshot.items():
            if float(st.get("stamp", 0.0)) <= 0.0:
                continue
            lim = self._joint_limits_deg.get(mid)
            if lim is None:
                continue
            lo, hi = float(lim[0]), float(lim[1])
            cur_cal = self._raw_to_cal(mid, float(st["position"]))
            center = 0.5 * (lo + hi)

            def _dist_interval(x: float, a: float, b: float) -> float:
                if x < a:
                    return a - x
                if x > b:
                    return x - b
                return 0.0

            best_k = 0
            best_dist = _dist_interval(cur_cal, lo, hi)
            best_center = abs(cur_cal - center)
            for k in range(-3, 4):
                cand = cur_cal + 360.0 * k
                cand_dist = _dist_interval(cand, lo, hi)
                cand_center = abs(cand - center)
                if (cand_dist < best_dist) or (cand_dist == best_dist and cand_center < best_center):
                    best_k = k
                    best_dist = cand_dist
                    best_center = cand_center
            if best_k != 0:
                delta = 360.0 * best_k
                self._motor_offset_deg[mid] += delta
                if self.config.startup_wrap_shift_limits:
                    self._joint_limits_deg[mid] = (lo + delta, hi + delta)
                updates.append((mid, delta))
        if updates:
            upd_txt = ", ".join([f"m{mid}:{delta:+.1f}deg" for mid, delta in updates])
            logger.info("startup-wrap-correction applied -> %s", upd_txt)
        return True

    def _is_estop(self) -> bool:
        with self._safety_lock:
            return bool(self._estop)

    def get_estop_reason(self) -> str:
        with self._safety_lock:
            return str(self._estop_reason)

    def clear_estop(self) -> None:
        with self._safety_lock:
            self._estop = False
            self._estop_reason = ""

    def _trigger_estop(self, reason: str) -> None:
        should_disable_torque = False
        with self._safety_lock:
            if not self._estop:
                self._estop = True
                self._estop_reason = str(reason)
                should_disable_torque = True
        if should_disable_torque:
            logger.error("E-STOP triggered: %s", reason)
            for bus in self._buses:
                try:
                    bus.disable_torque()
                except Exception as e:
                    logger.warning("Failed to disable torque after E-STOP: %s", e)

    def _check_observed_state_safety(self) -> None:
        with self._state_lock:
            snapshot = {mid: st for mid, st in self._latest_states.items()}
        for mid, st in snapshot.items():
            if float(st.get("stamp", 0.0)) <= 0.0:
                continue
            pos_raw = float(st["position"])
            if not self._state_in_bounds(mid, pos_raw):
                lim = self._joint_limits_deg.get(mid)
                reason = (
                    f"Observed state out of limits on motor {mid}: {pos_raw:.2f} deg, "
                    f"limits={lim}, margin={float(self.config.state_margin_deg):.2f}"
                )
                self._trigger_estop(reason)
                return

    def _check_ankle_configuration_guard(self) -> None:
        if not self.config.ankle_guard_enabled:
            return
        with self._state_lock:
            raw_pos = {mid: float(self._latest_states[mid]["position"]) for mid in self._motor_ids}
        zero = {mid: 0.0 for mid in self._motor_ids}
        joint_obs = self._joint_observation_from_raw_states(raw_pos, zero, zero)
        abs_lim = float(self.config.ankle_guard_abs_deg)
        ankle_keys = [
            "left_ankle_pitch.pos",
            "left_ankle_roll.pos",
            "right_ankle_pitch.pos",
            "right_ankle_roll.pos",
        ]
        for key in ankle_keys:
            v = float(joint_obs.get(key, 0.0))
            if abs(v) > abs_lim:
                reason = (
                    f"Ankle startup guard violated on {key}: {v:.2f} deg "
                    f"(allowed +/-{abs_lim:.2f} deg). Possible flipped configuration."
                )
                self._trigger_estop(reason)
                return

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        with self._state_lock:
            raw_pos = {mid: float(self._latest_states[mid]["position"]) for mid in self._motor_ids}
            raw_vel = {mid: float(self._latest_states[mid]["velocity"]) for mid in self._motor_ids}
            raw_tau = {mid: float(self._latest_states[mid]["torque"]) for mid in self._motor_ids}

        obs = self._joint_observation_from_raw_states(raw_pos, raw_vel, raw_tau)

        with self._imu_lock:
            imu_state = dict(self._imu_state)

        quat = imu_state.get("quaternion_xyzw", (0.0, 0.0, 0.0, 1.0))
        gyro = imu_state.get("gyro_rads", (0.0, 0.0, 0.0))
        if not (isinstance(quat, (list, tuple)) and len(quat) == 4):
            quat = (0.0, 0.0, 0.0, 1.0)
        if not (isinstance(gyro, (list, tuple)) and len(gyro) == 3):
            gyro = (0.0, 0.0, 0.0)

        obs["base.orientation.x"] = float(quat[0])
        obs["base.orientation.y"] = float(quat[1])
        obs["base.orientation.z"] = float(quat[2])
        obs["base.orientation.w"] = float(quat[3])
        obs["base.ang_vel.x"] = float(gyro[0])
        obs["base.ang_vel.y"] = float(gyro[1])
        obs["base.ang_vel.z"] = float(gyro[2])
        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        if self._is_estop():
            raise RuntimeError(f"E-STOP active, refusing action. Reason: {self.get_estop_reason()}")
        self._check_observed_state_safety()
        self._check_ankle_configuration_guard()
        if self._is_estop():
            raise RuntimeError(f"E-STOP active, refusing action. Reason: {self.get_estop_reason()}")

        with self._action_lock:
            current_targets = dict(self._desired_raw_positions)
        joint_targets = self._joint_targets_from_raw_targets(current_targets)

        touched: dict[str, float] = {}
        for key, value in action.items():
            if not key.endswith(".pos"):
                continue
            joint = key[:-4]
            if joint in joint_targets:
                joint_targets[joint] = float(value)
                touched[joint] = float(value)

        desired_raw = self._raw_targets_from_joint_targets(joint_targets)
        if self.config.clamp_raw_targets:
            with self._state_lock:
                snapshot = {mid: st for mid, st in self._latest_states.items()}
            desired_raw = {
                mid: self._clamp_raw_target(
                    mid,
                    val,
                    float(snapshot[mid]["position"]) if float(snapshot[mid].get("stamp", 0.0)) > 0.0 else None,
                )
                for mid, val in desired_raw.items()
            }
        if not self._passes_action_update_jump_guard(desired_raw):
            failures = self._action_update_guard_failures(desired_raw)
            print("no change in internal value")
            if failures:
                print("action rejected by safety guard:")
                for line in failures[:6]:
                    print(f"  - {line}")
                if len(failures) > 6:
                    print(f"  ... and {len(failures) - 6} more")
            with self._action_lock:
                desired_raw = dict(self._desired_raw_positions)

        with self._action_lock:
            self._desired_raw_positions = desired_raw

        effective_joint = self._joint_targets_from_raw_targets(desired_raw)
        return {f"{joint}.pos": float(effective_joint[joint]) for joint in touched}

    @check_if_not_connected
    def disconnect(self) -> None:
        self._stop_event.set()
        if self._control_thread is not None:
            self._control_thread.join(timeout=2.0)
        self._control_thread = None

        if self.config.disable_torque_on_disconnect:
            for bus in self._buses:
                try:
                    bus.disable_torque()
                except Exception as e:
                    logger.warning("Failed to disable torque on disconnect: %s", e)

        for bus in self._buses:
            bus.disconnect(disable_torque=False)

        self._stop_imu_thread()

        self._connected = False
        logger.info("%s disconnected", self)

    def __del__(self) -> None:
        try:
            self._tx_executor.shutdown(wait=False, cancel_futures=True)
        except Exception:
            pass
