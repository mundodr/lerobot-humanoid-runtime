#!/usr/bin/env python

from __future__ import annotations

import time
from collections import deque
from types import SimpleNamespace
from typing import TYPE_CHECKING

from lerobot.motors import Motor
from lerobot.motors.robstride.tables import (
    CAN_CMD_CLEAR_FAULT,
    CAN_CMD_DISABLE,
    CAN_CMD_ENABLE,
    CAN_CMD_SET_ZERO,
    MOTOR_LIMIT_PARAMS,
    MotorType,
)
from lerobot.utils.import_utils import _can_available

if TYPE_CHECKING or _can_available:
    import can
else:
    # Minimal fallback so the mock can still be instantiated when python-can
    # is absent. Only fields used by RobstrideMotorsBus are provided.
    class _MockMessage:
        def __init__(self, *, arbitration_id: int, data: bytes, is_extended_id: bool = False):
            self.arbitration_id = int(arbitration_id)
            self.data = bytearray(data)
            self.is_extended_id = bool(is_extended_id)

    can = SimpleNamespace(Message=_MockMessage)


def _float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    x = max(x_min, min(x_max, float(x)))
    span = x_max - x_min
    if span <= 0:
        return 0
    return int(((x - x_min) / span) * ((1 << bits) - 1))


def _uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    if span <= 0:
        return x_min
    return (float(x) / ((1 << bits) - 1)) * span + x_min


class RobstrideMockBus:
    """
    Minimal python-can compatible mock bus for Robstride MIT mode.
    - Command frames (FF... + cmd) echo current state.
    - MIT frames update internal state immediately and return feedback.
    """

    def __init__(
        self,
        *,
        motors: dict[str, Motor],
        default_temp_c: float = 30.0,
        send_sleep_s: float = 0.0,
        initial_position_deg_by_motor_id: dict[int, float] | None = None,
    ):
        self._rx_queue: deque[can.Message] = deque()
        self._default_temp_c = float(default_temp_c)
        self._send_sleep_s = float(max(0.0, send_sleep_s))
        initial_position_deg_by_motor_id = initial_position_deg_by_motor_id or {}

        self._motors_by_id: dict[int, Motor] = {int(m.id): m for m in motors.values()}
        self._limits_by_id: dict[int, tuple[float, float, float]] = {}
        self._response_id_by_motor_id: dict[int, int] = {}
        self._state_by_motor_id: dict[int, dict[str, float]] = {}
        for m in motors.values():
            mid = int(m.id)
            mtype_str = (m.motor_type_str or "o0").upper()
            mtype = getattr(MotorType, mtype_str, MotorType.O0)
            self._limits_by_id[mid] = MOTOR_LIMIT_PARAMS[mtype]
            self._response_id_by_motor_id[mid] = int(m.recv_id if m.recv_id is not None else m.id)
            self._state_by_motor_id[mid] = {
                "position_deg": float(initial_position_deg_by_motor_id.get(mid, 0.0)),
                "velocity_deg_s": 0.0,
                "torque_nm": 0.0,
                "temp_c": self._default_temp_c,
            }

    def send(self, msg: can.Message) -> None:
        if self._send_sleep_s > 0.0:
            time.sleep(self._send_sleep_s)

        data = bytes(msg.data)
        if len(data) < 8:
            return

        motor_id = int(msg.arbitration_id)
        if motor_id not in self._state_by_motor_id:
            return

        # Command frames: FF FF FF FF FF FF FF CMD
        is_cmd_ff = len(data) == 8 and all(b == 0xFF for b in data[:7])
        if is_cmd_ff:
            cmd = int(data[7])
            if cmd in {int(CAN_CMD_CLEAR_FAULT), int(CAN_CMD_ENABLE), int(CAN_CMD_DISABLE), int(CAN_CMD_SET_ZERO)}:
                if cmd == int(CAN_CMD_SET_ZERO):
                    self._state_by_motor_id[motor_id]["position_deg"] = 0.0
                self._enqueue_feedback(motor_id)
            return

        # MIT frame decode.
        pmax, vmax, tmax = self._limits_by_id[motor_id]
        q_uint = (int(data[0]) << 8) | int(data[1])
        dq_uint = (int(data[2]) << 4) | (int(data[3]) >> 4)
        tau_uint = ((int(data[6]) & 0x0F) << 8) | int(data[7])

        q_rad = _uint_to_float(q_uint, -pmax, pmax, 16)
        dq_rad = _uint_to_float(dq_uint, -vmax, vmax, 12)
        tau_nm = _uint_to_float(tau_uint, -tmax, tmax, 12)

        st = self._state_by_motor_id[motor_id]
        st["position_deg"] = float(q_rad * 180.0 / 3.141592653589793)
        st["velocity_deg_s"] = float(dq_rad * 180.0 / 3.141592653589793)
        st["torque_nm"] = float(tau_nm)
        self._enqueue_feedback(motor_id)

    def recv(self, timeout: float | None = None) -> can.Message | None:
        if timeout is None:
            while True:
                if self._rx_queue:
                    return self._rx_queue.popleft()
                time.sleep(0.0001)

        timeout_s = float(timeout)
        if timeout_s <= 0.0:
            return self._rx_queue.popleft() if self._rx_queue else None

        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            if self._rx_queue:
                return self._rx_queue.popleft()
            time.sleep(0.0001)
        return None

    def shutdown(self) -> None:
        self._rx_queue.clear()

    def _enqueue_feedback(self, motor_id: int) -> None:
        pmax, vmax, tmax = self._limits_by_id[motor_id]
        st = self._state_by_motor_id[motor_id]
        response_id = self._response_id_by_motor_id[motor_id]

        q_rad = float(st["position_deg"]) * 3.141592653589793 / 180.0
        dq_rad = float(st["velocity_deg_s"]) * 3.141592653589793 / 180.0
        tau_nm = float(st["torque_nm"])
        temp_tenths = int(max(0, min(65535, round(float(st["temp_c"]) * 10.0))))

        q_uint = _float_to_uint(q_rad, -pmax, pmax, 16)
        dq_uint = _float_to_uint(dq_rad, -vmax, vmax, 12)
        tau_uint = _float_to_uint(tau_nm, -tmax, tmax, 12)

        payload = [0] * 8
        payload[0] = int(response_id) & 0xFF
        payload[1] = (q_uint >> 8) & 0xFF
        payload[2] = q_uint & 0xFF
        payload[3] = (dq_uint >> 4) & 0xFF
        payload[4] = ((dq_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F)
        payload[5] = tau_uint & 0xFF
        payload[6] = (temp_tenths >> 8) & 0xFF
        payload[7] = temp_tenths & 0xFF

        self._rx_queue.append(
            can.Message(
                arbitration_id=int(motor_id),
                data=bytes(payload),
                is_extended_id=False,
            )
        )
