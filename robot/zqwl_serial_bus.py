"""ZQWL USB-CANFD 串口协议适配（python-can 兼容接口）。

协议与 MIT 帧格式参考 topsun-bot/robstride-motor-config：
- 串口 460800（CDC），非 6Mbps
- RobStride MIT 使用经典 CAN 标准帧（8 字节），非 CANFD
- 配置帧布局与 lingzu/transport/zqwl.py 一致
"""
from __future__ import annotations

import glob
import logging
import os
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

import can
import serial

logger = logging.getLogger(__name__)

# ZQWL 配置帧
_CFG_HEAD = bytes((0x49, 0x3B))
_CFG_TAIL = bytes((0x45, 0x2E))
_CAN_HEAD = 0x5A
_CAN_TAIL = 0xA5
_RW_WRITE = 0x57
_FUNC_CAN_PARAM = 0x42
_FUNC_SYSTEM = 0x44
_BAUD_1000K = 0x0
_HEARTBEAT_BYTE1 = (0xFF, 0xFE)

# 默认 USB 串口波特率（ZQWL UCANFD-200U CDC）
DEFAULT_SERIAL_BAUDRATE = 460_800


def find_zqwl_port() -> Optional[str]:
    """查找 ZQWL USB-CANFD 对应的 ttyACM 设备。"""
    for path in sorted(glob.glob("/dev/ttyACM*")):
        tty = os.path.basename(path)
        for rel in ("device/idVendor", "device/../idVendor"):
            vendor_path = f"/sys/class/tty/{tty}/{rel}"
            try:
                with open(vendor_path, encoding="ascii") as f:
                    if f.read().strip().lower() == "3562":
                        return path
            except OSError:
                continue
    return None


def _config_frame(func_code: int, rw: int, payload16: bytes) -> bytes:
    if len(payload16) != 16:
        raise ValueError("ZQWL 配置帧 payload 必须为 16 字节")
    return _CFG_HEAD + bytes((func_code, rw)) + payload16 + _CFG_TAIL


def _build_set_can_param(channel: int, baud_code: int = _BAUD_1000K) -> bytes:
    payload = bytearray(16)
    payload[0] = int(channel) & 0x03
    payload[2] = baud_code & 0xFF
    return _config_frame(_FUNC_CAN_PARAM, _RW_WRITE, bytes(payload))


def _build_apply_and_start(*, can0_on: bool = True, can1_on: bool = True) -> bytes:
    payload = bytearray(16)
    payload[0] = 0x00
    payload[1] = 0x00
    payload[2] = 0x01 if can0_on else 0x00
    payload[3] = 0x01 if can1_on else 0x00
    return _config_frame(_FUNC_SYSTEM, _RW_WRITE, bytes(payload))


def _encode_channel(channel: int) -> tuple[int, int]:
    ch = int(channel) & 0x07
    byte1_ch_bit = (ch & 0x01) << 7
    byte2_ch_bits = ((ch >> 1) & 0x03) << 3
    return byte1_ch_bit, byte2_ch_bits


def _decode_channel(byte1: int, byte2: int) -> int:
    return ((byte2 >> 3) & 0x03) << 1 | ((byte1 >> 7) & 0x01)


def pack_can_tx(
    channel: int,
    can_id: int,
    data: bytes,
    *,
    extended: bool = False,
    canfd: bool = False,
) -> bytes:
    """打包 ZQWL 串口 CAN 发送帧。MIT 协议应使用 extended=False, canfd=False。"""
    payload = bytes(data)
    dlc = len(payload)
    if dlc > 8 and not canfd:
        raise ValueError("经典 CAN 数据长度不能超过 8 字节")

    ch_b1, ch_b2 = _encode_channel(channel)
    byte1 = ch_b1 | (dlc & 0x7F)
    info2 = ch_b2 | (0x04 if extended else 0x00)

    if extended:
        raw = int(can_id) & 0x1FFFFFFF
        id3 = (raw >> 24) & 0x7F
        if canfd:
            id3 |= 0x80
        id4 = (raw >> 16) & 0xFF
        id5 = (raw >> 8) & 0xFF
        id6 = raw & 0xFF
    else:
        std_id = int(can_id) & 0x7FF
        id3 = 0x80 if canfd else 0x00
        id4 = 0x00
        id5 = (std_id >> 8) & 0x07
        id6 = std_id & 0xFF

    return bytes((_CAN_HEAD, byte1, info2, id3, id4, id5, id6)) + payload + bytes((_CAN_TAIL,))


@dataclass
class _ParsedCanFrame:
    channel: int
    arbitration_id: int
    data: bytes
    is_extended_id: bool

    def to_message(self) -> can.Message:
        return can.Message(
            arbitration_id=int(self.arbitration_id),
            data=bytearray(self.data),
            is_extended_id=bool(self.is_extended_id),
        )


def _unpack_can_rx(frame: bytes) -> _ParsedCanFrame:
    if len(frame) < 7 or frame[0] != _CAN_HEAD or frame[-1] != _CAN_TAIL:
        raise ValueError("invalid ZQWL CAN frame")
    byte1, info2 = frame[1], frame[2]
    if byte1 in _HEARTBEAT_BYTE1:
        raise ValueError("heartbeat")
    dlc = byte1 & 0x7F
    channel = _decode_channel(byte1, info2)
    extended = bool(info2 & 0x04)
    payload = frame[7:-1]
    if extended:
        can_id = ((frame[3] & 0x7F) << 24) | (frame[4] << 16) | (frame[5] << 8) | frame[6]
    else:
        can_id = ((frame[5] & 0x07) << 8) | frame[6]
    data = payload[:dlc].ljust(8, b"\x00")
    return _ParsedCanFrame(
        channel=channel,
        arbitration_id=can_id,
        data=data,
        is_extended_id=extended,
    )


class ZqwlSerialAdapter:
    """单串口双通道 ZQWL 适配器，供两个 ZqwlCanBus 共享。"""

    def __init__(
        self,
        port: str,
        *,
        serial_baudrate: int = DEFAULT_SERIAL_BAUDRATE,
        persist_config: bool = False,
        recv_queue_size: int = 4096,
    ) -> None:
        self.port = str(port)
        self.serial_baudrate = int(serial_baudrate)
        self.persist_config = bool(persist_config)
        self._recv_queue_size = max(128, int(recv_queue_size))

        self._ser: Optional[serial.Serial] = None
        self._rx_queues: dict[int, Deque[can.Message]] = {0: deque(), 1: deque()}
        self._rx_cond = threading.Condition()
        self._write_lock = threading.Lock()
        self._reader_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._users = 0
        self._users_lock = threading.Lock()
        self._opened = False

    def acquire(self) -> None:
        with self._users_lock:
            self._users += 1
            if not self._opened:
                self._open_and_configure()
                self._opened = True

    def release(self) -> None:
        with self._users_lock:
            self._users = max(0, self._users - 1)
            if self._users == 0 and self._opened:
                self._close()
                self._opened = False

    def _open_and_configure(self) -> None:
        self._ser = serial.Serial(
            self.port,
            baudrate=self.serial_baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.02,
            write_timeout=0.1,
        )
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        time.sleep(0.1)

        self._write_cfg(_build_set_can_param(0, _BAUD_1000K))
        self._write_cfg(_build_set_can_param(1, _BAUD_1000K))
        apply = bytearray(16)
        apply[0] = 0x01 if self.persist_config else 0x00
        apply[1] = 0x00
        apply[2] = 0x01
        apply[3] = 0x01
        self._write_cfg(_config_frame(_FUNC_SYSTEM, _RW_WRITE, bytes(apply)))
        time.sleep(0.15)

        self._stop.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="zqwl-serial-rx",
            daemon=True,
        )
        self._reader_thread.start()
        logger.info(
            "ZQWL 已初始化: port=%s serial_baud=%s can=1000kbps classic",
            self.port,
            self.serial_baudrate,
        )

    def _write_cfg(self, frame: bytes) -> None:
        assert self._ser is not None
        with self._write_lock:
            self._ser.write(frame)
            self._ser.flush()

    def _close(self) -> None:
        self._stop.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
            self._reader_thread = None
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        with self._rx_cond:
            for q in self._rx_queues.values():
                q.clear()

    def send_can(
        self,
        channel: int,
        arbitration_id: int,
        data: bytes,
        *,
        is_extended_id: bool = False,
        canfd: bool = False,
    ) -> None:
        if self._ser is None:
            raise RuntimeError("ZQWL 串口未打开")
        if is_extended_id:
            raise NotImplementedError("ZQWL 适配器当前仅支持 MIT 标准帧")
        frame = pack_can_tx(
            int(channel),
            int(arbitration_id),
            bytes(data),
            extended=False,
            canfd=bool(canfd),
        )
        with self._write_lock:
            self._ser.write(frame)
            self._ser.flush()

    def recv(self, channel: int, timeout: Optional[float]) -> Optional[can.Message]:
        deadline = None if timeout is None else time.perf_counter() + max(0.0, float(timeout))
        ch = int(channel)
        with self._rx_cond:
            while True:
                q = self._rx_queues.get(ch)
                if q:
                    return q.popleft()
                if timeout is not None and time.perf_counter() >= deadline:
                    return None
                remaining = None if deadline is None else max(0.0, deadline - time.perf_counter())
                self._rx_cond.wait(timeout=remaining if remaining is not None else 0.05)

    def _enqueue_rx(self, channel: int, msg: can.Message) -> None:
        ch = int(channel)
        with self._rx_cond:
            q = self._rx_queues.setdefault(ch, deque())
            q.append(msg)
            while len(q) > self._recv_queue_size:
                q.popleft()
            self._rx_cond.notify_all()

    def _reader_loop(self) -> None:
        assert self._ser is not None
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(512)
            except Exception as exc:
                logger.warning("ZQWL 串口读取失败: %s", exc)
                time.sleep(0.01)
                continue
            if chunk:
                buf.extend(chunk)
            self._drain_buffer(buf)

    def _drain_buffer(self, buf: bytearray) -> None:
        while True:
            if len(buf) >= 2 and buf[0] == 0x49 and buf[1] == 0x3B:
                if len(buf) < 22:
                    return
                del buf[:22]
                continue

            idx = buf.find(bytes((_CAN_HEAD,)))
            if idx < 0:
                if len(buf) > 4096:
                    buf.clear()
                return
            if idx > 0:
                del buf[:idx]

            if len(buf) >= 2 and buf[1] in _HEARTBEAT_BYTE1:
                want = 17 if buf[1] == 0xFF else 32
                if len(buf) < want:
                    return
                del buf[:want]
                continue

            tail = buf.find(bytes((_CAN_TAIL,)), 3)
            if tail < 0:
                return

            raw = bytes(buf[: tail + 1])
            del buf[: tail + 1]
            try:
                parsed = _unpack_can_rx(raw)
            except ValueError:
                continue
            self._enqueue_rx(parsed.channel, parsed.to_message())


class ZqwlCanBus:
    """单路逻辑 CAN 总线（can0=通道0，can1=通道1）。"""

    def __init__(self, adapter: ZqwlSerialAdapter, channel: int) -> None:
        self._adapter = adapter
        self._channel = int(channel)
        self._adapter.acquire()

    def send(self, msg: can.Message) -> None:
        # RobStride MIT：经典 CAN 标准帧，仲裁域 1Mbps
        self._adapter.send_can(
            self._channel,
            int(msg.arbitration_id),
            bytes(msg.data),
            is_extended_id=bool(msg.is_extended_id),
            canfd=False,
        )

    def recv(self, timeout: Optional[float] = None) -> Optional[can.Message]:
        return self._adapter.recv(self._channel, timeout)

    def shutdown(self) -> None:
        self._adapter.release()


def open_zqwl_can_buses(
    port: Optional[str] = None,
    *,
    serial_baudrate: int = DEFAULT_SERIAL_BAUDRATE,
    persist_config: bool = False,
    arb_bitrate: int = 1_000_000,
    data_bitrate: int = 5_000_000,
) -> tuple[ZqwlCanBus, ZqwlCanBus]:
    """打开 ZQWL 双通道总线，返回 (can0, can1) 兼容对象。

    arb_bitrate/data_bitrate 保留参数兼容旧调用；MIT 模式下固定 1Mbps 经典 CAN。
    """
    _ = (arb_bitrate, data_bitrate)
    resolved = port or find_zqwl_port()
    if not resolved:
        raise RuntimeError("未找到 ZQWL 设备（/dev/ttyACM* 且 idVendor=3562）")
    adapter = ZqwlSerialAdapter(
        resolved,
        serial_baudrate=serial_baudrate,
        persist_config=persist_config,
    )
    return ZqwlCanBus(adapter, 0), ZqwlCanBus(adapter, 1)
