#!/usr/bin/env python3
"""烟测 ZQWL 串口 CAN：初始化双通道并尝试收发一帧。"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="测试 ZQWL 串口 CAN 适配层")
    p.add_argument("--port", type=str, default=None)
    p.add_argument("--channel", type=int, default=0, choices=(0, 1))
    p.add_argument("--motor-id", type=int, default=1, help="MIT 清故障探测目标 ID")
    p.add_argument("--mit-probe", action="store_true", help="发送 MIT 清故障帧 (FF×6+FF+FB)")
    p.add_argument("--listen-s", type=float, default=2.0)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    import can

    from robot.zqwl_serial_bus import find_zqwl_port, open_zqwl_can_buses

    port = args.port or find_zqwl_port()
    if not port:
        print("[error] 未找到 ZQWL 设备")
        return 1
    print(f"[ok] ZQWL port={port}")

    bus0, bus1 = open_zqwl_can_buses(port=port)
    bus = bus0 if int(args.channel) == 0 else bus1
    try:
        if args.mit_probe:
            mid = int(args.motor_id)
            data = bytes([0xFF] * 6 + [0xFF, 0xFB])
            probe = can.Message(arbitration_id=mid, data=data, is_extended_id=False)
            print(f"[tx] MIT clear_fault ch{args.channel} id={mid} data={data.hex()}")
        else:
            can_id = int(getattr(args, "can_id", 0x7FF))
            probe = can.Message(arbitration_id=can_id, data=[0xFF] * 8, is_extended_id=False)
            print(f"[tx] ch{args.channel} id=0x{can_id:x} data=ff*8")
        bus.send(probe)

        deadline = time.perf_counter() + float(args.listen_s)
        count = 0
        while time.perf_counter() < deadline:
            rx = bus.recv(0.05)
            if rx is None:
                continue
            count += 1
            data_hex = " ".join(f"{b:02x}" for b in rx.data)
            print(f"[rx] id=0x{int(rx.arbitration_id):x} data={data_hex}")
        print(f"[done] 收到 {count} 帧（电机未上电时可能为 0）")
        return 0
    finally:
        bus0.shutdown()
        bus1.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
