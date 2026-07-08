#!/usr/bin/env python3
"""扫描 ZQWL 双通道上 MIT 电机 1-12 是否在线。"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def looks_like_status(data: bytes) -> bool:
    if len(data) < 8:
        return False
    temp = ((data[6] << 8) | data[7]) / 10.0
    return -40.0 <= temp <= 150.0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="扫描 MIT 电机在线状态")
    p.add_argument("--port", type=str, default=None)
    p.add_argument("--wait-s", type=float, default=0.15)
    return p.parse_args()


def main() -> int:
    import can
    from robot.zqwl_serial_bus import find_zqwl_port, open_zqwl_can_buses

    args = parse_args()
    port = args.port or find_zqwl_port()
    if not port:
        print("[error] 未找到 ZQWL 设备")
        return 1

    expected_ch = {i: 0 for i in range(1, 7)} | {i: 1 for i in range(7, 13)}
    bus0, bus1 = open_zqwl_can_buses(port=port)
    buses = {0: bus0, 1: bus1}
    online: dict[int, tuple[int, int, str, float]] = {}

    try:
        for mid in range(1, 13):
            for ch in (0, 1):
                bus = buses[ch]
                data = bytes([0xFF] * 6 + [0xFF, 0xFB])
                bus.send(can.Message(arbitration_id=mid, data=data, is_extended_id=False))
                deadline = time.perf_counter() + float(args.wait_s)
                while time.perf_counter() < deadline:
                    rx = bus.recv(0.02)
                    if rx is None:
                        continue
                    raw = bytes(rx.data)
                    if len(raw) >= 8 and int(raw[0]) == mid and looks_like_status(raw):
                        temp = ((raw[6] << 8) | raw[7]) / 10.0
                        online[mid] = (ch, int(rx.arbitration_id), raw.hex(), temp)
                        break
                if mid in online:
                    break

        print(f"port={port}\n")
        print("=== 电机在线状态 ===")
        for mid in range(1, 13):
            exp = expected_ch[mid]
            if mid in online:
                ch, rx_id, hx, temp = online[mid]
                tag = "OK" if ch == exp else f"WARN(在ch{ch},期望ch{exp})"
                print(f"m{mid:2d}: 在线 [{tag}] rx_id=0x{rx_id:x} temp={temp:.1f}C data={hx}")
            else:
                print(f"m{mid:2d}: 离线 (期望 ch{exp})")
        print(f"\n在线: {len(online)}/12")
        return 0 if online else 1
    finally:
        bus0.shutdown()
        bus1.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
