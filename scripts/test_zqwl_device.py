#!/usr/bin/env python3
"""用 bc-stark-sdk 检测 ZQWL USB-CANFD（串口模式，非 SocketCAN）。"""
from __future__ import annotations

import sys


def main() -> int:
    try:
        from bc_stark_sdk import main_mod as sdk
    except ImportError:
        print("[error] 未安装 bc-stark-sdk。请执行: uv pip install bc-stark-sdk")
        return 1

    devices = sdk.list_zqwl_devices()
    if not devices:
        print("[error] 未发现 ZQWL 设备。请检查 USB 连接与权限（dialout 组）。")
        return 2

    for i, dev in enumerate(devices):
        print(f"[ok] ZQWL #{i}: port={dev.port_name}")

    port = devices[0].port_name
    arb_bitrate = 1_000_000
    data_bitrate = 5_000_000
    print(f"[test] init_zqwl_canfd({port!r}, {arb_bitrate}, {data_bitrate}) ...")
    try:
        sdk.init_zqwl_canfd(port, arb_bitrate, data_bitrate)
    except Exception as exc:
        print(f"[error] 初始化失败: {exc}")
        return 3

    print("[ok] ZQWL CANFD 串口初始化成功（注意：这不会创建 can0/can1 网卡）")
    print("      runtime 真机部署仍需 SocketCAN 或 ZQWL 总线适配层。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
