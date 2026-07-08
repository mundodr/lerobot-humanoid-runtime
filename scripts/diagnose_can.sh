#!/usr/bin/env bash
# 检查树莓派上 CAN 接口是否可用于 lerobot-humanoid-runtime（SocketCAN: can0/can1）
set -euo pipefail

echo "=== CAN 诊断 ==="
echo "内核: $(uname -r)"
echo

echo "--- 网络接口 ---"
ip -br link || true
echo

HAS_CAN0=false
HAS_CAN1=false
ip link show can0 >/dev/null 2>&1 && HAS_CAN0=true
ip link show can1 >/dev/null 2>&1 && HAS_CAN1=true

if $HAS_CAN0 && $HAS_CAN1; then
  echo "[ok] 已发现 SocketCAN 接口 can0、can1"
  echo
  echo "拉起命令（项目默认 CAN-FD 参数）："
  echo "  sudo ip link set can0 down 2>/dev/null || true"
  echo "  sudo ip link set can1 down 2>/dev/null || true"
  echo "  sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on"
  echo "  sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on"
  echo
  echo "验证："
  echo "  ip -details link show can0 can1"
  echo "  candump can0   # 电机上电后应有帧"
  exit 0
fi

echo "[error] 未发现 can0/can1（当前仅有 lo/eth0/wlan0 等普通网卡）"
echo

echo "--- USB 设备 ---"
if command -v lsusb >/dev/null 2>&1; then
  lsusb | grep -iE 'can|3562|pibiger|savvy|zlg|kunhong|peak' || lsusb
else
  echo "lsusb 不可用"
fi
echo

ZQWL=false
SAVVY=false
if lsusb 2>/dev/null | grep -q '3562:0100'; then
  ZQWL=true
fi
if lsusb 2>/dev/null | grep -qiE 'pibiger|savvycan'; then
  SAVVY=true
fi

if $ZQWL; then
  echo ">>> 检测到 ZQWL-CANFD（USB ID 3562:0100）"
  echo "    该设备在 Linux 上通常只出现为 /dev/ttyACM0，不会自动创建 can0/can1。"
  echo "    厂商提供的 ZQWL-CANFD_Drivers.rar 仅含 Windows 驱动（usbser.sys），无法在树莓派上使用。"
  echo
  echo "    本项目 runtime 使用 python-can + SocketCAN，需要网卡名 can0/can1。"
  echo "    项目文档验证过的适配器：SAVVYCANFD 2CH（Pibiger），插上即出现 can0/can1。"
  echo
  echo "    可选方案："
  echo "    A) 更换为 SAVVYCANFD 双路 CAN-FD 适配器（推荐，与文档一致）"
  echo "    B) 向智嵌物联索取 Linux SocketCAN 驱动/SDK（需支持 aarch64 / 树莓派内核）"
    echo "    C) 短期仅验证策略管线（不接电机）："
    echo "       uv run python deploy/run_real_policy_sequential.py \\"
    echo "         --policy-dir control/policy/remote-run-fast-yawsoft \\"
    echo "         --use-mock-bus --no-with-imu --no-with-meshcat --no-with-gamepad \\"
    echo "         --no-pause-between-stages"
    echo
    echo "    D) 使用 runtime 内置 ZQWL 串口适配（已实现）："
    echo "       uv run python deploy/run_real_policy_sequential.py \\"
    echo "         --policy-dir control/policy/remote-run-fast-yawsoft \\"
    echo "         --use-zqwl-bus --no-with-gamepad --no-pause-between-stages"
    echo "       或: bash scripts/start_policy_on_pi.sh"
    exit 2
fi

if $SAVVY; then
  echo ">>> 检测到 Pibiger/SAVVYCANFD 设备，但未出现 can0/can1"
  echo "    请尝试：重新插拔 USB、换口、检查供电与线缆；"
  echo "    若仍无接口，查看 dmesg | tail -50 是否有驱动报错。"
  exit 2
fi

echo ">>> 未识别到已知 CAN 适配器，或未插入 USB CAN 设备。"
echo "    请确认："
echo "    1) SAVVYCANFD 2CH 适配器已插入树莓派 USB"
echo "    2) 电机总线已接线（can0 左腿 ID 1-6，can1 右腿 ID 7-12）"
echo "    3) 适配器供电与 120Ω 终端电阻配置正确"
exit 2
