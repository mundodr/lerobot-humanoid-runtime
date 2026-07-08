#!/usr/bin/env bash
# 在树莓派上启动真机策略（SocketCAN 或 ZQWL 串口）
set -euo pipefail

REPO="${REPO:-/home/big/lerobot/lerobot-humanoid-runtime}"
POLICY_DIR="${POLICY_DIR:-control/policy/remote-run-fast-yawsoft}"
ACTION_SCALE="${ACTION_SCALE:-0.25}"
LOG="${LOG:-/tmp/deploy-remote-yawsoft.log}"

export PATH="${HOME}/.local/bin:${PATH}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ZQWL_ARGS=()

has_socketcan() {
  ip link show can0 >/dev/null 2>&1 && ip link show can1 >/dev/null 2>&1
}

has_zqwl_usb() {
  lsusb 2>/dev/null | grep -q '3562:0100'
}

if has_socketcan; then
  for iface in can0 can1; do
    if ! ip link show "$iface" | grep -q "state UP"; then
      echo "[setup] 拉起 $iface ..."
      sudo ip link set "$iface" down 2>/dev/null || true
      sudo ip link set "$iface" up type can bitrate 1000000 dbitrate 5000000 fd on
    fi
    echo "[ok] $iface: $(ip -details link show "$iface" | head -1)"
  done
elif has_zqwl_usb; then
  echo "[setup] 未检测到 can0/can1，使用 ZQWL 串口适配 (--use-zqwl-bus)"
  ZQWL_ARGS=(--use-zqwl-bus)
else
  echo "[error] 未发现 SocketCAN (can0/can1) 或 ZQWL USB-CAN 设备。"
  if [[ -x "$SCRIPT_DIR/diagnose_can.sh" ]]; then
    bash "$SCRIPT_DIR/diagnose_can.sh" || true
  fi
  exit 1
fi

cd "$REPO"
echo "[deploy] policy=$POLICY_DIR action_scale=$ACTION_SCALE log=$LOG"
exec uv run python deploy/run_real_policy_sequential.py \
  --policy-dir "$POLICY_DIR" \
  --no-pause-between-stages \
  --action-scale "$ACTION_SCALE" \
  --no-with-gamepad \
  "${ZQWL_ARGS[@]}" \
  2>&1 | tee "$LOG"
