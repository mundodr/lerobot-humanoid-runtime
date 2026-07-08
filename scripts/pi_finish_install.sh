#!/usr/bin/env bash
# 在树莓派上完成依赖安装（网络不稳时可配合本地 wheels 目录）
set -euo pipefail

REPO="${REPO:-/home/big/lerobot/lerobot-humanoid-runtime}"
WHEELS="${WHEELS:-/home/big/lerobot/wheels}"
export PATH="${HOME}/.local/bin:${PATH}"
export UV_HTTP_TIMEOUT="${UV_HTTP_TIMEOUT:-600}"
export UV_CONCURRENT_DOWNLOADS="${UV_CONCURRENT_DOWNLOADS:-2}"
# 国内网络可改用清华镜像
export UV_INDEX_URL="${UV_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}"

cd "${REPO}"

if [[ -d "${WHEELS}" ]]; then
  export UV_FIND_LINKS="${WHEELS}"
  echo "[install] using local wheels: ${WHEELS}"
fi

echo "[install] uv sync (policy imu viz gamepad)..."
uv sync --extra policy --extra imu --extra viz --extra gamepad

echo "[verify] import test..."
uv run python -c "
from robot.bipedal_robot import BipedalRobotController
from control.rl_agent import RLAgent
print('IMPORT_OK')
"

echo "[done] ready. example deploy:"
echo "  cd ${REPO}"
echo "  uv run python deploy/run_real_policy_sequential.py --policy-dir control/policy/codex_iteration_6"
