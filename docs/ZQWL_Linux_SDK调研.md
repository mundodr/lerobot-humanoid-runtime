# ZQWL-CANFD 在树莓派 Linux 上的 SDK 调研

整理日期：2026-07-08  
设备：`USB 3562:0100 ZQWL-CANFD` → `/dev/ttyACM0`

## 1. 结论（先看这个）

| 方式 | 能否在 Pi 上用 | 能否出现 `can0`/`can1` | 说明 |
|------|----------------|------------------------|------|
| 官方「二次开发资料 V1.20」 | ❌ 仅 Windows | ❌ | 只有 `zlgcan.dll` / `ControlCAN.dll` |
| 官方 `ZQWL-CANFD_Drivers.rar` | ❌ 仅 Windows | ❌ | `usbser.sys`，无 Linux |
| **串口协议**（二次开发通讯协议 PDF） | ✅ | ❌ | 走 `/dev/ttyACM0`，需自己写驱动层 |
| **bc-stark-sdk**（含 ZQWL 实现） | ✅ 已实测 | ❌ | `pip install bc-stark-sdk`，aarch64 wheel |
| 周立功 Linux `.so`（zlgcan） | ⚠️ 多为 x86_64 | ❌ | ZQWL 兼容 ZLG API，但 Pi 无官方 arm64 库 |
| SAVVYCANFD（项目推荐） | ✅ | ✅ 原生 SocketCAN | 与当前 runtime 直接兼容 |

**当前 `lerobot-humanoid-runtime` 使用 `python-can` + SocketCAN（`can0`/`can1`），ZQWL 插上后不会出现网卡，必须换适配器或改总线层。**

---

## 2. 官方下载地址（智嵌物联）

基础 URL：`http://39.108.220.80/download/user/ZQWL/UCANFD/`

| 资源 | 路径 | 大小 | 用途 |
|------|------|------|------|
| **二次开发资料 V1.20** | `example_code/二次开发资料 V1.20.zip` | ~35MB | Windows DLL + 例程（Python/C#/Qt） |
| 通讯协议 | `MANUAL/ZQWL-USBCANFD二次开发通讯协议_V1.05.pdf` | 2.1MB | **Linux 串口开发依据** |
| 规格书 | `MANUAL/ZQWL-USBCANFD规格书V1.0.2.pdf` | 3.0MB | 硬件说明 |
| 上位机工具 | `TOOL/ZQWL-USB-CANFD-Tool V1.3.8.rar` | 421KB | Windows 调试 |
| Windows 驱动 | `TOOL/ZQWL-CANFD_Drivers.rar` | 14KB | 仅 Windows |
| readme | `readme.txt` | 616B | 两种开发方式说明 |

### 一键下载（在 Pi 上）

```bash
mkdir -p ~/zqwl-sdk && cd ~/zqwl-sdk
wget -O zqwl_sdk_v1.20.zip \
  "http://39.108.220.80/download/user/ZQWL/UCANFD/example_code/%e4%ba%8c%e6%ac%a1%e5%bc%80%e5%8f%91%e8%b5%84%e6%96%99%20V1.20.zip"
unzip zqwl_sdk_v1.20.zip

wget -O ZQWL-USBCANFD协议_V1.05.pdf \
  "http://39.108.220.80/download/user/ZQWL/UCANFD/MANUAL/ZQWL-USBCANFD%e4%ba%8c%e6%ac%a1%e5%bc%80%e5%8f%91%e9%80%9a%e8%ae%af%e5%8d%8f%e8%ae%ae_V1.05.pdf"
```

Pi 上已下载解压目录：`/home/big/zqwl-search/zqwl_sdk_v1.20/`

### V1.20 包内容

- `zlgcan 二次开发库/`：`x86` / `x64` 的 `zlgcan.dll` + `kerneldlls/`
- `ControlCAN 二次开发库/`：经典 CAN 的 `controlcan.dll`
- 例程：`demo_python_64`、`c++_example`、`qt-example 64` 等（均 Windows）
- **无** `linux/`、**无** `aarch64/`、**无** `.so`

readme 写明两种开发方式：

1. 基于库函数（DLL，Windows）
2. **基于串口通讯协议**（跨平台，适合 Linux）

---

## 3. Linux 上可行的两条路

### 路线 A：串口协议（官方文档 + 第三方实现）

设备枚举为 CDC ACM 串口，不是 SocketCAN 网卡：

```bash
lsusb -d 3562:0100
# Bus ... ID 3562:0100 ZQWL-CANFD
ls -l /dev/ttyACM0
```

按 `ZQWL-USBCANFD二次开发通讯协议_V1.05.pdf` 自行实现收发，或参考已封装库。

**已验证可用的 aarch64 封装：BrainCo `bc-stark-sdk`**

```bash
cd ~/lerobot/lerobot-humanoid-runtime
export PATH=$HOME/.local/bin:$PATH
uv pip install bc-stark-sdk

uv run python scripts/test_zqwl_device.py
```

该 SDK 在 Pi 上实测：

- `list_zqwl_devices()` → `/dev/ttyACM0`
- `init_zqwl_canfd("/dev/ttyACM0", 1000000, 5000000)` → 成功

文档：[BrainCo Revo2 获取 SDK](https://www.brainco-hz.com/docs/revolimb-hand/revo2/get_sdk.html)（含 ZQWL CANFD 用法）

源码参考：[brainco-hand-sdk](https://github.com/BrainCoTech/brainco-hand-sdk)（`init_zqwl_canfd`、`list_zqwl_devices`）

**局限**：不提供 `can0`/`can1`，不能直接给现有 `bipedal_robot.py` 用，需要：

- 为 runtime 写 ZQWL 串口总线适配层，或
- 写「串口 ↔ vcan」桥接（工作量大）

### 路线 B：周立功兼容 API（Windows SDK 同族，Linux 库稀缺）

ZQWL 与周立功 ControlCAN / zlgcan API 兼容（官方 V1.20 即 zlgcan 库）。

周立功 Linux 资料：

- 驱动下载：https://manual.zlg.cn/web/#/146
- CANFD Linux 二次开发：https://manual.zlg.cn/web/#/188/6982
- 社区封装：`pip install zlgcan` + [jesses2025smith/rust-can](https://github.com/jesses2025smith/rust-can/tree/master/zlgcan)

**问题**：公开 `library/linux/` 通常只有 **x86_64**，树莓派 **aarch64 需向周立功/ZQWL 技术支持索取**。Jetson 等平台也有同类反馈。

---

## 4. 与 lerobot-humanoid-runtime 的对接建议

### 最快上真机（推荐）

换 **SAVVYCANFD 2CH**，插上即有 `can0`/`can1`，无需改代码：

```bash
bash scripts/diagnose_can.sh
bash scripts/start_policy_on_pi.sh
```

### 坚持用 ZQWL（runtime 已支持）

```bash
uv run python deploy/run_real_policy_sequential.py \
  --policy-dir control/policy/remote-run-fast-yawsoft \
  --use-zqwl-bus --no-with-gamepad --no-pause-between-stages

bash scripts/start_policy_on_pi.sh   # 无 can0/can1 时自动加 --use-zqwl-bus
uv run python tools/test_zqwl_bus.py --listen-s 3
```

实现：`robot/zqwl_serial_bus.py`（ZQWL 协议 V1.05，can0=通道0，can1=通道1）。

电机 48V 上电且总线接线正确后，应能收到 MIT 反馈；否则会看到 `no valid state yet`。

### 仅验证策略（无电机）

```bash
uv run python deploy/run_real_policy_sequential.py \
  --policy-dir control/policy/remote-run-fast-yawsoft \
  --use-mock-bus --no-with-imu --no-with-meshcat --no-with-gamepad \
  --no-pause-between-stages
```

---

## 5. 厂商联系

- 智嵌物联官网：https://www.zhiqwl.com/
- 技术支持可索取：Linux aarch64 `libzlgcan.so` 或 SocketCAN 驱动（说明型号 ZQWL-UCANFD-200U 类双通道、树莓派 5 / Debian arm64）

---

## 6. 相关脚本

| 脚本 | 作用 |
|------|------|
| `scripts/diagnose_can.sh` | 检查 can0/can1，识别 ZQWL vs SAVVYCANFD |
| `scripts/test_zqwl_device.py` | 用 bc-stark-sdk 枚举并初始化 ZQWL |
| `tools/test_zqwl_bus.py` | 测试 runtime 内置 ZQWL 串口适配收发 |
