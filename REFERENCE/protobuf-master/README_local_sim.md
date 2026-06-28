# 本地模拟与测试指南 (local_sim2.py)

`local_sim2.py` 是一个用于测试服务器数据接收链路的 Python 脚本。它可以在不接 STM32 板子、不接 DTU 的情况下，直接从电脑通过 MQTT 向服务器发送 `TelemetryFrame`。

## 1. 环境准备

确保你已经安装了 Python 以及必要的依赖库。

### 1.1 使用虚拟环境（推荐）

推荐使用仓库根目录的 `.venv`。如果还没有依赖，安装：

```bash
cd /Users/poli/STM32CubeIDE/workspace_2.1.1/CAN2RS485
source .venv/bin/activate        # macOS / Linux
pip install paho-mqtt protobuf pyserial python-can
```

退出虚拟环境：

```bash
deactivate
```

如果不激活虚拟环境，也可以直接用 venv 内的 python 运行：

```bash
.venv/bin/python REFERENCE/protobuf-master/local_sim2.py --mode mqtt
```

### 1.2 全局安装（不推荐）

```bash
pip install paho-mqtt protobuf pyserial python-can
```

如果你使用的是较新的 Python 版本（例如 Python 3.14），不要固定安装旧版 `protobuf==4.25.3`，否则可能在导入 `fsae_telemetry_pb2.py` 时触发兼容性错误。此时保持 `protobuf` 为当前最新版即可。

注意：你需要确保 `REFERENCE/protobuf-master/fsae_telemetry_pb2.py` 与服务器使用的 `fsae_telemetry.proto` 同步。当前仓库已生成好；如果改了 `.proto`，从仓库根目录运行：

```bash
protoc --proto_path=REFERENCE/protobuf-master/server_config/protos \
  --python_out=REFERENCE/protobuf-master \
  REFERENCE/protobuf-master/server_config/protos/fsae_telemetry.proto
```

## 2. 脚本功能

这个脚本的主要功能是：
1.  **连接 MQTT 服务器**：连接到配置的公网 IP 和端口。
2.  **模拟物理数据**：模拟车辆的加速、刹车、滑行状态，并生成相应的 RPM、电压、电流、温度、SOC、单体极值与故障码等数据，使其看起来像真实的赛车数据（有物理惯性，不是纯随机）。
3.  **发送数据**：
    *   `fsae/telemetry` Topic: 统一发送 `TelemetryFrame`（10Hz）。
    *   服务器当前实际配置仍读取顶层旧字段 `timestamp_ms/frame_id/hv_voltage/hv_current/battery_temp_max/ready_to_drive/vcu_status`，脚本会同时填这些字段和 v2 嵌套字段。
    *   脚本也会填 `TelemetryFrame.ivt_telemetry`，包括 `ivt_current_ma`、`ivt_voltage_u1_mv`、`ivt_power_w`、`ivt_energy_wh` 和对应状态位，便于模拟 IVT-S 接入后的服务器字段。
    *   脚本会填 `TelemetryFrame.motion`，包括 `gps_speed_kmh`、`accel_x_g`、`accel_y_g`、`accel_z_g`、`yaw_rate_dps`、`yaw_deg`，用于验证服务器 GPS/IMU 字段映射。
    *   `local_sim2.py` 没有真实 IVT-S 硬件，所以 `ivt_energy_wh` 用 `V * A * dt / 3600` 生成模拟值；真车链路中该字段来自 IVT-S `0x528` 硬件能量计数，不由服务器或脚本重算。
    *   `TelemetryFrame.modules` 中的 BMS 详细数据每 5 帧刷新一次（即约 2Hz），仍复用同一个 Topic 和旧的 `modules` 结构。
4.  **支持多种输出链路**：
    *   `--mode mqtt`: 直接发到服务器 MQTT
    *   `--mode serial`: 通过 USB 转 485 向 DTU 串口送 Protobuf 原始字节流
    *   `--mode both`: 同时发 MQTT 和串口，适合联调
    *   `--mode pcan`: 通过 PCAN-USB 直接模拟旧主控 `CAN1` 发包，发送旧协议电压帧、温度帧、摘要帧和 `0x03C0` 霍尔电流帧
    *   `--mode mqtt+pcan` / `serial+pcan` / `all`: 用于边喂板子边做链路对照

## 3. 配置修改

在 `local_sim2.py` 的头部，你可以修改服务器配置：

```python
# ================= 配置区域 =================
# 腾讯云服务器的公网 IP，对应 bitfsae.com
SERVER_IP = "82.157.204.124"   # 旧服务器IP为123.57.174.98
SERVER_PORT = 1883
TOPIC_TELEMETRY = "fsae/telemetry"
```

也可以直接用命令行参数覆盖，无需改文件。

## 4. 运行模拟

### 4.1 纯电脑 MQTT 直发服务器

这是最适合临时验证服务器/Grafana 的方式，不需要 PCAN、不需要串口、不需要 STM32 板子。

从仓库根目录运行：

```bash
cd /Users/poli/STM32CubeIDE/workspace_2.1.1/CAN2RS485
source .venv/bin/activate
python REFERENCE/protobuf-master/local_sim2.py --mode mqtt
```

或者不激活虚拟环境：

```bash
.venv/bin/python REFERENCE/protobuf-master/local_sim2.py --mode mqtt
```

默认会连接：

```text
82.157.204.124:1883
topic: fsae/telemetry
```

如果要显式指定服务器：

```bash
.venv/bin/python REFERENCE/protobuf-master/local_sim2.py \
  --mode mqtt \
  --server-ip 82.157.204.124 \
  --server-port 1883 \
  --topic fsae/telemetry
```

**VSCode 方式**：

1. `Cmd+Shift+P` → `Python: Select Interpreter` → 选择 `protobuf-master/.venv/bin/python`
2. 直接按 ▶ 运行，或在终端中执行 `python REFERENCE/protobuf-master/local_sim2.py --mode mqtt`

### 4.2 其他运行示例

```bash
python REFERENCE/protobuf-master/local_sim2.py --mode mqtt
```

如果要测试 USB 转 485 + DTU：

```bash
python REFERENCE/protobuf-master/local_sim2.py --mode serial --serial-port COM3 --baudrate 115200
```

如果要一边喂 DTU，一边直接发 MQTT 对照：

```bash
python REFERENCE/protobuf-master/local_sim2.py --mode both --serial-port COM3 --baudrate 115200
```

如果要直接通过 PCAN 模拟旧主控 CAN1：

```bash
python REFERENCE/protobuf-master/local_sim2.py --mode pcan --pcan-channel PCAN_USBBUS1 --pcan-bitrate 250000
```

如果要一边经 PCAN 喂 STM32 板子，一边保留串口或 MQTT 对照：

```bash
python REFERENCE/protobuf-master/local_sim2.py --mode serial+pcan --serial-port COM3 --pcan-channel PCAN_USBBUS1
python REFERENCE/protobuf-master/local_sim2.py --mode mqtt+pcan --pcan-channel PCAN_USBBUS1
python REFERENCE/protobuf-master/local_sim2.py --mode all --serial-port COM3 --pcan-channel PCAN_USBBUS1
```

说明：

*   `pcan` 后端依赖 `python-can` 和 PEAK 驱动。
*   默认按旧主控 `CAN1` 速率 `250000` bps 发包。
*   发送内容按当前固件接收逻辑组织：6 组模组电压扩展帧、6 组温度扩展帧、500 ms 一次的摘要/极值/状态/报警帧，以及每个循环一次的标准帧 `0x03C0` 霍尔电流。

如果 DTU 需要用特定包尾来切包，可以追加：

```bash
python REFERENCE/protobuf-master/local_sim2.py --mode serial --serial-port COM3 --packet-suffix-hex 0D0A
```

## 5. 服务器侧验证

脚本运行后每 10 帧会打印一次当前 HV 电压、电流、Wh 和 SOC。服务器 Telegraf 会把同一条 MQTT 消息写入 `telemetry` measurement。

可以 SSH 到服务器快速看最近数据：

```bash
ssh bitfsae-com
sudo docker exec -it fsae_influxdb influx -database fsae_db \
  -execute 'SELECT last(hv_voltage), last(hv_current), last(ivt_voltage_u1_mv), last(ivt_current_ma), last(ivt_power_w), last(ivt_energy_wh) FROM "telemetry"'
```

## 6. 常见问题

*   **依赖装到了全局而非虚拟环境**：运行 `which python` 确认当前 python 指向 `.venv/bin/python`；如果指向系统 python，说明虚拟环境未激活，执行 `source .venv/bin/activate`。

*   **缺少模块错误**：如果提示 `ModuleNotFoundError: No module named 'fsae_telemetry_pb2'`，说明你还没生成 Python 的 Protobuf 库文件。请运行 `protoc --python_out=. fsae_telemetry.proto`（确保你安装了 protoc 编译器）。
*   **看不到新增字段**：如果脚本还能正常发送，但服务器只收到旧字段，通常是因为本地 `fsae_telemetry_pb2.py` 还是旧版本，需要用最新 `.proto` 重新生成。
*   **Protobuf 导入时报错**：如果安装完依赖后，在导入 `fsae_telemetry_pb2` 或 `google.protobuf` 时仍报错，并且你的 Python 版本较新（如 3.14），优先升级 `protobuf` 到最新版，而不是固定到旧版。
*   **连接失败**：检查 `SERVER_IP` 是否正确，以及云服务器安全组和防火墙是否放行 TCP `1883` 入站。
*   **串口打不开**：确认 USB 转 485 的串口号是否正确（设备管理器里查看），并关闭其他占用该串口的软件。
*   **DTU 收不到完整包**：优先检查 DTU 串口参数是否和脚本一致；如果 DTU 依赖包尾切包，再用 `--packet-suffix-hex` 增加结束符。
*   **PCAN 打不开**：先确认已安装 PEAK 驱动，且 `python-can` 能识别 `pcan` 接口；通道名通常是 `PCAN_USBBUS1`。
*   **板子收不到 CAN**：优先检查 PCAN 波特率是否为 `250000`，以及接线是否接在板子的 `CAN1` 总线上。
