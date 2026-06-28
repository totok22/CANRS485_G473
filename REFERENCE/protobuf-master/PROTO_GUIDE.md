# Protobuf 协议维护指南

本项目的通信核心以 `server_config/protos/fsae_telemetry.proto` 为唯一协议源。服务器、STM32 端生成代码、`local_sim2.py` 使用的 Python pb2 都必须从这份协议同步。
为了保证服务器端（Telegraf -> InfluxDB）和车端编码正常，**请严格遵守以下规范进行修改**。

## 1. 核心文件

*   `server_config/protos/fsae_telemetry.proto`: 唯一协议源，定义单 Topic 使用的顶层 `TelemetryFrame` 及其嵌套消息。
*   `server_config/protos/fsae_telemetry.options`: Nanopb 生成选项，定义 STM32 静态数组最大长度和字符串长度。它不是服务器解析协议本身，但必须和 `.proto` 一起同步到 `stm32_code/`。
*   `server_config/telegraf/telegraf.conf`: 服务器 XPath 映射。新增或改名字段后必须同步这里，否则数据不会写入 InfluxDB。
*   `stm32_code/fsae_telemetry.proto` / `.options`: 车端生成代码副本，应与 `server_config/protos/` 中同名文件保持一致。
*   `fsae_telemetry_pb2.py`: `local_sim2.py` 使用的 Python 生成文件，应由唯一协议源重新生成。

## 2. 修改规范 (CRITICAL)

服务器端的 Telegraf 配置文件 (`telegraf.conf`) 使用了 XPath 来提取 Protobuf 数据。这意味着：

1.  **禁止修改仍在服务器 XPath 中使用的字段名称和类型**：
    *   例如：`uint32 module_id = 1;` 中的 `module_id` 被写入在 `telegraf.conf` 中。如果你把它改名为 `id`，服务器将无法解析该字段，数据会丢失。
    *   如果必须修改，你**必须**同步修改服务器上 `telegraf.conf` 中的 `xpath` 映射，并重启 Telegraf 容器。

2.  **禁止破坏现有字段 ID 和字段名**：
    *   Protobuf 依赖 ID (`= 1`, `= 2`) 来序列化。修改 ID 会导致新旧版本不兼容。
    *   当前服务器和 STM32 仍使用 `TelemetryFrame` 顶层旧字段 `1~14`：
        `timestamp_ms`、`frame_id`、`hv_voltage`、`hv_current`、`battery_temp_max`、`ready_to_drive`、`vcu_status` 等。不要删除、reserved 或改名这些字段。
    *   历史试验文件 `fsae_telemetry_v2.proto` 已废弃并删除；不要再按其中的 repeated `cell_mv/temp_dc` 结构生成代码。

3.  **新增字段**：
    *   保持单 Topic 时，优先继续在 `TelemetryFrame` 末尾追加新的嵌套消息字段，使用新的 ID。
    *   例如：`VehicleState vehicle_state = 28;`
    *   **注意**：新增字段后，如果能在服务器数据库看到它，还需要手动修改服务器的 `telegraf.conf`，添加对应的 XML Path 映射。否则服务器只会忽略这个新数据，虽然不会报错。
    *   **单 Topic**方案：`TelemetryFrame` 既承载基础遥测，也承载 `modules` 中的 BMS 详细数据。新增字段时同时检查 `telemetry` 和 `bms_data` 两个 measurement 的采集逻辑。
    *   带宽有限，优先保留上云必须的摘要量。当前 BMS 摘要字段只保留：`battery_soc`、最大/最小单体电压及编号、最大/最小温度及编号、`battery_fault_code`。
    *   当前约定：BMS 相关字段继续沿用老结构，`modules = 15` 和 `16~25` 的摘要字段不要随意改成新的 repeated 形状。
    *   当前 IVT-S 字段位于 `TelemetryFrame.ivt_telemetry = 31`，其中电压使用 U1：`voltage_u1_mv` / `voltage_u1_state`；`power_w` 为 U1 电压乘 IVT 电流得到的瞬时功率。

4.  **数组长度控制**：
    *   如果有 `repeated` 字段，必须在 `fsae_telemetry.options` 中指定 `max_count`。这是为了让 STM32 (C语言) 能够静态分配内存。

## 3. 现有结构参考

**fsae_telemetry.proto:**

```protobuf
syntax = "proto3";
package fsae;

message BatteryModule {
    uint32 module_id = 1;
    // 23 节电芯电压
    uint32 v01 = 2;
    ...
    // 8 个温度
    sint32 t1 = 30;
    ...
}

message TelemetryFrame {
    // 旧顶层字段仍在服务器和 STM32 中使用，不能删除
    uint32 timestamp_ms = 1;
    uint32 frame_id = 2;
    float hv_voltage = 6;
    float hv_current = 7;
    float battery_temp_max = 8;
    uint32 ready_to_drive = 13;
    uint32 vcu_status = 14;

    // BMS 详细数据
    repeated BatteryModule modules = 15;

    // BMS 摘要
    uint32 battery_soc = 16;
    ...

    // 追加嵌套字段
    PacketHeader header = 26;
    FastTelemetry fast_telemetry = 27;
    VehicleState vehicle_state = 28;
    ThermalSummary thermal_summary = 29;
    repeated Alarm alarms = 30;
    IvtTelemetry ivt_telemetry = 31;
    EnergyMeterTelemetry energy_meter = 32;
    MotionTelemetry motion = 33;
}
```

**fsae_telemetry.options (Nanopb):**

```plaintext
fsae.Alarm.message                 max_size:64
fsae.TelemetryFrame.modules        max_count:6
fsae.TelemetryFrame.alarms         max_count:8
fsae.ThermalSensorSummary.chunks   max_count:4
fsae.ThermalSummary.sensors        max_count:4
fsae.VehicleState.motors           max_count:4
```

## 4. 常见操作流程

### 场景：修改协议后同步全链路

从仓库根目录执行：

```bash
# 1. 修改唯一协议源
$EDITOR REFERENCE/protobuf-master/server_config/protos/fsae_telemetry.proto
$EDITOR REFERENCE/protobuf-master/server_config/protos/fsae_telemetry.options

# 2. 同步给 STM32 生成目录
cp REFERENCE/protobuf-master/server_config/protos/fsae_telemetry.proto \
   REFERENCE/protobuf-master/stm32_code/fsae_telemetry.proto
cp REFERENCE/protobuf-master/server_config/protos/fsae_telemetry.options \
   REFERENCE/protobuf-master/stm32_code/fsae_telemetry.options

# 3. 重新生成 Python pb2，供 local_sim2.py 使用
protoc --proto_path=REFERENCE/protobuf-master/server_config/protos \
  --python_out=REFERENCE/protobuf-master \
  REFERENCE/protobuf-master/server_config/protos/fsae_telemetry.proto

# 4. 重新生成 STM32 nanopb
cd REFERENCE/protobuf-master/stm32_code
/Users/poli/STM32CubeIDE/workspace_2.1.1/CAN2RS485/.venv/bin/python \
  /Users/poli/nanopb-0.4.9.1-macosx-x86/generator/nanopb_generator.py \
  fsae_telemetry.proto
cd -

# 5. 拷贝生成结果到工程实际编译目录
cp REFERENCE/protobuf-master/stm32_code/fsae_telemetry.pb.h \
   Middlewares/Third_Party/nanopb/inc/fsae_telemetry.pb.h
cp REFERENCE/protobuf-master/stm32_code/fsae_telemetry.pb.c \
   Middlewares/Third_Party/nanopb/src/fsae_telemetry.pb.c

# 6. 修改并同步 Telegraf XPath
$EDITOR REFERENCE/protobuf-master/server_config/telegraf/telegraf.conf
```

然后构建验证：

```bash
cmake --build --preset Debug
python3 -m py_compile REFERENCE/protobuf-master/local_sim2.py
```

服务器部署时，把 `server_config/protos/fsae_telemetry.proto` 和 `server_config/telegraf/telegraf.conf` 上传到 `bitfsae-com:~/fsae_project/` 对应目录，并重启 Telegraf。新增 GPS/IMU 字段时，服务器端必须同时更新 XPath：`//motion/gps_speed_kmh`、`//motion/accel_x_g`、`//motion/accel_y_g`、`//motion/accel_z_g`、`//motion/yaw_rate_dps`、`//motion/yaw_deg`。

### 场景：我想加一个“整车状态”嵌套数据

1.  **修改 Proto**: 在 `fsae_telemetry.proto` 的 `TelemetryFrame` 末尾添加新的 message 字段：
    ```protobuf
    VehicleState vehicle_state = 28;
    ```
2.  **生成代码**:
    *   **Python (本地模拟)**: 运行 `protoc --python_out=. fsae_telemetry.proto`，这会生成` fsae_telemetry_pb2.py`
    *   **STM32 (车载)**: 运行 Nanopb 生成器，更新 STM32 工程中的 `.c/.h` 文件。
3.  **修改服务器配置 (重要)**:
    *   登录服务器，编辑 `server_config/telegraf/telegraf.conf`。
    *   在 `[[inputs.mqtt_consumer.xpath.fields]]` 下添加：
        ```toml
        speed_kmh = "number(//vehicle_state/speed_kmh)"
        ```
    *   重启 Telegraf: `docker-compose restart telegraf`
