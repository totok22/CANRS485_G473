# Grafana 指南

链路：`CAN2RS485 -> Protobuf TelemetryFrame -> MQTT fsae/telemetry -> Telegraf xpath_protobuf -> InfluxDB fsae_db -> Grafana`。线上入口：`https://bitfsae.com/monitor/`。

## 1. 服务器与数据源

Grafana 数据源使用 InfluxDB 1.x：

| 项 | 值 |
| --- | --- |
| URL | `http://influxdb:8086` |
| Database | `fsae_db` |
| Query language | `InfluxQL` |
| Min time interval | `100ms` |

常用检查：

```bash
ssh bitfsae-com
cd ~/fsae_project
sudo docker compose ps
sudo docker compose logs --tail=80 telegraf
sudo docker exec -it fsae_influxdb influx -database fsae_db -execute 'SHOW MEASUREMENTS'
sudo docker exec -it fsae_influxdb influx -database fsae_db -execute 'SHOW FIELD KEYS FROM "telemetry"'
sudo docker exec -it fsae_influxdb influx -database fsae_db -execute 'SHOW FIELD KEYS FROM "motor_state"'
sudo docker exec -it fsae_influxdb influx -database fsae_db -execute 'SHOW TAG VALUES FROM "bms_data" WITH KEY = "module_id"'
```

## 2. Measurements

| Measurement | 来源 | 写入周期 | Tags | 主要 fields |
| --- | --- | ---: | --- | --- |
| `telemetry` | `TelemetryFrame` 顶层摘要、`ivt_telemetry`、`energy_meter`、`motion` | 10 Hz | 无 | `hv_voltage`, `hv_current`, `battery_temp_max`, `battery_soc`, `battery_fault_code`, `vehicle_mode`, `ivt_*`, `energy_meter_*`, `gps_speed_kmh`, `accel_*_g`, `yaw_*` |
| `bms_data` | `TelemetryFrame.modules` | 2 Hz | `module_id` | `v_01`~`v_23`, `t_1`~`t_8` |
| `motor_state` | `vehicle_state.motors` | 随 CANB 0x502~0x509 | `position` | `rpm`, `torque_nm`, `motor_temp_dc`, `inverter_temp_dc`, `igbt_temp_dc`, `diagnostic_number`, `motor_error`, `logic_state`, `power_w` |
| `thermal_summary` | `thermal_summary.sensors` | 扩展/仿真 | `position` | `min_temp_centi_c`, `max_temp_centi_c`, `avg_temp_centi_c` |
| `alarm_state` | `alarms` | 扩展/仿真 | `alarm_id` | `severity`, `message` |

`position` 枚举值来自 protobuf：`1=FL`, `2=FR`, `3=RL`, `4=RR`。

`vehicle_mode` 枚举值来自 protobuf：`0=unknown`, `1=default`, `2=straight`, `3=autocross`, `4=skidpad`, `5=endurance`。

## 3. 单位

| 字段 | 原始单位 | Grafana 处理 |
| --- | --- | --- |
| `hv_voltage` | V | Unit `volt` |
| `hv_current` | A | Unit `ampere` |
| `apps_position` | % | Unit `percent (0-100)` |
| `brake_pressure` | kPa | Unit `pressurekpa` 或 custom `kPa` |
| `steering_angle` | deg | Unit `degree` |
| `gps_speed_kmh` | km/h | Unit `kmh` 或 custom `km/h` |
| `accel_x_g`, `accel_y_g`, `accel_z_g` | g | custom `g` |
| `yaw_rate_dps` | deg/s | custom `deg/s` |
| `yaw_deg` | deg | Unit `degree` |
| `battery_temp_max` | degC | Unit `celsius` |
| `battery_soc` | % | Unit `percent (0-100)` |
| `max_cell_voltage`, `min_cell_voltage`, `v_01`~`v_23`, `ivt_voltage_u1_mv`, `energy_meter_voltage_mv` | mV | Unit `millivolt` |
| `ivt_current_ma`, `energy_meter_current_ma` | mA | 查询 `/1000` 后 Unit `ampere`，或直接 Unit `mA` |
| `ivt_power_w`, `energy_meter_power_w` | W | Unit `watt` |
| `ivt_energy_wh`, `energy_meter_energy_wh` | Wh | Unit `watt-hour` 或 custom `Wh` |
| `max_temp`, `min_temp`, `t_1`~`t_8`, `motor_temp_dc`, `inverter_temp_dc`, `igbt_temp_dc` | 0.1 degC | 查询 `/10` 后 Unit `celsius` |
| `*_centi_c` | 0.01 degC | 查询 `/100` 后 Unit `celsius` |
| `torque_nm` | AMK 原始 0.1%Mn | 查询 `* 9.8 / 1000` 后 Unit `newtonmeter` 或 custom `Nm` |

能量计来源：

| `energy_meter_source` | 含义 |
| ---: | --- |
| `1` | 自家 IVT-S：小端 `0x521/0x522/0x526/0x528` 结果帧 |
| `2` | 赛会能量计：`0x430` 状态帧，或大端 `0x521/0x522/0x526/0x528` 结果帧 |

赛会能量计型号不完全一致：有的发送 `0x430` 状态帧，有的只发送 IVT 兼容的 `0x52x` 结果帧；若型号发送 `0x526` 功率或 `0x528` Wh，固件会填充 `energy_meter_power_w` / `energy_meter_energy_wh`。

## 4. 查询规则

趋势面板必须带 `$timeFilter`：

```sql
SELECT "hv_voltage" FROM "telemetry" WHERE $timeFilter
```

最新快照不要带 `$timeFilter`，避免 Grafana 时间窗口太短导致空值：

```sql
SELECT "hv_voltage", "hv_current", "battery_soc", "battery_fault_code", "vehicle_mode"
FROM "telemetry"
ORDER BY time DESC
LIMIT 1
```

Tag 条件必须用字符串：

```sql
WHERE "module_id" = '1'
WHERE "position" = '3'
```

InfluxDB 会保留历史 field key。判断实时数据用 `SELECT last(...)`，不要只看 Grafana 字段下拉。

## 5. Dashboard 结构

建议 6 行：

| Row | 面板 |
| --- | --- |
| Overview | HV Voltage, HV Current, Vehicle Mode, Energy Meter Source, Energy Meter Power, SOC, Max Battery Temp, Fault Code |
| Energy Meter | IVT/FS 电压、电流、功率、能量、状态、MsgCnt |
| BMS Cells | 单模组 23 节电芯柱状图、包级 max/min/压差、关键电芯趋势 |
| BMS Temps | 单模组 8 路温度柱状图、包级 max/min/温差 |
| Motors | 四轮 rpm、扭矩 Nm、电机温度、逆变器温度、IGBT 温度、诊断码 |
| Debug | 最新 telemetry、字段完整性、Telegraf/Influx 状态 |

## 6. 常用变量

`module_id`：

```sql
SHOW TAG VALUES FROM "bms_data" WITH KEY = "module_id"
```

`position`：

```sql
SHOW TAG VALUES FROM "motor_state" WITH KEY = "position"
```

变量使用：

```sql
WHERE "module_id" = '$module_id'
WHERE "position" = '$position'
```

## 7. 常用查询

### Overview

```sql
SELECT "hv_voltage" FROM "telemetry" WHERE $timeFilter
SELECT "hv_current" FROM "telemetry" WHERE $timeFilter
SELECT "battery_soc" FROM "telemetry" WHERE $timeFilter
SELECT "battery_temp_max" FROM "telemetry" WHERE $timeFilter
SELECT "battery_fault_code" FROM "telemetry" WHERE $timeFilter
SELECT "vehicle_mode" FROM "telemetry" WHERE $timeFilter
```

最新摘要：

```sql
SELECT "hv_voltage", "hv_current", "battery_soc", "battery_temp_max", "battery_fault_code", "vehicle_mode", "energy_meter_source"
FROM "telemetry"
ORDER BY time DESC
LIMIT 1
```

### CANB DataLogger

油门开度、刹车油压、方向盘转角来自 CANB `0x305`：

```sql
SELECT "apps_position", "brake_pressure", "steering_angle"
FROM "telemetry"
WHERE $timeFilter
```

### CANB GPS/IMU

GPS 速度来自 CANB `0x301`；三轴加速度来自 `0x61`；横摆角速度取 `0x62 IMU_GyroZ`；横摆角来自 `0x65`：

```sql
SELECT "gps_speed_kmh", "accel_x_g", "accel_y_g", "accel_z_g", "yaw_rate_dps", "yaw_deg"
FROM "telemetry"
WHERE $timeFilter
```

### Energy Meter

当前能量计电压：

```sql
SELECT "energy_meter_voltage_mv" / 1000 FROM "telemetry" WHERE $timeFilter
```

当前能量计电流：

```sql
SELECT "energy_meter_current_ma" / 1000 FROM "telemetry" WHERE $timeFilter
```

当前能量计功率：

```sql
SELECT "energy_meter_power_w" FROM "telemetry" WHERE $timeFilter
```

当前能量计最新状态：

```sql
SELECT "energy_meter_source", "energy_meter_status", "energy_meter_msg_counter"
FROM "telemetry"
ORDER BY time DESC
LIMIT 1
```

IVT-S 细节：

```sql
SELECT "ivt_voltage_u1_mv" / 1000 FROM "telemetry" WHERE $timeFilter
SELECT "ivt_current_ma" / 1000 FROM "telemetry" WHERE $timeFilter
SELECT "ivt_power_w" FROM "telemetry" WHERE $timeFilter
SELECT "ivt_energy_wh" FROM "telemetry" WHERE $timeFilter
SELECT "ivt_current_state", "ivt_voltage_u1_state", "ivt_energy_state" FROM "telemetry" ORDER BY time DESC LIMIT 1
```

IVT state：`0` 正常；bit1/bit2/bit3 分别表示超规格/测量错误/系统错误。

### BMS Cells

单模组电芯快照，适合 Bar chart：

```sql
SELECT v_01, v_02, v_03, v_04, v_05, v_06, v_07, v_08, v_09, v_10, v_11, v_12, v_13, v_14, v_15, v_16, v_17, v_18, v_19, v_20, v_21, v_22, v_23
FROM "bms_data"
WHERE "module_id" = '$module_id'
ORDER BY time DESC
LIMIT 1
```

包级极值和压差：

```sql
SELECT "max_cell_voltage", "min_cell_voltage" FROM "telemetry" WHERE $timeFilter
SELECT "max_cell_voltage" - "min_cell_voltage" FROM "telemetry" WHERE $timeFilter
SELECT "max_cell_voltage_no", "min_cell_voltage_no" FROM "telemetry" ORDER BY time DESC LIMIT 1
```

关键电芯趋势：

```sql
SELECT "v_01", "v_12", "v_23"
FROM "bms_data"
WHERE "module_id" = '$module_id' AND $timeFilter
```

### BMS Temps

单模组温度快照：

```sql
SELECT "t_1" / 10, "t_2" / 10, "t_3" / 10, "t_4" / 10, "t_5" / 10, "t_6" / 10, "t_7" / 10, "t_8" / 10
FROM "bms_data"
WHERE "module_id" = '$module_id'
ORDER BY time DESC
LIMIT 1
```

包级温度极值：

```sql
SELECT "max_temp" / 10, "min_temp" / 10 FROM "telemetry" WHERE $timeFilter
SELECT "max_temp_no", "min_temp_no" FROM "telemetry" ORDER BY time DESC LIMIT 1
```

### Motors

四轮转速：

```sql
SELECT "rpm" FROM "motor_state" WHERE $timeFilter GROUP BY "position"
```

四轮扭矩，显示单位 Nm。字段名保留 `torque_nm` 兼容旧表，但数据库里存的是 AMK 原始 `0.1%Mn`；当前电机 `Mn=9.8 Nm`：

```sql
SELECT "torque_nm" * 9.8 / 1000 FROM "motor_state" WHERE $timeFilter GROUP BY "position"
```

四轮温度：

```sql
SELECT "motor_temp_dc" / 10 FROM "motor_state" WHERE $timeFilter GROUP BY "position"
SELECT "inverter_temp_dc" / 10 FROM "motor_state" WHERE $timeFilter GROUP BY "position"
SELECT "igbt_temp_dc" / 10 FROM "motor_state" WHERE $timeFilter GROUP BY "position"
```

最新诊断码：

```sql
SELECT "diagnostic_number", "motor_error"
FROM "motor_state"
WHERE "position" = '$position'
ORDER BY time DESC
LIMIT 1
```

四轮 LogicState：

```sql
SELECT "logic_state" FROM "motor_state" WHERE $timeFilter GROUP BY "position"
```

### Faults

```sql
SELECT "battery_fault_code" FROM "telemetry" WHERE $timeFilter
SELECT "severity", "message" FROM "alarm_state" WHERE $timeFilter ORDER BY time DESC LIMIT 50
```

## 8. 面板配置

| 场景 | Visualization | Transform | Unit/阈值 |
| --- | --- | --- | --- |
| 最新值 | Stat/Gauge | `Reduce -> Last not null` | 按字段单位设置 |
| 电芯 23 根柱 | Bar chart | `Organize fields` 隐藏 `Time`，重命名 `Cell 01`~`Cell 23` | `millivolt`; 3300 yellow, 3000 red |
| 单模组温度柱 | Bar chart | `Organize fields` 隐藏 `Time` | `celsius`; 50 yellow, 60 red |
| 多轮位曲线 | Time series | 保留 `position` series | rpm/Nm/celsius |
| 故障码/状态位 | Stat/Table | `Last not null` 或最新快照查询 | `>0` red |

推荐 Display name：

| 字段 | 显示名 |
| --- | --- |
| `energy_meter_source` | `Meter Source` |
| `energy_meter_voltage_mv / 1000` | `Meter Voltage` |
| `energy_meter_current_ma / 1000` | `Meter Current` |
| `energy_meter_power_w` | `Meter Power` |
| `torque_nm * 9.8 / 1000` | `Motor Torque` |
| `igbt_temp_dc / 10` | `IGBT Temp` |

## 9. 排查

| 现象 | 检查 |
| --- | --- |
| Overview 空 | `sudo docker compose ps`; `sudo docker compose logs --tail=80 telegraf`; `SHOW FIELD KEYS FROM "telemetry"` |
| 新字段无值 | 确认服务器 `protos/fsae_telemetry.proto` 和 `telegraf/telegraf.conf` 已更新并重启 Telegraf |
| Energy Meter Source 一直是 IVT | CANB 是否实际收到赛会大端 `0x52x`；如果只收到小端 `0x52x`，会按自家 IVT 处理 |
| Wh 不动 | CANB 是否有 `0x528`；部分赛会能量计型号不发送 Wh |
| motor_state 空 | CANB 是否有 `0x502`~`0x509` 标准帧 |
| 单模组 BMS 空 | `modules` 只 2 Hz；查 `SELECT * FROM "bms_data" ORDER BY time DESC LIMIT 1` |
| tag 查不到 | `module_id` 和 `position` 是 tag，条件必须写字符串 |
| 数值倍率错 | `*_dc` 和 `t_*` 要 `/10`；mA 转 A 要 `/1000`；mV 转 V 要 `/1000` |
| 字段下拉有但 no data | Influx 保留历史 field key；用 `SELECT last("field")` 看当前是否仍写入 |

快速查最新帧：

```bash
sudo docker exec -it fsae_influxdb influx -database fsae_db -execute 'SELECT * FROM "telemetry" ORDER BY time DESC LIMIT 1'
sudo docker exec -it fsae_influxdb influx -database fsae_db -execute 'SELECT * FROM "motor_state" ORDER BY time DESC LIMIT 4'
sudo docker exec -it fsae_influxdb influx -database fsae_db -execute 'SELECT * FROM "bms_data" WHERE "module_id"='\''1'\'' ORDER BY time DESC LIMIT 1'
```
