# CANRS485_G473

`CANRS485_G473` 是旧 `CAN2RS485` 固件的 STM32G473 版本，用于把车上 CAN 数据编码为
`fsae_TelemetryFrame`，再通过 `USART2 + RS485` 发送给 DTU。

本工程目标是直接替代旧 F405 工程，同时保留旧项目已经验证过的 CAN 解析、protobuf 编码、
RS485 上报节奏和服务器协议资料。

## 当前状态

- 已从旧 `CAN2RS485` 迁移主业务代码到 `Core/Src/app.c`。
- 旧 F405 `CAN1` 250 kbit/s 业务链路映射到 G473 `FDCAN3`。
- 旧 F405 `CAN2` 500 kbit/s 业务链路映射到 G473 `FDCAN2` / `CANB`。
- `FDCAN1` / `CANA` 已初始化并配置全通接收，但当前不参与旧业务解析。
- `CANC` 的 MCP2518FD 驱动尚未实现。
- `USART2` 使用硬件 RS485 DE，`PA1=DE`，不再使用旧 F405 的 `RS485_DIR` GPIO。
- protobuf / nanopb 发送格式保持旧项目的 `fsae_TelemetryFrame`。
- 已包含旧项目中真实有用的 `REFERENCE` 和 `DOC` 资料；旧 F405 的 `STM32CubeF4-master`
  固件包没有复制到本仓库。

## 硬件与总线映射

| 名称 | 控制器 | 引脚 | 速率 | 当前软件职责 |
| --- | --- | --- | --- | --- |
| CANA | FDCAN1 | PA11 / PA12 | 500 kbit/s | 已初始化，暂不解析业务 |
| CANB | FDCAN2 | PB12 / PB13 | 500 kbit/s | 旧 F405 `CAN2` 的替代口 |
| CAN1 | FDCAN3 | PB3 / PB4 | 250 kbit/s | 旧 F405 `CAN1` 的替代口 |
| CANC | MCP2518FD + SPI1 | PA4 / PA5 / PA6 / PA7 + PC4 | 500 kbit/s | 待实现驱动 |
| RS485 | USART2 | PA1 / PA2 / PA3 | 115200 8N1 | DTU 上报口 |
| 调试串口 | USART1 | PA9 / PA10 | 115200 8N1 | 默认关闭的调试 CLI |
| 状态灯 | GPIO | PC8 | - | 高电平点亮 |

## 业务行为

- 收到 `CAN1/FDCAN3` 或 `CANB/FDCAN2` 任一总线的有效业务数据后开始周期上报。
- 基础遥测帧周期为 `100 ms`。
- 带 `modules` 的低频帧周期为 `500 ms`。
- CAN RX 回调单次最多处理 `8` 帧，避免长时间占用中断。
- 调试 CLI 默认关闭：`APP_DEBUG_CLI_ENABLE = 0U`。
- 电池模组数量、电芯数量、温度点数量和 nanopb `modules` 容量有编译期断言保护。
- CANB 能量计逻辑沿用旧项目的自动模式：
  - 有新鲜 `0x430` 状态帧时，`0x521/0x522/0x526/0x528` 按赛会 FS Datalogger 大端 result 帧解析。
  - 没有新鲜 `0x430` 时，`0x52x` 结果帧按现场约定用字节序区分赛会能量计和自家 IVT。

## 目录说明

- `Core/Src/app.c`：业务主逻辑，包含 CAN 解析、状态聚合、protobuf 编码、RS485 发送。
- `Middlewares/Third_Party/nanopb/`：旧项目迁移来的 nanopb 与生成代码。
- `REFERENCE/protobuf-master/`：服务器、protobuf、仿真和 STM32 端协议资料。
- `REFERENCE/旧主控 CAN 通讯协议.md`：旧主控 CAN 协议。
- `REFERENCE/新主控 CAN 通讯协议.md`：新主控 CAN 协议。
- `REFERENCE/*.dbc`：CANA/CANB、IVT、FS Datalogger 等参考 DBC。
- `DOC/`：当前 G473 工程说明和迁移待办。
- `CANRS485_G473.ioc`：CubeMX 配置源头。
- `AGENTS.md`：给代码代理使用的工程约束。

## 构建

```bash
cmake --preset Debug
cmake --build --preset Debug
```

清理后重新构建：

```bash
cmake --build --preset Debug --clean-first
```

构建产物位于 `build/Debug/`：

- `CANRS485_G473.elf`
- `CANRS485_G473.hex`
- `CANRS485_G473.bin`

## 当前仍需完善

- 实现 `CANC` / MCP2518FD 驱动。
- 明确 `CANA` 的业务协议后，将对应解析接入 `App_ProcessCanRx()`。
- 增加 FDCAN 错误计数、协议状态、bus-off 状态和发送失败统计。
- 按 G473 工程更新调试 CLI 输出，使其显示 `CANA/CANB/CAN1` 而不是旧 `CAN1/CAN2` 视角。
- 如需修改 protobuf 字段，必须同步更新 `.proto`、nanopb 生成代码、服务器 `telegraf.conf`
  和相关说明文档。

## 参考资料

- [DOC/todo.md](./DOC/todo.md)
- [DOC/电路信息.md](./DOC/%E7%94%B5%E8%B7%AF%E4%BF%A1%E6%81%AF.md)
- [REFERENCE/旧主控 CAN 通讯协议.md](./REFERENCE/%E6%97%A7%E4%B8%BB%E6%8E%A7%20CAN%20%E9%80%9A%E8%AE%AF%E5%8D%8F%E8%AE%AE.md)
- [REFERENCE/新主控 CAN 通讯协议.md](./REFERENCE/%E6%96%B0%E4%B8%BB%E6%8E%A7%20CAN%20%E9%80%9A%E8%AE%AF%E5%8D%8F%E8%AE%AE.md)
- [REFERENCE/protobuf-master/README.md](./REFERENCE/protobuf-master/README.md)
- [REFERENCE/protobuf-master/PROTO_GUIDE.md](./REFERENCE/protobuf-master/PROTO_GUIDE.md)
- [REFERENCE/protobuf-master/STM32_GUIDE.md](./REFERENCE/protobuf-master/STM32_GUIDE.md)
