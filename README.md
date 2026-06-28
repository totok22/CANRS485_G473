# CANRS485_G473

`CANRS485_G473` 是旧 `CAN2RS485` 的 STM32G473 版本，用于把车上 CAN 数据编码为
`fsae_TelemetryFrame`，再通过 `USART2 + RS485` 发送给 DTU。

## 当前状态

- 旧 F405 `CAN1` 250 kbit/s 业务已映射到 G473 `FDCAN3 / CAN1`。
- 旧 F405 `CAN2` 500 kbit/s 业务已映射到 G473 `FDCAN2 / CANB`。
- `FDCAN1 / CANA` 已初始化，当前不参与旧业务解析。
- `CANC / MCP2518FD` 已有基础 SPI/寄存器/FIFO 驱动，当前只记录原始帧，不参与旧业务解析。
- protobuf / nanopb 发送格式保持旧项目的 `fsae_TelemetryFrame`。
- 旧项目的协议、DBC、protobuf/server 资料已收进 `REFERENCE/`。

## 主要目录

- `Core/Src/app.c`：业务主逻辑；`Core/Src/app_*.c` 为拆分出的内部辅助模块。
- `Middlewares/Third_Party/nanopb/`：nanopb 与生成代码。
- `DOC/电路信息.md`：G473 硬件连接摘要。
- `DOC/CubeMX配置.md`：CubeMX/固件配置摘要。
- `DOC/MCP2518FD驱动.md`：CANC 外置 CAN 控制器驱动边界。
- `DOC/todo.md`：迁移状态和待办。
- `REFERENCE/`：CAN 协议、DBC、protobuf、服务器配置和仿真资料。
- `CANRS485_G473.ioc`：CubeMX 配置源头。

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

## 参考入口

- [DOC/电路信息.md](./DOC/%E7%94%B5%E8%B7%AF%E4%BF%A1%E6%81%AF.md)
- [DOC/CubeMX配置.md](./DOC/CubeMX%E9%85%8D%E7%BD%AE.md)
- [DOC/MCP2518FD驱动.md](./DOC/MCP2518FD%E9%A9%B1%E5%8A%A8.md)
- [DOC/todo.md](./DOC/todo.md)
- [REFERENCE/旧主控 CAN 通讯协议.md](./REFERENCE/%E6%97%A7%E4%B8%BB%E6%8E%A7%20CAN%20%E9%80%9A%E8%AE%AF%E5%8D%8F%E8%AE%AE.md)
- [REFERENCE/新主控 CAN 通讯协议.md](./REFERENCE/%E6%96%B0%E4%B8%BB%E6%8E%A7%20CAN%20%E9%80%9A%E8%AE%AF%E5%8D%8F%E8%AE%AE.md)
- [REFERENCE/protobuf-master/README.md](./REFERENCE/protobuf-master/README.md)
