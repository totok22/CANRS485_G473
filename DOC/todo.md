# 待办与迁移状态

## 已完成

- [x] 使用 CubeMX 生成 `STM32G473RCT6` CMake 工程。
- [x] 配置三路片内 FDCAN：
  - `FDCAN1 / CANA`：500 kbit/s。
  - `FDCAN2 / CANB`：500 kbit/s。
  - `FDCAN3 / CAN1`：250 kbit/s。
- [x] 配置 `USART2` 硬件 RS485 DE：`PA1=DE`，`PA2=TX`，`PA3=RX`。
- [x] 配置 `SPI1 + PA4 CS + PC4 EXTI`，预留给 MCP2518FD。
- [x] 从旧 F405 工程迁移 CAN 解析、状态聚合、protobuf 编码和 RS485 上报业务。
- [x] 将旧 F405 `CAN1` 业务映射到 `FDCAN3 / CAN1`。
- [x] 将旧 F405 `CAN2` 业务映射到 `FDCAN2 / CANB`。
- [x] 迁移 nanopb 和 `fsae_TelemetryFrame` 生成代码。
- [x] 迁移 `REFERENCE/protobuf-master`、新旧主控协议、DBC、IVT/FS 资料。
- [x] 构建输出 `.elf`、`.hex`、`.bin`。

## 当前可以继续实现

- [ ] 增加 FDCAN 错误计数、协议状态、bus-off 状态和发送失败统计。
- [ ] 改造调试 CLI，使 `status`、`can`、`ids` 输出显示 `CANA/CANB/CAN1`。
- [ ] 给 `CANA/FDCAN1` 增加原始帧计数和最近帧记录，但暂不做业务解析。
- [ ] 实现 MCP2518FD 基础 SPI 读寄存器函数和寄存器通信检查。
- [ ] 为 CANB 模式命令发送增加 FDCAN TX 失败计数。
- [ ] 将 G473 的总线映射和协议迁移关系写入更细的开发文档。

## 需要明确协议或硬件职责后再做

- [ ] 明确 `CANA/FDCAN1` 接入哪个车上子系统，再决定是否复用现有解析函数。
- [ ] 明确 `CANC/MCP2518FD` 接入哪个车上子系统，再决定其业务解析和发送策略。
- [ ] 若未来要完整支持新主控工具链，继续实现 `0x18A*` / `0x188350F5` 等命令语义。
- [ ] 若未来要把更多诊断字段上云，同步修改 `.proto`、nanopb 生成代码和云端 `telegraf.conf`。
