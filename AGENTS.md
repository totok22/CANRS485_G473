# AGENTS.md

## 代码边界

- `Core/Src/app.c`、`Core/Inc/app.h` 和 `Core/Src/app_*.c`、`Core/Inc/app_*.h` 是业务代码：CAN 解析、状态聚合、protobuf 编码、RS485 发送及其内部辅助模块。
- `main.c`、`fdcan.c`、`usart.c`、`gpio.c`、`spi.c`、`stm32g4xx_it.c` 是 CubeMX 生成文件；除同步修改 `.ioc` 外，只改 USER CODE 区。
- `Middlewares/Third_Party/nanopb/` 是生成/第三方代码；改 protobuf 时必须从 `.proto/.options` 重新生成。
- `Drivers/`、启动文件和链接脚本默认不改。

## 关键映射

- 旧 F405 `CAN1` 业务 -> G473 `FDCAN3 / CAN1`。
- 旧 F405 `CAN2` 业务 -> G473 `FDCAN2 / CANB`。
- `FDCAN1 / CANA` 当前不套用旧 `CAN2` 解析。
- `MCP2518FD / CANC` 需要独立 SPI 驱动，不要当成 STM32 片内 FDCAN。
- RS485 使用 `USART2` 硬件 DE，不要恢复旧 `RS485_DIR` GPIO。

## 协议约束

- 旧项目 `/Users/poli/STM32CubeIDE/workspace_2.1.1/CAN2RS485` 是当前业务行为基准。
- CAN 协议判断优先看 `REFERENCE/旧主控 CAN 通讯协议.md`、`REFERENCE/新主控 CAN 通讯协议.md`。
- protobuf/服务器协议优先看 `REFERENCE/protobuf-master/`。
- 对 STM32G4 HAL、启动流程、外设寄存器或 CubeMX 生成语义不确定时，不要猜；优先查官方源码 `/Users/poli/stm32g4/STM32CubeG4-master`，再决定实现。
- 不要随意改 CAN ID、字节序、单位、鲜度窗口、100 ms/500 ms 上报节奏或 `fsae_TelemetryFrame` 布局。

## 中断与时序

- FDCAN/EXTI 回调保持短小；不要在中断里做 protobuf 编码、RS485 发送、SPI 轮询或阻塞等待。
- `PB4` 用作 `FDCAN3_TX`，启动路径必须保留 `HAL_PWREx_DisableUCPDDeadBattery()`。
