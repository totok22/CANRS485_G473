# AGENTS.md

## 项目目标

本项目运行在 `STM32G473RCT6` 上，是旧 `CAN2RS485` / STM32F405 固件的 G473/FDCAN 版本。
目标链路是：

`CAN -> fsae_TelemetryFrame -> USART2/RS485 -> DTU`

代理在本仓库内工作时，必须优先保持旧项目已经验证过的解析、编码和上报语义稳定。不要为了
“看起来更像 G473”而重写协议、单位、字节序、鲜度窗口或 protobuf 布局。

## 代码分区

- `Core/Src/app.c`、`Core/Inc/app.h` 是业务主区。CAN 解析、状态聚合、protobuf 组帧、
  RS485 发送逻辑优先放这里。
- `Core/Src/main.c`、`Core/Src/fdcan.c`、`Core/Src/usart.c`、`Core/Src/gpio.c`、
  `Core/Src/spi.c`、`Core/Src/stm32g4xx_it.c` 是 CubeMX 生成文件。
- `CANRS485_G473.ioc` 是硬件配置源头。外设、引脚、时钟、NVIC、波特率以它为准。
- `Middlewares/Third_Party/nanopb/` 来自旧 F405 工程。除非明确在更新 `.proto` 或重新生成
  nanopb，否则不要手改生成文件。
- `REFERENCE/protobuf-master/` 是 protobuf、服务器配置、仿真脚本和协议说明的源头资料。
- `REFERENCE/旧主控 CAN 通讯协议.md`、`REFERENCE/新主控 CAN 通讯协议.md` 是判断 CAN
  解析是否正确的优先依据。
- `Drivers/` 与 `startup_stm32g473xx.s` 默认不改。

## 生成代码规则

- 修改 CubeMX 生成文件时，只允许写在 `/* USER CODE BEGIN ... */` 和
  `/* USER CODE END ... */` 区块内，除非同步修改 `.ioc`。
- 不要随意给 `FDCAN_InitTypeDef` 增加网上常见但本 HAL 头文件没有的字段，例如
  `MessageRAMOffset`。
- 保持当前 `160 MHz` SYSCLK 和 `40 MHz` FDCAN kernel clock，除非明确调整 `.ioc`。
- `PB4` 用作 `FDCAN3_TX`，启动路径中必须保留 `HAL_PWREx_DisableUCPDDeadBattery()`。

## 当前硬件映射

- `CANA`：`FDCAN1`，`PA11/PA12`，`500 kbit/s`。当前只初始化，不套旧 `CAN2` 业务解析。
- `CANB`：`FDCAN2`，`PB12/PB13`，`500 kbit/s`。这是旧 F405 `CAN2` 的确认替代口。
- `CAN1`：`FDCAN3`，`PB3/PB4`，`250 kbit/s`。这是旧 F405 `CAN1` 的确认替代口。
- `CANC`：`MCP2518FD + SPI1`，`PA4/PA5/PA6/PA7 + PC4 INT`，驱动尚未实现。
- `RS485`：`USART2`，`PA1=DE`，`PA2=TX`，`PA3=RX`。使用硬件 DE，不要恢复旧 `RS485_DIR` GPIO。
- `LED0`：`PC8`，高电平点亮。

## 业务规则

- `CAN1/FDCAN3` 走旧 F405 `CAN1` 解析逻辑。
- `CANB/FDCAN2` 走旧 F405 `CAN2` 解析逻辑，包括模式命令发送和 IMU 转发队列。
- `CANA/FDCAN1` 当前不做业务解析，除非用户明确给出其协议职责。
- `CANC/MCP2518FD` 不是 STM32 片内 FDCAN，不要把它当 `hfdcan4` 处理。
- CAN/FDCAN 中断必须短小，只取帧、转换头、调用轻量解析；不要在中断里做 protobuf 编码、
  RS485 发送、SPI 轮询或阻塞等待。
- 当前有效业务数据来自 `CAN1/FDCAN3` 或 `CANB/FDCAN2` 任一路后，允许周期上报。
- 基础帧周期 `100 ms`，带 `modules` 的低频帧周期 `500 ms`，鲜度窗口 `2000 ms`。
- 单位命名沿用旧工程：`mv`、`deci_v`、`deci_c`、`ma` 等，不要混用。

## Protobuf / nanopb 规则

- 遥测编码目标是 `fsae_TelemetryFrame`。
- 修改 protobuf 结构前必须先看 `REFERENCE/protobuf-master/`，尤其是：
  - `server_config/protos/fsae_telemetry.proto`
  - `server_config/protos/fsae_telemetry.options`
  - `server_config/telegraf/telegraf.conf`
  - `PROTO_GUIDE.md`
  - `STM32_GUIDE.md`
- 修改 `.proto` 后必须同步生成 STM32 nanopb 代码，并确认云端消费者兼容。
- `APP_MODULE_COUNT = 6`、`APP_CELLS_PER_MODULE = 23`、`APP_TEMPS_PER_MODULE = 8` 与
  nanopb 生成布局强相关，不能只改一边。

## 构建与验证

首选 CMake 预设：

```sh
cmake --preset Debug
cmake --build --preset Debug
```

重新检查近期改动时使用：

```sh
cmake --build --preset Debug --clean-first
```

构建后应生成 `.elf`、`.hex`、`.bin`。不要提交 `build/` 目录。

## 提交前自检

- 是否改变了 CAN ID、字节序、单位或鲜度语义。
- 是否把旧 F405 `CAN2` 行为错误迁到 `CANA` 或 `CANC`。
- 是否引入阻塞到 CAN/FDCAN/EXTI 中断上下文。
- 是否破坏了 USART2 硬件 DE 的半双工发送语义。
- 是否需要同步更新 `README.md`、`DOC/` 或 `REFERENCE/protobuf-master/`。
