# CubeMX 配置

本文件汇总当前 G473 工程需要保持一致的 CubeMX/固件配置，是本仓库内的配置入口。

## 工程

- MCU：`STM32G473RCTx`。
- Toolchain：CMake。
- 代码生成：外设初始化拆分为 `.c/.h`。
- 保留 USER CODE：开启。

## RCC / SYS

- HSE：Crystal/Ceramic Resonator，`8 MHz`。
- Clock Security System：开启。
- SYS Debug：Serial Wire。
- Timebase：SysTick。
- 系统时钟：`160 MHz`。
- FDCAN kernel clock：`40 MHz`，来源 PLLQ。
- 当前 PLL：
  - `PLLM = 2`
  - `PLLN = 80`
  - `PLLR = 2`
  - `PLLQ = 8`

## GPIO

| 引脚 | 配置 | Label | 初始/参数 |
| --- | --- | --- | --- |
| PC8 | GPIO_Output | LED0 | Low，Push-Pull，Low Speed |
| PA4 | GPIO_Output | MCP2518_CS | High，Push-Pull，High Speed |
| PC4 | GPIO_EXTI4 | MCP2518_INT | Falling edge，No Pull |
| PB8 | GPIO_Input | BOOT0_IN | No Pull |

## USART

### USART1

- 用途：CH340X 串口 / 应用层可选调试 CLI。
- 引脚：`PA9=TX`，`PA10=RX`。
- 参数：`115200 8N1`。
- 硬件流控：关闭。
- NVIC：`USART1_IRQn` 优先级 4。

### USART2 / RS485

- 用途：DTU RS485。
- 引脚：`PA1=DE`，`PA2=TX`，`PA3=RX`。
- 参数：`115200 8N1`。
- 模式：`HAL_RS485Ex_Init()`，DE 高有效。
- DE assertion/deassertion：当前为 `0/0`。
- NVIC：`USART2_IRQn` 优先级 3。
- 业务发送函数仍等待 `UART_FLAG_TC`，用于保持“最后一个停止位已发完”的旧工程语义。

## FDCAN

三路 FDCAN 都使用 Classic CAN、Normal mode、Tx FIFO operation。

| 控制器 | 总线名 | 速率 | Prescaler | Seg1 | Seg2 | SJW | StdFilters | ExtFilters | NVIC |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| FDCAN1 | CANA | 500 kbit/s | 5 | 13 | 2 | 2 | 1 | 1 | 优先级 2 |
| FDCAN2 | CANB | 500 kbit/s | 5 | 13 | 2 | 2 | 1 | 1 | 优先级 2 |
| FDCAN3 | CAN1 | 250 kbit/s | 10 | 13 | 2 | 2 | 1 | 1 | 优先级 2 |

计算：

- 500 kbit/s：`40 MHz / 5 / (1 + 13 + 2)`。
- 250 kbit/s：`40 MHz / 10 / (1 + 13 + 2)`。
- 采样点：`(1 + 13) / (1 + 13 + 2) = 87.5%`。

应用层在 `App_FDCAN_ConfigFilter()` 中配置标准帧/扩展帧全通，并启动 FIFO0 新消息通知。

## SPI1 / MCP2518FD

- Mode：Full-Duplex Master。
- NSS：Software。
- Data Size：8 Bits。
- First Bit：MSB First。
- CPOL/CPHA：Mode 0。
- Prescaler：32，当前约 5 MHz 级别。
- 引脚：
  - `PA5=SPI1_SCK`
  - `PA6=SPI1_MISO`
  - `PA7=SPI1_MOSI`
- `PA4` 不交给 SPI NSS，由 GPIO `MCP2518_CS` 控制。

## NVIC

- Priority Group：`NVIC_PRIORITYGROUP_4`。
- `EXTI4_IRQn`：优先级 1。
- `FDCAN1_IT0_IRQn`：优先级 2。
- `FDCAN2_IT0_IRQn`：优先级 2。
- `FDCAN3_IT0_IRQn`：优先级 2。
- `USART2_IRQn`：优先级 3。
- `USART1_IRQn`：优先级 4。

## USER CODE

`main()` 中 `SystemClock_Config()` 之后、外设初始化之前保留：

```c
HAL_PWREx_DisableUCPDDeadBattery();
```

`MX_USART2_UART_Init()` 后调用：

```c
App_Init();
```

主循环调用：

```c
App_Run();
```

## 不要照搬的旧 F405 配置

- bxCAN 的 filter bank、`SlaveStartFilterBank` 不适用于 G4 FDCAN。
- `CAN_HandleTypeDef`、`CAN_RxHeaderTypeDef`、`HAL_CAN_*` 不适用于当前业务入口。
- `RS485_DIR` GPIO 不适用于当前硬件；方向由 USART2 DE 控制。
- `PC13` LED 不适用于当前硬件；状态灯为 `PC8 LED0`。
