# MCP2518FD / CANC 驱动说明

本文记录当前 `CANC` 外置 CAN 控制器驱动的实现边界。硬件为
`STM32G473RCT6 + MCP2518FDT-E/SL + SN65HVD233DR`，MCP2518FD 使用独立 `40 MHz`
晶振，SPI 接 `SPI1`，`PA4` 为手动片选，`PC4` 为低有效 `INT#`。

## 当前实现

- 驱动文件：
  - `Core/Inc/mcp2518fd.h`
  - `Core/Src/mcp2518fd.c`
- SPI 指令：
  - `RESET`
  - `READ`
  - `WRITE`
- 寄存器访问：
  - 16 bit 指令头：`4 bit command + 12 bit address`
  - SFR 以小端 32 bit 辅助函数读写
  - Message RAM 只按 4 字节对齐地址、4 字节倍数长度访问
- 初始化：
  - 软件复位
  - 等待配置模式
  - `OSC=0`，使用外部 `40 MHz` 晶振，不开 PLL
  - `IOCON=0`，TXCAN/INT 推挽默认输出
  - `ECCCON.ECCEN=1`
  - `C1NBTCFG` 配置为 `40 MHz / 500 kbit/s` Classic CAN 默认时序
  - `C1CON.BRSDIS=1`，当前不启用 BRS
  - FIFO1：RX FIFO，8 byte payload，16 深度
  - FIFO2：TX FIFO，8 byte payload，8 深度
  - Filter0：掩码全 0，接收标准帧/扩展帧到 FIFO1
  - 进入 Normal CAN FD mode，但发送对象默认 `FDF=0/BRS=0`，即 Classic CAN 帧
- 应用层接入：
  - `App_Init()` 尝试初始化 MCP2518FD；失败不阻塞片内 FDCAN 与 RS485 主业务
  - `HAL_GPIO_EXTI_Callback()` 只记录 MCP2518FD 中断标志，不在中断里做 SPI
  - `App_Run()` 轮询 FIFO1，最多每轮处理 8 帧
  - CANC 原始帧进入调试记录；当前不进入 `CAN1` 或 `CANB` 业务解析

## 当前不做

- 不把 `CANC` 当作 STM32 片内 FDCAN。
- 不把 `CANC` 默认映射成旧 F405 `CAN1` 或 `CAN2`，避免在硬件职责未明确前污染业务状态。
- 不启用 CAN FD payload > 8、BRS、TEF、TXQ、时间戳或 CRC SPI 指令。
- 不在 `EXTI4_IRQHandler()` / `HAL_GPIO_EXTI_Callback()` 中执行 SPI 读写。

## 后续接业务时的入口

1. 明确 `CANC` 接入的车上子系统和 CAN ID 集合。
2. 若它应复用旧 `CAN2` 业务，在 `App_Mcp2518ServiceRx()` 中把帧分发到
   `App_ProcessCan2Rx()`；若是独立业务，应新增独立解析函数。
3. 若需要通过 `CANC` 发送命令，优先使用 `MCP2518FD_Transmit()`，保持发送对象为 Classic CAN：
   `FDF=0`、`BRS=0`、DLC 不超过 8。
4. 若启用 CAN FD，需同步调整 FIFO payload size、DLC 编解码、业务数据缓冲长度和 protobuf 字段设计。

