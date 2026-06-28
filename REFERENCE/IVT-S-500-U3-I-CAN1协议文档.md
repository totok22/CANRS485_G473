## 1. 型号对应信息
**IVT-S-500-U3-I-CAN1-12/24** 含义如下：
- **500**：额定电流量程 500 A
- **U3**：带 3 路电压测量通道
- **I**：带隔离
- **CAN1**：**有板载 CAN 终端电阻**
- **12/24**：典型 12V / 24V 供电
结合设备识别响应 `DEVICE_ID`，该型号在协议上的固定特征应为：
- Device-type = **IVT-S**
- I-nominal = **500 A**
- Voltage channels = **3**
- Feature = **I（Isolation）**
- Communication type = **CAN1（with termination）**
- Nominal supply = **12/24V**

## 2. 物理层 / 总线参数

### 2.1 CAN 类型

- **CAN 2.0A**
- 标准帧，**11-bit CAN ID**
### 2.2 波特率

可选：
- **250 kbit/s**
- **500 kbit/s（默认）**
- **1 Mbit/s**

## 3. 默认 CAN ID 与报文总览

### 3.1 默认 CAN ID

|报文名|默认 CAN ID|DLC|说明|
|---|--:|--:|---|
|IVT_Msg_Command|0x411|8|命令报文，SET/GET|
|IVT_Msg_Debug|0x510|8|内部使用|
|IVT_Msg_Response|0x511|8|对 SET/GET 的响应|
|IVT_Msg_Result_I|0x521|6|电流结果|
|IVT_Msg_Result_U1|0x522|6|电压 1 结果|
|IVT_Msg_Result_U2|0x523|6|电压 2 结果|
|IVT_Msg_Result_U3|0x524|6|电压 3 结果|
|IVT_Msg_Result_T|0x525|6|温度结果|
|IVT_Msg_Result_W|0x526|6|功率结果|
|IVT_Msg_Result_As|0x527|6|电量计数 As|
|IVT_Msg_Result_Wh|0x528|6|能量计数 Wh|

### 3.2 字节序默认值

- 默认：**Big Endian**
- 可配置为 Little Endian

### 3.3 命令规则

- 未定义字节在**命令报文中必须填 0x00**
- 响应报文中未用字节通常返回 **0x00**
- 连续命令发送间隔不得快于 **2 ms**，或者等待对应响应后再发
- 一般响应最迟应在对应命令后的 **500 ms** 内出现在总线上；特殊命令另有说明

## 4. 多路复用 MUX 规则

所有报文通过 **DB0（Muxbyte）** 区分类型。  
高 4 bit 表示大类，低 4 bit 表示具体子类型或通道。规则如下：

| DB0 范围 | 含义                           |
| ------ | ---------------------------- |
| 0x0n   | 结果报文                         |
| 0x1n   | Set CAN ID                   |
| 0x2n   | Set config result            |
| 0x3n   | Set commands                 |
| 0x4n   | Get error/log data           |
| 0x5n   | Get CAN ID                   |
| 0x6n   | Get config result            |
| 0x7n   | Get commands                 |
| 0x8n   | Response on error/log data   |
| 0x9n   | Response on CAN ID           |
| 0xAn   | Response on config result    |
| 0xBn   | Response on set/get commands |
| 0xFF   | 非法/不允许命令响应                   |

## 5. 默认输出配置（与这款 U3 型号直接相关）

默认测量/输出配置如下：

| 通道 n | 信号       | 默认模式    |   默认周期 | 最小周期 | 说明           |
| ---: | -------- | ------- | -----: | ---: | ------------ |
|    0 | Current  | Cyclic  |  20 ms | 1 ms | 输出周期=测量周期    |
|    1 | U1       | Cyclic  |  60 ms | 3 ms | 与 U1~U3 配置相关 |
|    2 | U2       | Cyclic  |  60 ms | 3 ms | 与 U1~U3 配置相关 |
|    3 | U3       | Cyclic  |  60 ms | 3 ms | 与 U1~U3 配置相关 |
|    4 | T        | Disable | 100 ms | 1 ms | 固定 100ms     |
|    5 | P(WU1)   | Disable |  30 ms | 1 ms | 功率=I×U1      |
|    6 | Q(As)    | Disable |  30 ms | 1 ms | 电流积分         |
|    7 | ΔE(WhU1) | Disable |  30 ms | 1 ms | 能量=I×U1      |
补充默认参数：
- OC Threshold(+/-)：**Off**
- OC Reset Threshold(+/-)：**Off**

## 6. 结果报文格式（最重要）

所有结果报文（0x521~0x528）数据格式相同，**DLC=6**：

| Byte    | 信号               | 说明                 |
| ------- | ---------------- | ------------------ |
| DB0     | MuxID            | 0x00~0x07，表示结果类型   |
| DB1 低4位 | IVT_MsgCount     | 每通道独立循环计数器 0x0~0xF |
| DB1 高4位 | IVT_Result_state | 状态位                |
| DB2~DB5 | IVT_Resultname   | 4 字节有符号 long 值     |

### 6.1 MuxID 定义

|MuxID|结果名|单位|
|---|---|---|
|0x00|IVT_Msg_Result_I|1 mA|
|0x01|IVT_Msg_Result_U1|1 mV|
|0x02|IVT_Msg_Result_U2|1 mV|
|0x03|IVT_Msg_Result_U3|1 mV|
|0x04|IVT_Msg_Result_T|0.1 °C|
|0x05|IVT_Msg_Result_W|1 W|
|0x06|IVT_Msg_Result_As|1 As|
|0x07|IVT_Msg_Result_Wh|1 Wh|

### 6.2 状态位定义（DB1 高 4 位）

- **bit0**：OCS 为 true（过流开关状态）
- **bit1**：该结果超规格范围 / 精度降低 / 测量错误
- **bit2**：任意结果存在 measurement-error
- **bit3**：任意结果存在 system-error，传感器功能不再保证
说明：  
- 即使未开启 measurement / system error bit mask 上报，**bit2/bit3 仍可能因 CRC 错误置位**
- **bit2** 还可能因 **Vref error** 置位

### 6.3 数据解析示例

默认大端示例：
- DB0=0x01
- DB1=0x05
- DB2~DB5=00 00 88 B8
表示：
- Result_U1
- 状态位=0
- MsgCount=5
- 数值=0x000088B8 = **35000 mV = 35.000 V**

### 6.4 数值字段类型

- `DB2~DB5` 为结果值字段
- 所有结果统一按 **signed long（32-bit 有符号整数）** 解释
- 字节序由对应通道配置决定：默认 **Big Endian**，可改为 **Little Endian**

## 7. 结合本型号的关键测量含义

对 **IVT-S-500-U3-I-CAN1-12/24**：
- 电流额定量程：**±500 A**
- 电流结果单位：**1 mA**
- 电压量程：**±1000 V**
- U1/U2/U3 结果单位：**1 mV**
- 温度结果单位：**0.1 °C**
- 功率结果：**I × U1**
- 能量 / 电量计数也都以 U1 作为功率参考电压通道

## 8. CAN ID 设置命令

### 8.1 命令格式：Set CAN ID

通过 `IVT_Msg_Command`（默认 0x411）发送，DLC=8。

|DB|含义|
|---|---|
|DB0|0x10~0x17 / 0x1D / 0x1F，指明要修改哪个报文的 CAN ID|
|DB1|目标 11-bit CAN ID 高字节|
|DB2|目标 11-bit CAN ID 低字节|
|DB3~DB6|32-bit 序列号|
|DB7|未定义，置 0|
对应关系：
- 0x10 = Result_I
- 0x11 = Result_U1
- 0x12 = Result_U2
- 0x13 = Result_U3
- 0x14 = Result_T
- 0x15 = Result_W
- 0x16 = Result_As
- 0x17 = Result_Wh
- 0x1D = Command
- 0x1F = Response
限制：
- **只能在 Stop 模式下配置**

## 9. 结果通道配置命令（模式、周期、字节序、符号）

### 9.1 命令格式：Set Config Result

DB0 = `0x2n`，其中 n 为通道号：
- 0x20 = I
- 0x21 = U1
- 0x22 = U2
- 0x23 = U3
- 0x24 = T
- 0x25 = W
- 0x26 = As
- 0x27 = Wh

|DB|含义|
|---|---|
|DB0|0x2n|
|DB1 低4位|模式|
|DB1 高4位|配置标志|
|DB2~DB3|周期/触发延时，单位 ms|

#### DB1 低4位：模式

- 0x0 = disabled
- 0x1 = triggered
- 0x2 = cyclic running

#### DB1 高4位：配置标志

- bit4：预留
- bit5：
    - 0 = 不在 `IVT_Result_state` bit2/bit3 中上报测量/系统错误掩码（默认）
    - 1 = 在 `IVT_Result_state` bit2/bit3 中上报
- bit6：
    - 0 = Big Endian（默认）
    - 1 = Little Endian
- bit7：
    - 0 = 符号不变（默认）
    - 1 = 符号翻转（+/- 互换）

#### DB2~DB3

- 输出周期 / 测量周期 / 触发延时，单位 **ms**
- `0x0000` 被忽略
- Trigger delay 从 **1 ms** 开始
限制：
- **只能在 Stop 模式下配置**
- bit5 是**全局设置**，最后一次配置决定全局行为

## 10. 运行控制与普通命令

### 10.1 Reset Error / Logdata

DB0 = **0x30**

|DB|值|说明|
|---|---|---|
|DB1|0x00|Reset Measurement Error|
|DB1|0x01|Reset System Error|
|DB1|0x02|Reset Logdata Since Reset|
|DB2|0x00|全部计数清零|
|DB2|0x01..0xFF|清指定项|
|DB3~DB6|序列号|32-bit serial|
限制：
- **仅 Stop 模式**
- 响应至少在 **+1200 ms** 后返回

### 10.2 Trigger

DB0 = **0x31**  
DB1~DB2 为通道位图：
- bit0 = I
- bit1 = U1
- bit2 = U2
- bit3 = U3
- bit4 = T
- bit5 = W
- bit6 = As
- bit7 = Wh
限制：
- **仅 Run 模式可用**

### 10.3 Store

DB0 = **0x32**
将以下配置写入非易失存储器：
- 测量配置
- 过流阈值
- 启动模式
- CAN ID
- 波特率
限制：
- 存储完成以前不允许其他命令
- 响应时间最多约 **1000 ms**
- **仅 Stop 模式**

### 10.4 START_OC_TEST

DB0 = **0x33**  
DB1~DB2 = OC 信号持续时间（ms）  
限制：**仅 Stop 模式**

### 10.5 SET_MODE

DB0 = **0x34**

|DB|值|说明|
|---|---|---|
|DB1|0x00|当前模式 Stop|
|DB1|0x01|当前模式 Run|
|DB2|0x00|上电默认 Stop|
|DB2|0x01|上电默认 Run|
|DB3~DB4|访问级别代码|预留|
说明：
- Mode 指整个传感器工作模式，不是单个结果通道
- **Stop 和 Run 均可读取**

### 10.6 过流阈值

- `0x35` = 正方向过流阈值
- `0x36` = 负方向过流阈值
格式：
- DB1~DB2 = Set threshold，单位 1A，0=off
- DB3~DB4 = Reset threshold，单位 1A，0=off
限制：
- **仅 Stop 模式**

### 10.7 重启相关

- `0x3A` = Restart to configured bitrate
    - DB1=0x08 → 250k
    - DB1=0x04 → 500k
    - DB1=0x02 → 1000k
- `0x3D` = Restart to default
- `0x3F` = Restart
说明：
- 重启命令会导致 **Reset Watchdog** 类错误复位
- `0x3A` 执行前会**自动保存当前 bitrate**，因此可能额外增加最多约 **1 s** 的重启时间
- `0x3A/0x3D` 仅 Stop 模式
- `0x3F` Stop/Run 均可

## 11. Get 类命令

### 11.1 获取测量错误 `0x40`

DB1：
- 0x00 = 获取错误位图
- 0x01~0x0F = 获取对应错误计数器
测量错误类型包括：
- ADC interrupt
- ADC channel1 overflow / underflow
- ADC channel2 overflow / underflow
- Vref error
- Current implausible I1-I2
- Thermal EMF correction
- I1 open circuit
- U1/U2/U3 open circuit
- ntc-h / ntc-l open circuit
- calibration data invalid

### 11.2 获取系统错误 `0x41`

DB1：
- 0x00 = 获取位图
- 0x01~0x10 = 获取某项错误计数
系统错误包括：
- Code CRC
- Parameter CRC
- CAN receive/transmit data
- overtemp / undertemp
- power failure
- system clock
- system init
- configuration
- internal ADC / EEPROM / ADC Clock
- Reset illegal opcode / Watchdog / EMC 等

### 11.3 获取总日志 `0x42`

### 11.4 获取复位后日志 `0x43`

可获取：
- Ah overall / charge / discharge
- Wh overall / charge / discharge
- runtime overall
- 各通道 within/outside specified limits 运行时间
- 正/负方向 overcurrent 激活时间
- I/U1/U2/U3/T 的最大最小值

### 11.5 获取 CAN ID `0x50~0x57 / 0x5D / 0x5F`

与 Set CAN ID 对应，用序列号定位设备。
### 11.6 获取配置 `0x60~0x67`

- 0x60 = I
- 0x61 = U1
- 0x62 = U2
- 0x63 = U3
- 0x64 = T
- 0x65 = W
- 0x66 = As
- 0x67 = Wh

### 11.7 其他 GET 命令

- `0x73` GET_OC_TESTTIME
- `0x74` GET_MODE
- `0x75` GET_THRESHOLD_POS
- `0x76` GET_THRESHOLD_NEG
- `0x79` GET_DEVICE_ID
- `0x7A` GET_SW_VERSION
- `0x7B` GET_SERIAL_NUMBER
- `0x7C` GET_ARTICLE_NUMBER

## 12. 响应报文定义

所有响应默认发到 **IVT_Msg_Response = 0x511**，DLC=8，DB0 为响应类型。

### 12.1 错误/日志响应

- `0x80` = measurement errors response
- `0x81` = system errors response
- `0x82` = overall logdata response
- `0x83` = logdata since reset response

### 12.2 CAN ID 响应

- `0x90~0x97 / 0x9D / 0x9F`  
    分别对应 I/U1/U2/U3/T/W/As/Wh/Command/Response 的 CAN ID 响应。

### 12.3 配置响应

- `0xA0~0xA7`  
    对应 I/U1/U2/U3/T/W/As/Wh 的配置响应。  
    格式与 Set Config Result 一致。

### 12.4 命令响应

- `0xB0` Reset Error/Logdata 响应
- `0xB1` Trigger 响应
- `0xB2` Store 响应
- `0xB3` OC Test 响应
- `0xB4` Mode 响应
- `0xB5` Threshold_Pos 响应
- `0xB6` Threshold_Neg 响应
- `0xB9` Device_ID 响应
- `0xBA` SW_VERSION 响应
- `0xBB` SERIAL_NUMBER 响应
- `0xBC` ARTICLE_NUMBER 响应

### 12.5 非法命令响应

- `DB0 = 0xFF`
- `DB1 = 无效命令的 MUX ID`

## 13. 该型号的 DEVICE_ID 应如何识别

对 **IVT-S-500-U3-I-CAN1-12/24**，`GET_DEVICE_ID (0x79)` 的响应 `0xB9` 中应表现为：

| Byte    | 含义                 | 期望值             |
| ------- | ------------------ | --------------- |
| DB1     | Device-type        | 0x02（IVT-S）     |
| DB2     | I-nominal/16       | 0x1F（500A）      |
| DB3 高4位 | I-nominal%16       | 0x4（500A）       |
| DB3 低4位 | 电压通道数              | 0x3（3 路）        |
| DB4     | Feature            | 0x03（Isolation） |
| DB5     | Communication type | 0x01（CAN1，有终端）  |
| DB6     | 供电                 | 0x01（12/24V）    |

## 14. 上电启动报文

设备上电完成内部自检后，会发送 **Alive message**：
- `DB0 = 0xBF`
- DB1~DB2 = Command ID
- DB3~DB6 = Serial number
含义：  
设备启动完成，已经可以通信。

## 15. 常用配置流程：启用 4~7 并将 0~7 设为小端

目标：
- 启用默认关闭的 `T / W / As / Wh`（通道 `4~7`）
- 将结果通道 `0~7` 统一配置为 `cyclic + little-endian`
- `STORE` 后重启验证，确保掉电保持

前提：
- 命令发到 `0x411`，响应在 `0x511`
- 全部配置命令仅能在 **Stop 模式** 下执行
- 连续命令间隔不少于 `2 ms`，或等待上一条响应后再发
- 命令帧未定义字节填 `0x00`

### 15.1 配置目标值

|通道|信号|DB0|DB1|周期|
|---:|---|---|---|---|
|0|I|`0x20`|`0x42`|`00 14` = 20 ms|
|1|U1|`0x21`|`0x42`|`00 3C` = 60 ms|
|2|U2|`0x22`|`0x42`|`00 3C` = 60 ms|
|3|U3|`0x23`|`0x42`|`00 3C` = 60 ms|
|4|T|`0x24`|`0x42`|`00 64` = 100 ms|
|5|W|`0x25`|`0x42`|`00 1E` = 30 ms|
|6|As|`0x26`|`0x42`|`00 1E` = 30 ms|
|7|Wh|`0x27`|`0x42`|`00 1E` = 30 ms|

其中 `DB1 = 0x42` 表示：
- 低 4 位 `0x2`：`cyclic running`
- `bit6 = 1`：Little Endian
- `bit5 = 0`：不额外上报 measurement/system error mask
- `bit7 = 0`：符号不翻转

### 15.2 命令顺序

1. 切到 `Stop`
   - 若只想临时进入 `Stop`，且保持上电默认仍为 `Run`：
   - `34 00 01 00 00 00 00 00`
   - 若希望上电默认也改为 `Stop`：
   - `34 00 00 00 00 00 00 00`
   - 期望响应：`DB0 = 0xB4`
2. 依次配置 8 个结果通道：

```text
20 42 00 14 00 00 00 00
21 42 00 3C 00 00 00 00
22 42 00 3C 00 00 00 00
23 42 00 3C 00 00 00 00
24 42 00 64 00 00 00 00
25 42 00 1E 00 00 00 00
26 42 00 1E 00 00 00 00
27 42 00 1E 00 00 00 00
```

3. 每条配置的期望响应：
   - `0xA0~0xA7` 分别对应 `I/U1/U2/U3/T/W/As/Wh`
4. 第一次读回验证：

```text
60 00 00 00 00 00 00 00
61 00 00 00 00 00 00 00
62 00 00 00 00 00 00 00
63 00 00 00 00 00 00 00
64 00 00 00 00 00 00 00
65 00 00 00 00 00 00 00
66 00 00 00 00 00 00 00
67 00 00 00 00 00 00 00
```

5. 读回检查：
   - 响应为 `0xA0~0xA7`
   - `DB1` 应为 `0x42`
   - 周期应分别为 `0014 / 003C / 003C / 003C / 0064 / 001E / 001E / 001E`
6. 保存到非易失存储器：
   - `32 00 00 00 00 00 00 00`
   - 期望响应：`DB0 = 0xB2`
   - 完成前不要发送其他命令，最长约 `1000 ms`
7. 重启：
   - `3F 00 00 00 00 00 00 00`
   - 重启后等待 Alive message：`DB0 = 0xBF`
8. 第二次读回验证：
   - 再次发送 `0x60~0x67`
   - 若配置仍一致，说明 `STORE` 已生效，掉电后可保持

### 15.3 结果报文验证

重启后应看到以下结果开始周期发送：
- `0x521` I
- `0x522` U1
- `0x523` U2
- `0x524` U3
- `0x525` T
- `0x526` W
- `0x527` As
- `0x528` Wh

验证要点：
- `0x525~0x528` 出现，说明默认关闭的 `4~7` 已启用
- `DB2~DB5` 需按 **Little Endian** 解释
- 例如 `U1` 收到 `DB2~DB5 = B8 88 00 00`，应解析为 `0x000088B8 = 35000 mV = 35.000 V`

### 15.4 关键说明

- 字节序配置是**按结果通道分别设置**，不是全局一次性切换
- 它只影响结果报文 `DB2~DB5` 的数值解析，不改变命令/响应报文格式
- `SET_MODE (0x34)` 的 `DB2` 会同时修改“上电默认模式”，配置时不要误写

## 16. CAN2RS485 当前接线与固件使用

当前 `CAN2RS485` 板子的 IVT-S 接在 `CANB/CAN2`（500 kbit/s），高压包电压接到 **U1**。因此固件只依赖以下 IVT 结果：
- `0x521` I：总电流，单位 `mA`
- `0x522` U1：总电压，单位 `mV`
- `0x526` W：功率，单位 `W`；若未收到则由 U1 电压和电流计算
- `0x528` Wh：能量积分，单位 `Wh`

`Wh` 在该型号协议中是 `WhU1`，所以高压电压接到 U1 后，硬件能量积分与实际总压通道一致。

PCAN 配置脚本可使用：

```bash
python3 REFERENCE/pcan_ivt_tool.py setup-can2rs485
```

该命令只配置 `I/U1/Wh` 为 `cyclic + little-endian` 并保存重启；如需调试全部通道，再使用 `setup-intel`。
