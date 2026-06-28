# AGENTS.md

## Goal

This project targets `STM32G473RCT6` and is the G4/FDCAN successor to the older `CAN2RS485` firmware. The hardware goal is four CAN inputs and one RS485 telemetry output:

- `CANA`: internal `FDCAN1`, `PA11/PA12`, 500 kbps.
- `CANB`: internal `FDCAN2`, `PB12/PB13`, 500 kbps.
- `CAN1`: internal `FDCAN3`, `PB3/PB4`, 250 kbps. This is the closest replacement for the old F405 `CAN1` input.
- `CANC`: external `MCP2518FD` over `SPI1`, `PA4/PA5/PA6/PA7` plus `PC4` interrupt, 500 kbps.
- `RS485`: `USART2` on `PA2/PA3`, hardware DE on `PA1`, 115200 8N1.

Keep the bring-up path stable before porting higher-level telemetry parsing.

## Sources Of Truth

- CubeMX hardware configuration lives in `CANRS485_G473.ioc`.
- CubeMX generated code is under `Core/Src`, `Core/Inc`, `Drivers`, and `cmake/stm32cubemx`.
- User business code starts in `Core/Src/app.c` and `Core/Inc/app.h`.
- The old F405 firmware is in `../CAN2RS485`; use it for protocol behavior, not for direct HAL API copying.
- G473 HAL and examples are available locally under `/Users/poli/stm32g4/STM32CubeG4-master`. Check these before guessing G4/FDCAN details.
- The hardware notes the user referenced are:
  - `/Users/poli/ObsianVault/ObsidianVault/车队/遥测系统/电路方案 2.md`
  - `/Users/poli/ObsianVault/ObsidianVault/车队/cubeide 配置/G473.md`

## Generated-Code Rules

- Put edits to CubeMX generated files only inside `/* USER CODE BEGIN ... */` blocks unless the matching `.ioc` value is changed at the same time.
- Do not add unsupported `FDCAN_InitTypeDef` fields such as message RAM offsets unless this exact HAL header contains them.
- Keep `SystemClock_Config()` on 160 MHz SYSCLK and 40 MHz FDCAN kernel clock unless the `.ioc` is intentionally changed.
- Keep `HAL_PWREx_DisableUCPDDeadBattery()` in the startup path because `PB4` is used as `FDCAN3_TX`.

## Firmware Rules

- FDCAN callbacks must stay short: drain a bounded number of FIFO frames, update state, and return.
- Do not do protobuf encoding, RS485 transmit, SPI register polling, or blocking waits from CAN or EXTI interrupt context.
- `CANA`/`CANB`/`CANC` physical responsibilities are not finalized in code yet. Ask before mapping legacy `CAN2` protocol behavior to a specific 500 kbps bus.
- `CANC` is an external MCP2518FD controller. Do not treat it as another STM32 FDCAN instance; it needs a separate driver and bring-up sequence.
- RS485 direction is controlled by USART2 hardware DE on `PA1`. Do not reintroduce the F405 `RS485_DIR` GPIO timing model.
- `PC8` LED is high-level on.

## Protocol Migration Notes

- The old telemetry target is still `fsae_TelemetryFrame` from nanopb unless a protocol change is explicitly requested.
- The old F405 app uses bxCAN APIs and filter banks; none of that maps directly to G4 FDCAN.
- The old `CAN1` 250 kbps parser should be migrated to `FDCAN3` only after its receive path is verified.
- Preserve old units and freshness semantics when moving parsing logic: `mv`, `deci_v`, `deci_c`, `ma`, 100 ms base telemetry, 500 ms module detail, and 2000 ms freshness windows.

## Build

Use the CMake presets from the repository root:

```sh
cmake --preset Debug
cmake --build --preset Debug
```

Do not commit generated build output.
