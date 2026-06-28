# CANRS485_G473

STM32G473RCT6 telemetry gateway firmware derived from the older `CAN2RS485`
STM32F405 project. The target board has three internal FDCAN buses, one external
MCP2518FD CAN controller, and one USART2 RS485 telemetry output.

## Current Status

- Builds with the CMake `Debug` preset.
- Old F405 `CAN1` 250 kbps business parser is mapped to `FDCAN3`.
- Old F405 `CAN2` 500 kbps business parser is mapped to `FDCAN2` / `CANB`.
- Telemetry protobuf encoding still uses the existing `fsae_TelemetryFrame`
  nanopb schema.
- RS485 output uses `USART2` with hardware DE on `PA1`.
- `CANA` / `FDCAN1` is initialized and accepts frames, but no business protocol
  is attached to it yet.
- `CANC` / MCP2518FD hardware pins are configured, but the MCP2518FD driver is
  not implemented yet.

## Hardware Mapping

| Bus | Controller | Pins | Bitrate | Firmware role |
| --- | --- | --- | --- | --- |
| CANA | FDCAN1 | PA11 / PA12 | 500 kbps | Bring-up only for now |
| CANB | FDCAN2 | PB12 / PB13 | 500 kbps | Successor of old F405 CAN2 |
| CAN1 | FDCAN3 | PB3 / PB4 | 250 kbps | Successor of old F405 CAN1 |
| CANC | MCP2518FD over SPI1 | PA4 / PA5 / PA6 / PA7 + PC4 INT | 500 kbps | Not implemented |
| RS485 | USART2 hardware DE | PA1 / PA2 / PA3 | 115200 8N1 | Telemetry output |
| LED0 | GPIO | PC8 | n/a | High level on |

## Build

```sh
cmake --preset Debug
cmake --build --preset Debug
```

For a clean check:

```sh
cmake --build --preset Debug --clean-first
```

The build currently produces `CANRS485_G473.elf` under `build/Debug/`.

## Firmware Architecture

- `Core/Src/app.c` contains business logic:
  - FDCAN filter setup and RX callback routing.
  - Old CAN1 and CAN2 protocol parsing.
  - Telemetry snapshot freshness handling.
  - nanopb encoding.
  - USART2 RS485 transmit path.
- CubeMX generated files should only be edited inside USER CODE blocks unless
  the matching `.ioc` setting is changed.
- `Middlewares/Third_Party/nanopb/` is copied from the old F405 project to keep
  protobuf compatibility.

## Known Issues

1. MCP2518FD / CANC is not usable yet.
   The SPI pins, CS, and INT pin are configured, but there is no register driver,
   oscillator check, bit timing setup, RX FIFO handling, or bus error handling.

2. CANA has no assigned protocol.
   FDCAN1 starts and receives, but `App_ProcessCanRx()` intentionally ignores it
   until its physical role is confirmed.

3. Hardware bring-up is not verified.
   The current validation is compile-time only. SWD, CH340X auto-boot, FDCAN
   bus timing, RS485 DE timing, and LED polarity still need board-level checks.

4. FDCAN error handling is minimal.
   The app starts the buses and handles RX, but does not yet record protocol
   status, error counters, bus-off recovery, or TX failure telemetry.

5. RS485 command RX is paused during telemetry TX.
   This follows the old firmware behavior and avoids half-duplex contention, but
   it should be checked with the DTU and real traffic rate.

6. Generated CubeMX settings need discipline.
   Re-generating from CubeMX is allowed, but any change to FDCAN filter counts,
   USART2 RS485 mode, or PB4/UCPD handling must be reviewed before committing.

## Can Be Implemented Now

- Add FDCAN error/status counters and expose them through the debug CLI.
- Add a lightweight CANA raw-frame counter so CANA bring-up can be observed
  before its business protocol is known.
- Add a minimal MCP2518FD SPI register read smoke test, once the exact register
  constants are taken from the Microchip datasheet or driver source.
- Add `objcopy` post-build generation for `.hex` and `.bin`, matching the old
  project.
- Add debug CLI commands for G473 bus mapping, FDCAN error counters, and recent
  CANA/CANB/CAN1 frames.
- Add a flash task or script for STM32CubeProgrammer after the preferred probe
  and interface are confirmed.

## Needs Hardware Confirmation First

- CH340X DTR/RTS polarity and BOOT0 option-byte behavior.
- RS485 DE assertion/deassertion timing on PA1.
- CAN transceiver wiring and termination on CANA, CANB, CAN1, and CANC.
- MCP2518FD SPI mode, crystal startup, INT polarity, and register access.
- Which real vehicle subsystem should connect to CANA and CANC.

## Bring-Up Order

1. Flash over SWD and confirm PC8 LED behavior.
2. Confirm USART1 logging or bootloader access through CH340X.
3. Confirm USART2 RS485 waveform and DE timing on PA1.
4. Confirm FDCAN3 receives old 250 kbps CAN1 traffic.
5. Confirm FDCAN2 receives old 500 kbps CAN2 traffic and can transmit mode
   commands.
6. Confirm FDCAN1 physical receive path with a CAN analyzer.
7. Bring up MCP2518FD by reading registers over SPI before enabling CANC traffic.

## Repository Notes

- Remote: `https://github.com/totok22/CANRS485_G473.git`
- Main branch: `main`
- Do not commit `build/` output.
