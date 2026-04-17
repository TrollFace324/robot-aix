# PRIZM Firmware

Stage-gated PRIZM controller firmware for 3-MCU architecture.

## Current scope
- Stage 1 implemented: SPI transport, startup self-check gate, run lock.
- Stage 2/3 scaffolding present: mode manager, heading hold, mission runner, HC12 stub.

## Build
- `pio run`

## Notes
- Uses reused adapters from `LigaRobotMainBoard` under `lib/legacy_liga_adapter`.
- CRSF runtime baud configured to `115200` in system controller.
