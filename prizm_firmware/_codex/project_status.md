# Project Status (prizm_firmware)

- Active stage: Stage 1 (transport + startup safety) with Stage 2/3 scaffolding.
- Stable now:
  - SPI master transport frame exchange (32-byte fixed frame).
  - Startup state machine: BOOT -> INIT -> LINK_CHECK -> SENSOR_CHECK -> READY/FAULT_LOCK.
  - Run-lock guard blocks wheel + M1/M2 while not READY.
  - Reused RC/drive/motors modules integrated as adapter base.
- In progress:
  - Bench-level validation against real compute MCU SPI slave timing.
  - Live SPI diagnostics over UART on PRIZM to isolate `RUN_LOCKED` root cause from compute self-check payload.
  - Stage 2 control behavior tuning (heading hold + mission runner wiring).
- Blocking issues:
  - Exact production pin map and mechanical constants are still defaulted in config.
