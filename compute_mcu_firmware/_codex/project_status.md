# Project Status (compute_mcu_firmware)

- Active stage: Stage 1 implementation.
- Stable now:
  - SPI slave transport with minimal ISR and main-loop packet processing.
  - Compute protocol router for HELLO/STATUS/CAPABILITIES/HEARTBEAT/ECHO.
  - Self-check aggregation (IMU + stepper link) and READY/RUN_LOCK signaling.
  - I2C stepper client transport with CRC8 frame handling.
- In progress:
  - Bench validation of SPI slave timing and duplicate/stale packet strategy under load.
  - IMU full data path beyond startup health check.
- Blocking issues:
  - Exact board-level pinout and real MPU6500 orientation constants pending.
