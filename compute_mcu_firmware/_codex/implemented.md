# Implemented (compute_mcu_firmware)

- [DONE] Shared protocol types and codec (SPI + I2C frame encoding/decoding + CRC).
- [DONE] SPI decoder auto-resync for rotated 32-byte frames in no-external-SS wiring (`src/protocol/protocol_codec.cpp`).
- [DONE] SPI slave transport backend with ISR byte pump and loop decode.
- [DONE] Library-assisted SPI interrupt setup (`SPI.attachInterrupt`) and periodic debug beacon over SPI (`src/transport/spi_slave_backend.cpp`, `src/app/compute_controller.cpp`).
- [DONE] Correct AVR SS handling in SPI slave no-SS mode + SS-inactive LED diagnostic (`src/transport/spi_slave_backend.cpp`, `src/app/compute_controller.cpp`).
- [DONE] I2C stepper client (command + status request).
- [DONE] Self-check runner (IMU + stepper online + startup readiness).
- [DONE] Compute protocol router and responses to Stage 1 mandatory messages.
- [DONE] Compute boot trace + status LED heartbeat (`src/app/compute_controller.cpp`).
- [DONE] I2C timeout protection and recovery (`WIRE_TIMEOUT`, `Wire.setWireTimeout`).
- [DONE] UART-free LED diagnostic mode with pulse codes for SPI/link/readiness states.
- [PARTIAL] MPU6500 service only provides startup WHO_AM_I health check.
- [PARTIAL] Stage 2 autonomous core and Stage 3 station layer are stubs.
