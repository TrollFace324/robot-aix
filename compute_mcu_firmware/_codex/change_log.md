# Change Log (compute_mcu_firmware)

## 2026-04-11
- Switched back to non-standard select wiring mode: PRIZM D2 -> compute PB0(D8) as frame-sync (`kSpiUseFrameSync=true`, `kSpiFrameSyncPin=8`), with local force-low on AVR SS(D10).
- Switched back to standard external SS mode (`kSpiUseExternalSs=true`) and disabled frame-sync path; expected wiring is PRIZM D2(CS) -> compute D10(SS).
- Updated compute frame-sync/select pin to `PD5/D5` per latest wiring change.
- Added external frame-sync support for non-standard wiring (`kSpiUseFrameSync`, PB0/D8): compute aligns SPI frame indices to sync line assertion.
- Moved compute status LED from D8 to D9 to avoid conflict with frame-sync line PB0/D8.
- Reverted compute transport config to 5-wire no-SS mode and enabled local SS force-low fallback (`kSpiUseExternalSs=false`, `kForceSsOutputLowInNoSsMode=true`).
- Switched compute SPI to explicit external SS mode (`kSpiUseExternalSs=true`), disabled no-SS fallback in production config.
- Added library-assisted SPI slave IRQ enable (`SPI.attachInterrupt`) while keeping AVR slave mode explicit.
- Added periodic SPI debug beacon (`TEST_FLAG`) from compute, independent of handshake, to validate MISO path on PRIZM logs.
- Added no-SS fallback option `kForceSsOutputLowInNoSsMode` for bench setups without an external SS wire.
- Removed incompatible mbed-only `spislave` package from `platformio.ini`; compute now relies on Arduino `SPI` library (`SPI.attachInterrupt`) plus project transport wrapper.
- Fixed SPI slave SS handling: in no-external-SS mode SS is now configured as input (pull-up disabled), with explicit requirement that D10 must be held LOW by hardware.
- Added compute LED diagnostic for inactive SS (4-pulse pattern) to quickly detect when AVR SPI slave is not selected.
- Added SPI protocol decoder auto-resynchronization (SOF+CRC scan across all offsets) to correctly decode rotated 32-byte frames in no-external-SS mode.
- Added initial complete firmware source tree.
- Implemented Stage 1 transport/protocol/self-check control flow.
- Added SPI slave and I2C stepper client backends.
- Added Stage 2 and Stage 3 placeholder modules for future expansion.
- Enabled `WIRE_TIMEOUT` and added runtime I2C timeout recovery path.
- Added compute boot serial trace and LED status heartbeat patterns for field diagnostics.
- Reverted compute status LED pin to `D8` and added guard against SPI pin overlap (`D10..D13`).
- Added per-loop cap for processed SPI packets to avoid control-loop starvation under heavy bus traffic.
- Added no-external-SS mode for SPI slave (`kSpiUseExternalSs=false`, SS forced low locally).
- Disabled boot serial trace by default and added LED pulse diagnostics for no-UART operation.
- Risk: transport timing and IMU pipeline require bench verification.
