# Change Log (prizm_firmware)

## 2026-04-11
- Updated SPI warning text to current non-standard select wiring: `PRIZM D2(CS) -> compute PB0(D8)`.
- Updated SPI warning text to standard SS wiring: `PRIZM D2(CS) -> compute D10(SS)`.
- Updated select-line diagnostic text to current wiring: `PRIZM D2 -> compute PD5(D5)`.
- Updated PRIZM select wiring config to `D2` (`kSpiUseExternalCs=true`, `kSpiCsPin=2`) to match field hardware.
- Reverted PRIZM transport config to 5-wire no-CS mode (`kSpiUseExternalCs=false`) per hardware limitation.
- Switched PRIZM external SPI CS from `D4` to `D10` to align slave-select with hardware SS line and remove board-specific `D4` uncertainty.
- Switched PRIZM SPI to explicit CS mode (`kSpiUseExternalCs=true`) and added boot trace line with active CS mode/pin.
- Updated SPI loopback warning text to explicit wiring requirement: `PRIZM D10 -> compute D10 (SS)`.
- Updated SPI master bring-up: HW SS (`D10`) is now always driven HIGH and SPI clock reduced to 250 kHz for bench stability.
- Added explicit SPI loopback detection counter and warning in PRIZM UART diagnostics (`loop=` + warning when PRIZM receives its own frames as RX).
- Added UART trace for dropped SPI frames with unexpected source (`src/dst/type/seq`) and included `stale_packets` in periodic SPI health log to diagnose loopback/wiring issues.
- Added SPI frame auto-resynchronization in protocol decoder (searches SOF+CRC across all byte offsets) to support no-external-CS operation where 32-byte frames may be rotated.
- Added periodic `SPI LINK` UART diagnostics on PRIZM (tx/rx/crc/decode/errors + handshake flags) for cases where no valid RX packet is decoded.
- Added SPI->UART packet trace on PRIZM (`kEnableSpiUartTrace`) with decoded message names, seq/ack/flags, payload hex, and parsed `STATUS` / `STARTUP_CHECK_RESULT` / `RUN_LOCKED`.
- Added debug policy switch to disable CRSF UART while SPI trace is active (`kDisableRcWhenSpiTrace`) to avoid single-UART contention.
- Added full Stage 1 architecture scaffolding and runtime loop.
- Imported and reused RC/mecanum/motor modules from `LigaRobotMainBoard`.
- Added SPI master transport, startup FSM, run-lock safety.
- Added Stage 2/3 scaffolding modules (mode manager, heading hold, mission runner, HC12 stub).
- Added PRIZM LED status patterns for BOOT/INIT/LINK_CHECK/SENSOR_CHECK/READY/FAULT_LOCK.
- Added early boot tracing and raw LED pre-init diagnostics in `SystemController::begin()`.
- Moved active SPI CS from `D10` to `D4`; `D10` is now reserved as HW SS output HIGH.
- Added no-external-CS mode (`kSpiUseExternalCs=false`) for 4-wire SPI operation.
- In no-external-CS mode, HW SS (`D10`) is held HIGH (master-safe behavior).
- Risk: stage2/stage3 behavior requires bench tuning before production use.
