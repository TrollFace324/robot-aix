# Implemented (prizm_firmware)

- [REUSED] `RcCrsf` imported from `D:\LigaProgram\LigaRobotMainBoard\src\rc_crsf.*`.
- [REUSED] `DriveMecanum` imported from `D:\LigaProgram\LigaRobotMainBoard\src\drive_mecanum.*`.
- [REUSED] `MotorsPrizmExp` imported from `D:\LigaProgram\LigaRobotMainBoard\src\motors_prizm_exp.*`.
- [DONE] Stage 1 protocol codec (`src/protocol/protocol_codec.cpp`).
- [DONE] Stage 1 SPI master backend (`src/transport/spi_master_backend.*`).
- [DONE] Startup state machine + run lock (`src/app/startup_state_machine.*`, `src/safety/run_lock_guard.*`).
- [DONE] Compute link handshake/status client (`src/app/compute_link_client.*`).
- [DONE] PRIZM SPI RX UART diagnostics with decoded `STATUS` and `STARTUP_CHECK_RESULT` payloads (`src/app/compute_link_client.cpp`).
- [DONE] PRIZM periodic SPI health UART diagnostics (`tx/rx/crc/decode/handshake flags`) for no-RX debugging (`src/app/compute_link_client.cpp`).
- [DONE] SPI protocol decoder auto-resync for frame rotation in no-SS/no-CS mode (`src/protocol/protocol_codec.cpp`).
- [DONE] LED status pattern manager by startup state (`src/status/led_status_manager.*`).
- [PARTIAL] Stage 2 mode manager + heading hold + mission runner are present but need field tuning.
- [PARTIAL] Stage 3 HC12 backend stub exists, integration pending.
