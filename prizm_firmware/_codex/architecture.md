# Architecture (prizm_firmware)

- Role: PRIZM main controller, CRSF ingest, manual drive, startup gatekeeper.
- Transport: hardware SPI master backend (`SpiMasterBackend`) toward compute MCU.
- Safety: explicit startup state machine and hard run-lock until compute returns valid self-check.
- Reuse strategy:
  - keep proven RC parser and mecanum/motor stack from `LigaRobotMainBoard`.
  - wrap with new startup/mode/transport orchestration instead of rewriting low-level logic.
- Data flow:
  - RC UART -> `RcCrsf` -> mode manager -> (manual/autonomous) -> drive mixer -> `MotorsPrizmExp`.
  - `ComputeLinkClient` handles HELLO/CAPABILITIES/STATUS/HEARTBEAT and READY lock gating.
