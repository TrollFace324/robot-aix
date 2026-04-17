# Protocols (prizm_firmware)

## SPI PRIZM <-> Compute
- Frame: 32 bytes fixed.
- Layout: `SOF(0xA5), VER, MSG, FLAGS, SRC, DST, SEQ, ACK_SEQ, LEN, PAYLOAD(<=20), CRC16, PAD`.
- CRC: CRC-16/CCITT-FALSE.
- Stage 1 messages implemented:
  - HELLO / HELLO_ACK
  - GET_STATUS / STATUS
  - GET_CAPABILITIES / CAPABILITIES
  - HEARTBEAT / HEARTBEAT_ACK
  - STARTUP_CHECK_RESULT
  - READY_TO_RUN / RUN_LOCKED
  - ECHO_TEST / TEST_FLAG
- Retry/timeout policy lives in PRIZM link client timing config.

## I2C Compute <-> Stepper
- Implemented on compute + stepper side; PRIZM consumes only aggregated health.
