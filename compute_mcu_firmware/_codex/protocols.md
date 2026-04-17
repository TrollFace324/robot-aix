# Protocols (compute_mcu_firmware)

## SPI (PRIZM <-> Compute)
- 32-byte fixed frame.
- Header: SOF/VER/MSG/FLAGS/SRC/DST/SEQ/ACK_SEQ/LEN.
- Payload up to 20 bytes.
- CRC16 (CCITT-FALSE) over bytes `[0..28]`.
- Implemented messages:
  - HELLO, HELLO_ACK
  - GET_STATUS, STATUS
  - GET_CAPABILITIES, CAPABILITIES
  - HEARTBEAT, HEARTBEAT_ACK
  - STARTUP_CHECK_RESULT
  - READY_TO_RUN, RUN_LOCKED
  - ECHO_TEST, TEST_FLAG

## I2C (Compute -> Stepper)
- 24-byte fixed frame with SOF `0x5A` and CRC8 polynomial `0xD5`.
- Commands: HOME/GOTO/EXTEND/RETRACT/STOP/SET_LIMITS/SET_STOP_MODE/REQUEST_STATUS.
- Status includes position, target, homed flag, driver state, home switch, fault flags.
