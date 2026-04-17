# Protocols (stepper_mcu_firmware)

## I2C command/status transport
- SOF: `0x5A`
- Frame size: 24 bytes
- CRC: CRC8 poly `0xD5`
- Commands:
  - CMD_HOME
  - CMD_GOTO_MM
  - CMD_EXTEND_MM
  - CMD_RETRACT_MM
  - CMD_STOP
  - CMD_SET_LIMITS
  - CMD_SET_STOP_MODE
  - CMD_REQUEST_STATUS
- Status states:
  - STATUS_IDLE
  - STATUS_MOVING
  - STATUS_HOMING
  - STATUS_STOPPED
  - STATUS_ERROR
