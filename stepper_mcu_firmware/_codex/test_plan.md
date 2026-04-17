# Test Plan (stepper_mcu_firmware)

1. Build check: `pio run`.
2. I2C smoke:
   - REQUEST_STATUS returns valid CRC frame.
   - GOTO/STOP updates status transitions.
3. Homing:
   - simulate/home-switch active path, verify homed flag.
4. Limits:
   - commands outside travel range are clamped.
5. Stop mode:
   - verify enable line behavior for configured mode.
