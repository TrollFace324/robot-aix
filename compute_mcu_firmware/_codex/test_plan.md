# Test Plan (compute_mcu_firmware)

1. Build check: `pio run`.
2. SPI protocol bench:
   - send HELLO from PRIZM and verify HELLO_ACK.
   - poll STATUS and CAPABILITIES.
3. Self-check gate:
   - simulate IMU fail and verify RUN_LOCKED.
   - restore IMU + stepper and verify READY_TO_RUN.
4. I2C link tests:
   - stepper online/offline transitions reflected in startup check result.
5. Fault injection:
   - invalid CRC packet ignored and not promoted to valid status.
