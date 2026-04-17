# Project Status (stepper_mcu_firmware)

- Active stage: Stage 1 implementation.
- Stable now:
  - I2C slave command server with low-overhead callbacks.
  - Stepper motion controller (home/goto/extend/retract/stop/limits/stop-mode).
  - Status publishing with required fields and CRC8 frame formatting.
- In progress:
  - Physical tuning of steps/mm, speed/accel, homing behavior on real mechanics.
- Blocking issues:
  - Real hardware pin mapping and stop-mode behavior policies need final calibration.
