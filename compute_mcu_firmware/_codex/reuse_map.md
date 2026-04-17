# Reuse Map (compute_mcu_firmware)

- Source project audited: `D:\LigaProgram\LigaRobotMainBoard`.

## Reused
- Architectural conventions from existing CRSF and mecanum stack influenced protocol + control contracts.

## Adapted
- New compute-side protocol and transport layers created to fit SPI slave + I2C master topology.

## Not directly reused
- No direct code import from old project for compute MCU transport and IMU/stepper bridge.
