# Architecture (stepper_mcu_firmware)

- Role: stepper MCU, I2C slave endpoint for compute MCU.
- Core modules:
  - `I2cSlaveServer`: receives commands and serves latest status frame.
  - `MotionController`: command execution and internal state machine.
  - `StepperDriverAdapter`: low-level stepper driver operations.
  - `StatusReporter`: maps runtime state to protocol payload.
- Stage 1 behavior:
  - supports required command enum and publishes required status fields.
