# Architecture (compute_mcu_firmware)

- Role: compute MCU, SPI slave to PRIZM, I2C master to stepper MCU.
- Main modules:
  - `SpiSlaveBackend`: transport ingress/egress for fixed-size SPI frames.
  - `ComputeController`: protocol router and high-level handshake/status behavior.
  - `SelfCheckRunner`: startup diagnostic aggregation.
  - `I2cStepperClient`: lower-level stepper status/command transport.
  - `Mpu6500Service`: IMU health probe layer.
- Startup gate semantics:
  - `startup_ready=true` only when IMU health is valid and stepper online (config policy).
