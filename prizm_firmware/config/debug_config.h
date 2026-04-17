#pragma once

namespace robot
{
namespace config
{
constexpr bool kEnableBootSerialTrace = false;
constexpr uint32_t kBootSerialBaud = 115200;
// Prints decoded SPI RX packets from compute to UART.
// WARNING: PRIZM has one HW UART; keep false in normal CRSF operation.
constexpr bool kEnableSpiUartTrace = false;
// When SPI UART trace is enabled, disable CRSF reader on Serial to avoid bus contention.
constexpr bool kDisableRcWhenSpiTrace = true;
// Temporary service mode for manual encoder calibration over USB serial.
// Disables RC/SPI runtime and prints raw encoder counts for FL/FR/RL/RR.
constexpr bool kEnableEncoderCalibrationSession = false;

} // namespace config
} // namespace robot
