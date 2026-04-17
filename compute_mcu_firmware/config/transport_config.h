#pragma once

#include <Arduino.h>

namespace robot
{
namespace config
{
constexpr uint8_t kSpiSsPin = 10;
// Standard wiring mode:
// PRIZM D2 drives compute hardware SS(D10).
constexpr bool kSpiUseExternalSs = true;
constexpr bool kForceSsOutputLowInNoSsMode = false;
constexpr bool kSpiUseFrameSync = false;
constexpr uint8_t kSpiFrameSyncPin = 5;
constexpr uint8_t kStepperI2cAddress = 0x13;
constexpr bool kRequireStepperOnlineForReady = true;
// Keep the shared bus conservative: the MPU is fine at higher rates, but the
// stepper branch only needs a 24-byte status frame and is more timing-sensitive.
constexpr uint32_t kComputeI2cClockHz = 25000;
// MiniCore applies the timeout to the whole blocking TWI transaction. A
// 24-byte stepper frame at 25 kHz takes roughly 9-10 ms on the wire, so keep
// generous headroom for ACK phases, clock stretching, and recovery retries.
constexpr uint16_t kComputeI2cTimeoutUs = 50000;
constexpr uint8_t kComputeI2cRecoveryClocks = 18;
constexpr uint8_t kComputeI2cRecoveryPulseUs = 6;

} // namespace config
} // namespace robot
