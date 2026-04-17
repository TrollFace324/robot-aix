#pragma once

namespace robot
{
namespace config
{
constexpr uint8_t kImuI2cAddress = 0x68;
constexpr bool kImuAutoCalibrateAtStartup = true;
constexpr int8_t kImuYawSignDefault = 1; // Temporary default, verify on hardware.
constexpr uint16_t kImuCalibrationDurationMsDefault = 2000;
constexpr uint16_t kImuCalibrationMinSamplesDefault = 200;
constexpr uint16_t kImuSamplePeriodUsDefault = 5000;
// The current IMU installation sits near 16 degrees at rest, so the default
// must leave headroom above that nominal mounting offset.
constexpr uint8_t kImuTiltLimitDegDefault = 20;
constexpr uint8_t kImuTiltConfirmSamplesDefault = 4;
constexpr uint8_t kImuPublishPeriodMs = 10;
constexpr uint8_t kImuFreshAgeMs = 60;
constexpr float kImuCalibrationGyroStillnessDps = 8.0f;
constexpr float kImuCalibrationAccelStillnessG = 0.10f;
// Require sustained motion before restarting startup calibration; this avoids
// single-sample spikes or brief bus jitter from keeping the IMU permanently in
// calibrating state.
constexpr uint16_t kImuCalibrationMotionResetMs = 250;
constexpr float kImuAccelScaleLsbPerG = 16384.0f;
constexpr float kImuGyroScale500Dps = 65.5f;

} // namespace config
} // namespace robot
