#pragma once

#include "imu_if.h"

namespace robot
{
namespace imu
{
struct Mpu6500RuntimeConfig
{
    int8_t yaw_sign = 1;
    uint16_t calibration_duration_ms = 2000;
    uint16_t calibration_min_samples = 200;
    uint16_t sample_period_us = 5000;
    uint8_t tilt_limit_deg = 20;
    uint8_t tilt_confirm_samples = 4;
};

class Mpu6500Service : public IImuService
{
public:
    void configure(const Mpu6500RuntimeConfig &config);
    bool begin(TwoWire &wire, uint8_t address) override;
    void update(uint32_t now_ms) override;
    bool healthy() const override;
    bool dataValid() const override;
    bool calibrating() const override;
    bool tiltFault() const override;
    uint32_t lastSampleMs() const override;
    float yawDeg() const override;
    float yawRateDps() const override;
    float rollDeg() const override;
    float pitchDeg() const override;

private:
    bool probeAddress(TwoWire &wire, uint8_t address);
    bool configureSensor();
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);
    void resetRuntime(uint32_t now_ms);
    void updateTilt(float accel_x_g, float accel_y_g, float accel_z_g);
    float wrapAngle(float deg) const;

    TwoWire *wire_ = nullptr;
    bool healthy_ = false;
    bool configured_ = false;
    bool calibrated_ = false;
    bool tilt_fault_ = false;
    uint8_t address_ = 0;
    uint32_t last_sample_us_ = 0;
    uint32_t last_sample_ms_ = 0;
    uint32_t calibration_started_ms_ = 0;
    uint32_t calibration_motion_started_ms_ = 0;
    float calibration_accum_dps_ = 0.0f;
    float calibration_min_dps_ = 0.0f;
    float calibration_max_dps_ = 0.0f;
    uint16_t calibration_samples_ = 0;
    float gyro_z_bias_dps_ = 0.0f;
    float yaw_deg_ = 0.0f;
    float yaw_rate_dps_ = 0.0f;
    float roll_deg_ = 0.0f;
    float pitch_deg_ = 0.0f;
    uint8_t tilt_confirm_count_ = 0;
    Mpu6500RuntimeConfig config_{};
};

} // namespace imu
} // namespace robot
