#include "imu/mpu6500_service.h"

#include <math.h>

#include "config/imu_config.h"

namespace
{
constexpr uint8_t kRegSmplrtDiv = 0x19;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegAccelXOutH = 0x3B;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegPwrMgmt2 = 0x6C;
constexpr uint8_t kRegWhoAmI = 0x75;

constexpr uint8_t kWhoAmIMpu6500 = 0x70;
constexpr uint8_t kWhoAmIMpu9250 = 0x71;
constexpr uint8_t kWhoAmIAlt = 0x68;

constexpr uint8_t kClockSelectGyroX = 0x01;
constexpr uint8_t kGyroFs500Dps = 0x08;
constexpr uint8_t kAccelFs2G = 0x00;
constexpr uint8_t kDlpf41Hz = 0x03;
constexpr uint8_t kSampleDivider200Hz = 0x04;

constexpr uint32_t kResetDelayMs = 100;
constexpr uint32_t kConfigDelayMs = 10;

int16_t makeInt16(uint8_t msb, uint8_t lsb)
{
    return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | static_cast<uint16_t>(lsb));
}

float clampfLocal(float value, float lo, float hi)
{
    if (value < lo)
    {
        return lo;
    }
    if (value > hi)
    {
        return hi;
    }
    return value;
}
} // namespace

namespace robot
{
namespace imu
{
void Mpu6500Service::configure(const Mpu6500RuntimeConfig &config)
{
    config_ = config;
}

bool Mpu6500Service::begin(TwoWire &wire, uint8_t address)
{
    wire_ = &wire;
    healthy_ = false;
    configured_ = false;
    address_ = 0;

    if (probeAddress(wire, address))
    {
        if (!configureSensor())
        {
            healthy_ = false;
            configured_ = false;
            address_ = 0;
            return false;
        }

        resetRuntime(millis());
        healthy_ = true;
        configured_ = true;
        return true;
    }

    const uint8_t alternate = (address == 0x68) ? 0x69 : ((address == 0x69) ? 0x68 : 0x00);
    if (alternate != 0x00 && probeAddress(wire, alternate))
    {
        if (!configureSensor())
        {
            healthy_ = false;
            configured_ = false;
            address_ = 0;
            return false;
        }

        resetRuntime(millis());
        healthy_ = true;
        configured_ = true;
        return true;
    }

    healthy_ = false;
    configured_ = false;
    address_ = 0;
    return false;
}

void Mpu6500Service::update(uint32_t now_ms)
{
    if (!healthy_ || !configured_ || wire_ == nullptr || address_ == 0)
    {
        return;
    }

    const uint32_t now_us = micros();
    if ((last_sample_us_ != 0) &&
        (static_cast<uint32_t>(now_us - last_sample_us_) < config_.sample_period_us))
    {
        return;
    }

    uint8_t raw[14]{};
    if (!readRegisters(kRegAccelXOutH, raw, sizeof(raw)))
    {
        healthy_ = false;
        configured_ = false;
        calibrated_ = false;
        tilt_fault_ = false;
        yaw_rate_dps_ = 0.0f;
        return;
    }

    const uint32_t prev_sample_us = last_sample_us_;
    last_sample_us_ = now_us;
    last_sample_ms_ = now_ms;

    const int16_t accel_x_raw = makeInt16(raw[0], raw[1]);
    const int16_t accel_y_raw = makeInt16(raw[2], raw[3]);
    const int16_t accel_z_raw = makeInt16(raw[4], raw[5]);
    const int16_t gyro_z_raw = makeInt16(raw[12], raw[13]);

    const float accel_x_g = static_cast<float>(accel_x_raw) / config::kImuAccelScaleLsbPerG;
    const float accel_y_g = static_cast<float>(accel_y_raw) / config::kImuAccelScaleLsbPerG;
    const float accel_z_g = static_cast<float>(accel_z_raw) / config::kImuAccelScaleLsbPerG;

    const float gyro_z_dps = static_cast<float>(gyro_z_raw) / config::kImuGyroScale500Dps;
    const float yaw_axis_dps = static_cast<float>(config_.yaw_sign) * gyro_z_dps;

    updateTilt(accel_x_g, accel_y_g, accel_z_g);

    if (!calibrated_)
    {
        const float accel_mag = sqrtf((accel_x_g * accel_x_g) +
                                      (accel_y_g * accel_y_g) +
                                      (accel_z_g * accel_z_g));
        const float accel_error = fabsf(accel_mag - 1.0f);

        if (accel_error > config::kImuCalibrationAccelStillnessG)
        {
            if (calibration_motion_started_ms_ == 0)
            {
                calibration_motion_started_ms_ = now_ms;
            }
            else if (now_ms - calibration_motion_started_ms_ >= config::kImuCalibrationMotionResetMs)
            {
                calibration_started_ms_ = now_ms;
                calibration_accum_dps_ = 0.0f;
                calibration_samples_ = 0;
                calibration_motion_started_ms_ = 0;
            }
            yaw_rate_dps_ = 0.0f;
            return;
        }

        calibration_motion_started_ms_ = 0;
        if (calibration_samples_ == 0)
        {
            calibration_min_dps_ = yaw_axis_dps;
            calibration_max_dps_ = yaw_axis_dps;
        }
        else
        {
            if (yaw_axis_dps < calibration_min_dps_)
            {
                calibration_min_dps_ = yaw_axis_dps;
            }
            if (yaw_axis_dps > calibration_max_dps_)
            {
                calibration_max_dps_ = yaw_axis_dps;
            }
        }
        calibration_accum_dps_ += yaw_axis_dps;
        ++calibration_samples_;
        yaw_rate_dps_ = yaw_axis_dps;

        if ((now_ms - calibration_started_ms_ >= config_.calibration_duration_ms) &&
            (calibration_samples_ >= config_.calibration_min_samples))
        {
            const float gyro_span_dps = calibration_max_dps_ - calibration_min_dps_;
            if (gyro_span_dps <= config::kImuCalibrationGyroStillnessDps)
            {
                gyro_z_bias_dps_ = calibration_accum_dps_ / static_cast<float>(calibration_samples_);
                calibrated_ = true;
                yaw_deg_ = 0.0f;
                yaw_rate_dps_ = 0.0f;
            }
            else
            {
                calibration_started_ms_ = now_ms;
                calibration_accum_dps_ = 0.0f;
                calibration_samples_ = 0;
                calibration_min_dps_ = 0.0f;
                calibration_max_dps_ = 0.0f;
            }
        }
        return;
    }

    const float corrected_dps = yaw_axis_dps - gyro_z_bias_dps_;
    yaw_rate_dps_ = corrected_dps;

    if (prev_sample_us != 0)
    {
        const float dt_s = static_cast<float>(now_us - prev_sample_us) * 1.0e-6f;
        yaw_deg_ = wrapAngle(yaw_deg_ + (corrected_dps * dt_s));
    }
}

bool Mpu6500Service::probeAddress(TwoWire &wire, uint8_t address)
{
    wire.beginTransmission(address);
    wire.write(kRegWhoAmI);
    if (wire.endTransmission(false) != 0)
    {
        return false;
    }

    const uint8_t req = wire.requestFrom(address, static_cast<uint8_t>(1));
    if (req != 1 || wire.available() <= 0)
    {
        return false;
    }

    const uint8_t who = static_cast<uint8_t>(wire.read());
    healthy_ = (who == kWhoAmIMpu6500 || who == kWhoAmIMpu9250 || who == kWhoAmIAlt);
    address_ = healthy_ ? address : 0;
    return healthy_;
}

bool Mpu6500Service::configureSensor()
{
    if (wire_ == nullptr || address_ == 0)
    {
        return false;
    }

    if (!writeRegister(kRegPwrMgmt1, 0x80))
    {
        return false;
    }
    delay(kResetDelayMs);

    if (!writeRegister(kRegPwrMgmt1, kClockSelectGyroX))
    {
        return false;
    }
    delay(kConfigDelayMs);

    if (!writeRegister(kRegPwrMgmt2, 0x00))
    {
        return false;
    }
    if (!writeRegister(kRegConfig, kDlpf41Hz))
    {
        return false;
    }
    if (!writeRegister(kRegSmplrtDiv, kSampleDivider200Hz))
    {
        return false;
    }
    if (!writeRegister(kRegGyroConfig, kGyroFs500Dps))
    {
        return false;
    }
    if (!writeRegister(kRegAccelConfig, kAccelFs2G))
    {
        return false;
    }

    uint8_t who_am_i = 0;
    if (!readRegisters(kRegWhoAmI, &who_am_i, 1))
    {
        return false;
    }

    return (who_am_i == kWhoAmIMpu6500) || (who_am_i == kWhoAmIMpu9250) || (who_am_i == kWhoAmIAlt);
}

bool Mpu6500Service::writeRegister(uint8_t reg, uint8_t value)
{
    if (wire_ == nullptr || address_ == 0)
    {
        return false;
    }

    wire_->beginTransmission(address_);
    wire_->write(reg);
    wire_->write(value);
    return wire_->endTransmission() == 0;
}

bool Mpu6500Service::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    if (wire_ == nullptr || address_ == 0 || buffer == nullptr || length == 0)
    {
        return false;
    }

    wire_->beginTransmission(address_);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0)
    {
        return false;
    }

    const uint8_t received = wire_->requestFrom(address_, length);
    if (received != length)
    {
        while (wire_->available() > 0)
        {
            static_cast<void>(wire_->read());
        }
        return false;
    }

    for (uint8_t i = 0; i < length; ++i)
    {
        if (wire_->available() <= 0)
        {
            return false;
        }
        buffer[i] = static_cast<uint8_t>(wire_->read());
    }

    return true;
}

void Mpu6500Service::resetRuntime(uint32_t now_ms)
{
    yaw_deg_ = 0.0f;
    yaw_rate_dps_ = 0.0f;
    roll_deg_ = 0.0f;
    pitch_deg_ = 0.0f;
    tilt_fault_ = false;
    tilt_confirm_count_ = 0;
    gyro_z_bias_dps_ = 0.0f;
    last_sample_us_ = 0;
    last_sample_ms_ = 0;
    calibration_started_ms_ = now_ms;
    calibration_motion_started_ms_ = 0;
    calibration_accum_dps_ = 0.0f;
    calibration_min_dps_ = 0.0f;
    calibration_max_dps_ = 0.0f;
    calibration_samples_ = 0;
    calibrated_ = !config::kImuAutoCalibrateAtStartup;
}

void Mpu6500Service::updateTilt(float accel_x_g, float accel_y_g, float accel_z_g)
{
    const float forward_g = -accel_y_g;
    const float left_g = accel_x_g;
    const float up_g = accel_z_g;

    roll_deg_ = atan2f(left_g, up_g) * 57.2957795f;
    pitch_deg_ = atan2f(forward_g, sqrtf((left_g * left_g) + (up_g * up_g))) * 57.2957795f;

    const bool over_limit = (fabsf(roll_deg_) >= static_cast<float>(config_.tilt_limit_deg)) ||
                            (fabsf(pitch_deg_) >= static_cast<float>(config_.tilt_limit_deg));
    if (over_limit)
    {
        if (tilt_confirm_count_ < 255)
        {
            ++tilt_confirm_count_;
        }
    }
    else if (tilt_confirm_count_ > 0)
    {
        --tilt_confirm_count_;
    }

    tilt_fault_ = tilt_confirm_count_ >= config_.tilt_confirm_samples;
}

bool Mpu6500Service::healthy() const
{
    return healthy_;
}

bool Mpu6500Service::dataValid() const
{
    return healthy_ && configured_ && calibrated_ && (last_sample_ms_ != 0);
}

bool Mpu6500Service::calibrating() const
{
    return healthy_ && configured_ && !calibrated_;
}

bool Mpu6500Service::tiltFault() const
{
    return tilt_fault_;
}

uint32_t Mpu6500Service::lastSampleMs() const
{
    return last_sample_ms_;
}

float Mpu6500Service::yawDeg() const
{
    return yaw_deg_;
}

float Mpu6500Service::yawRateDps() const
{
    return yaw_rate_dps_;
}

float Mpu6500Service::rollDeg() const
{
    return clampfLocal(roll_deg_, -90.0f, 90.0f);
}

float Mpu6500Service::pitchDeg() const
{
    return clampfLocal(pitch_deg_, -90.0f, 90.0f);
}

float Mpu6500Service::wrapAngle(float deg) const
{
    while (deg > 180.0f)
    {
        deg -= 360.0f;
    }
    while (deg < -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

} // namespace imu
} // namespace robot
