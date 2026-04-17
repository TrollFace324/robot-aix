#include "imu/imu_settings_store.h"

#include <EEPROM.h>
#include <string.h>

#include "config/imu_config.h"
#include "protocol_types.h"

namespace
{
constexpr uint16_t kImuConfigMagic = 0x4D49;
constexpr uint16_t kImuConfigVersion = 1;
constexpr int kEepromBaseAddress = 0;

struct StoredImuConfig
{
    uint16_t magic = kImuConfigMagic;
    uint16_t version = kImuConfigVersion;
    int8_t yaw_sign = 1;
    uint8_t tilt_limit_deg = 20;
    uint8_t tilt_confirm_samples = 4;
    uint8_t reserved0 = 0;
    uint16_t calibration_duration_ms = 2000;
    uint16_t calibration_min_samples = 200;
    uint16_t sample_period_us = 5000;
    uint16_t crc = 0;
};

uint16_t configCrc(const StoredImuConfig &stored)
{
    StoredImuConfig copy = stored;
    copy.crc = 0;
    return robot::protocol::crc16Ccitt(reinterpret_cast<const uint8_t *>(&copy), sizeof(copy));
}

robot::imu::Mpu6500RuntimeConfig runtimeConfigFromStored(const StoredImuConfig &stored)
{
    robot::imu::Mpu6500RuntimeConfig out;
    out.yaw_sign = (stored.yaw_sign >= 0) ? 1 : -1;
    out.tilt_limit_deg = stored.tilt_limit_deg;
    if (out.tilt_limit_deg < robot::config::kImuTiltLimitDegDefault)
    {
        out.tilt_limit_deg = robot::config::kImuTiltLimitDegDefault;
    }
    out.tilt_confirm_samples = stored.tilt_confirm_samples;
    out.calibration_duration_ms = stored.calibration_duration_ms;
    out.calibration_min_samples = stored.calibration_min_samples;
    out.sample_period_us = stored.sample_period_us;
    return out;
}
} // namespace

namespace robot
{
namespace imu
{
Mpu6500RuntimeConfig ImuSettingsStore::defaults() const
{
    Mpu6500RuntimeConfig out;
    out.yaw_sign = config::kImuYawSignDefault;
    out.calibration_duration_ms = config::kImuCalibrationDurationMsDefault;
    out.calibration_min_samples = config::kImuCalibrationMinSamplesDefault;
    out.sample_period_us = config::kImuSamplePeriodUsDefault;
    out.tilt_limit_deg = config::kImuTiltLimitDegDefault;
    out.tilt_confirm_samples = config::kImuTiltConfirmSamplesDefault;
    return out;
}

Mpu6500RuntimeConfig ImuSettingsStore::loadOrDefaults(bool *loaded_from_eeprom) const
{
    if (loaded_from_eeprom != nullptr)
    {
        *loaded_from_eeprom = false;
    }

    StoredImuConfig stored;
    EEPROM.get(kEepromBaseAddress, stored);
    if (stored.magic != kImuConfigMagic || stored.version != kImuConfigVersion)
    {
        return defaults();
    }
    if (stored.crc != configCrc(stored))
    {
        return defaults();
    }

    Mpu6500RuntimeConfig out = runtimeConfigFromStored(stored);

    if (loaded_from_eeprom != nullptr)
    {
        *loaded_from_eeprom = true;
    }
    return out;
}

bool ImuSettingsStore::save(const Mpu6500RuntimeConfig &config) const
{
    StoredImuConfig stored;
    stored.yaw_sign = (config.yaw_sign >= 0) ? 1 : -1;
    stored.tilt_limit_deg = config.tilt_limit_deg;
    stored.tilt_confirm_samples = config.tilt_confirm_samples;
    stored.calibration_duration_ms = config.calibration_duration_ms;
    stored.calibration_min_samples = config.calibration_min_samples;
    stored.sample_period_us = static_cast<uint16_t>(config.sample_period_us);
    stored.crc = configCrc(stored);
    EEPROM.put(kEepromBaseAddress, stored);
    return true;
}

bool ImuSettingsStore::resetToDefaults() const
{
    return save(defaults());
}

} // namespace imu
} // namespace robot
