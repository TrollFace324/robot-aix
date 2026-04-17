#pragma once

#include "imu/mpu6500_service.h"

namespace robot
{
namespace imu
{
class ImuSettingsStore
{
public:
    Mpu6500RuntimeConfig loadOrDefaults(bool *loaded_from_eeprom = nullptr) const;
    bool save(const Mpu6500RuntimeConfig &config) const;
    bool resetToDefaults() const;

private:
    Mpu6500RuntimeConfig defaults() const;
};

} // namespace imu
} // namespace robot
