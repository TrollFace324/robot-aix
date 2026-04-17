#include <cassert>
#include <cstdint>

#include "config/imu_config.h"
#include "imu/imu_settings_store.h"
#include "protocol_types.h"
#include "EEPROM.h"

namespace
{
using robot::imu::ImuSettingsStore;
using robot::imu::Mpu6500RuntimeConfig;

constexpr uint16_t kImuConfigMagic = 0x4D49;
constexpr uint16_t kImuConfigVersion = 1;

struct StoredImuConfig
{
    uint16_t magic = kImuConfigMagic;
    uint16_t version = kImuConfigVersion;
    int8_t yaw_sign = 1;
    uint8_t tilt_limit_deg = 15;
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

void testBlankEepromFallsBackToDefaults()
{
    EEPROM.clear();

    ImuSettingsStore store;
    bool loaded_from_eeprom = true;
    const Mpu6500RuntimeConfig runtime = store.loadOrDefaults(&loaded_from_eeprom);

    assert(!loaded_from_eeprom);
    assert(runtime.tilt_limit_deg == robot::config::kImuTiltLimitDegDefault);
}

void testLegacyTiltLimitGetsRaisedToNewDefault()
{
    EEPROM.clear();

    StoredImuConfig stored;
    stored.tilt_limit_deg = 15;
    stored.crc = configCrc(stored);
    EEPROM.put(0, stored);

    ImuSettingsStore store;
    bool loaded_from_eeprom = false;
    const Mpu6500RuntimeConfig runtime = store.loadOrDefaults(&loaded_from_eeprom);

    assert(loaded_from_eeprom);
    assert(runtime.tilt_limit_deg == robot::config::kImuTiltLimitDegDefault);
    assert(runtime.tilt_confirm_samples == stored.tilt_confirm_samples);
}

void testHigherStoredTiltLimitIsPreserved()
{
    EEPROM.clear();

    StoredImuConfig stored;
    stored.tilt_limit_deg = 27;
    stored.crc = configCrc(stored);
    EEPROM.put(0, stored);

    ImuSettingsStore store;
    const Mpu6500RuntimeConfig runtime = store.loadOrDefaults();

    assert(runtime.tilt_limit_deg == 27);
}
} // namespace

int main()
{
    testBlankEepromFallsBackToDefaults();
    testLegacyTiltLimitGetsRaisedToNewDefault();
    testHigherStoredTiltLimitIsPreserved();
    return 0;
}
