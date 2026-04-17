#include "drive/control_settings_store.h"

#include <EEPROM.h>

#include "protocol_types.h"

namespace
{
constexpr uint16_t kControlConfigMagic = 0x4844;
constexpr uint16_t kControlConfigVersion = 2;
constexpr int kEepromBaseAddress = 0;

struct StoredControlConfig
{
    uint16_t magic = kControlConfigMagic;
    uint16_t version = kControlConfigVersion;
    float heading_kp = 1.2f;
    float heading_ki = 0.08f;
    float rate_kp = 0.008f;
    float wheel_balance_kp = 0.10f;
    float heading_deadband_deg = 0.8f;
    float rate_deadband_dps = 2.5f;
    float target_rate_limit_dps = 100.0f;
    float output_limit = 0.50f;
    float slew_per_s = 2.5f;
    float static_output = 0.20f;
    float static_error_deg = 1.0f;
    float encoder_motion_threshold_cps = 16.0f;
    int16_t wheel_speed_limit_dps = 420;
    uint8_t imu_fresh_age_ms = 60;
    uint8_t encoder_poll_interval_ms = 3;
    uint16_t encoder_ticks_per_rev = 269;
    uint16_t crc = 0;
};

uint16_t configCrc(const StoredControlConfig &stored)
{
    StoredControlConfig copy = stored;
    copy.crc = 0;
    return robot::protocol::crc16Ccitt(reinterpret_cast<const uint8_t *>(&copy), sizeof(copy));
}
} // namespace

namespace robot
{
namespace drive
{
HeadingHoldConfig ControlSettingsStore::defaults() const
{
    return HeadingHoldConfig{};
}

HeadingHoldConfig ControlSettingsStore::loadOrDefaults(bool *loaded_from_eeprom) const
{
    if (loaded_from_eeprom != nullptr)
    {
        *loaded_from_eeprom = false;
    }

    StoredControlConfig stored;
    EEPROM.get(kEepromBaseAddress, stored);
    if (stored.magic != kControlConfigMagic || stored.version != kControlConfigVersion)
    {
        return defaults();
    }
    if (stored.crc != configCrc(stored))
    {
        return defaults();
    }

    HeadingHoldConfig out;
    out.heading_kp = stored.heading_kp;
    out.heading_ki = stored.heading_ki;
    out.rate_kp = stored.rate_kp;
    out.wheel_balance_kp = stored.wheel_balance_kp;
    out.heading_deadband_deg = stored.heading_deadband_deg;
    out.rate_deadband_dps = stored.rate_deadband_dps;
    out.target_rate_limit_dps = stored.target_rate_limit_dps;
    out.output_limit = stored.output_limit;
    out.slew_per_s = stored.slew_per_s;
    out.static_output = stored.static_output;
    out.static_error_deg = stored.static_error_deg;
    out.encoder_motion_threshold_cps = stored.encoder_motion_threshold_cps;
    out.wheel_speed_limit_dps = stored.wheel_speed_limit_dps;
    out.imu_fresh_age_ms = stored.imu_fresh_age_ms;
    out.encoder_poll_interval_ms = stored.encoder_poll_interval_ms;
    out.encoder_ticks_per_rev = (stored.encoder_ticks_per_rev != 0)
                                    ? stored.encoder_ticks_per_rev
                                    : defaults().encoder_ticks_per_rev;

    if (loaded_from_eeprom != nullptr)
    {
        *loaded_from_eeprom = true;
    }
    return out;
}

bool ControlSettingsStore::save(const HeadingHoldConfig &config) const
{
    StoredControlConfig stored;
    stored.heading_kp = config.heading_kp;
    stored.heading_ki = config.heading_ki;
    stored.rate_kp = config.rate_kp;
    stored.wheel_balance_kp = config.wheel_balance_kp;
    stored.heading_deadband_deg = config.heading_deadband_deg;
    stored.rate_deadband_dps = config.rate_deadband_dps;
    stored.target_rate_limit_dps = config.target_rate_limit_dps;
    stored.output_limit = config.output_limit;
    stored.slew_per_s = config.slew_per_s;
    stored.static_output = config.static_output;
    stored.static_error_deg = config.static_error_deg;
    stored.encoder_motion_threshold_cps = config.encoder_motion_threshold_cps;
    stored.wheel_speed_limit_dps = config.wheel_speed_limit_dps;
    stored.imu_fresh_age_ms = config.imu_fresh_age_ms;
    stored.encoder_poll_interval_ms = config.encoder_poll_interval_ms;
    stored.encoder_ticks_per_rev = config.encoder_ticks_per_rev;
    stored.crc = configCrc(stored);
    EEPROM.put(kEepromBaseAddress, stored);
    return true;
}

bool ControlSettingsStore::resetToDefaults() const
{
    return save(defaults());
}

} // namespace drive
} // namespace robot
