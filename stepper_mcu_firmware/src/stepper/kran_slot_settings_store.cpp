#include "stepper/kran_slot_settings_store.h"

#include <EEPROM.h>
#include <string.h>

#include "protocol_types.h"

namespace
{
constexpr uint16_t kKranSlotConfigMagic = 0x4B53;
constexpr uint16_t kKranSlotConfigVersion = 1;
constexpr int kEepromBaseAddress = 0;

struct StoredKranSlotConfig
{
    uint16_t magic = kKranSlotConfigMagic;
    uint16_t version = kKranSlotConfigVersion;
    uint8_t slot_count = robot::config::kKranSlotCount;
    uint8_t reserved = 0;
    int16_t slot_targets_mm[robot::config::kKranSlotCount]{};
    uint16_t crc = 0;
};

uint16_t configCrc(const StoredKranSlotConfig &stored)
{
    StoredKranSlotConfig copy = stored;
    copy.crc = 0;
    return robot::protocol::crc16Ccitt(
        reinterpret_cast<const uint8_t *>(&copy),
        sizeof(copy));
}
} // namespace

namespace robot
{
namespace stepper
{
void KranSlotSettingsStore::loadDefaults(
    int16_t out[config::kKranSlotCount]) const
{
    memcpy(out, config::kKranSlotTargetMm, sizeof(config::kKranSlotTargetMm));
}

bool KranSlotSettingsStore::loadOrDefaults(
    int16_t out[config::kKranSlotCount],
    bool *loaded_from_eeprom) const
{
    if (loaded_from_eeprom != nullptr)
    {
        *loaded_from_eeprom = false;
    }

    StoredKranSlotConfig stored;
    EEPROM.get(kEepromBaseAddress, stored);
    if (stored.magic != kKranSlotConfigMagic ||
        stored.version != kKranSlotConfigVersion ||
        stored.slot_count != config::kKranSlotCount ||
        stored.crc != configCrc(stored))
    {
        loadDefaults(out);
        return false;
    }

    memcpy(out, stored.slot_targets_mm, sizeof(stored.slot_targets_mm));
    if (loaded_from_eeprom != nullptr)
    {
        *loaded_from_eeprom = true;
    }
    return true;
}

bool KranSlotSettingsStore::save(
    const int16_t slot_targets[config::kKranSlotCount]) const
{
    StoredKranSlotConfig stored;
    memcpy(stored.slot_targets_mm, slot_targets, sizeof(stored.slot_targets_mm));
    stored.crc = configCrc(stored);
    EEPROM.put(kEepromBaseAddress, stored);
    return true;
}

bool KranSlotSettingsStore::resetToDefaults() const
{
    int16_t slot_targets[config::kKranSlotCount]{};
    loadDefaults(slot_targets);
    return save(slot_targets);
}

} // namespace stepper
} // namespace robot
