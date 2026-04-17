#include <cassert>

#include "EEPROM.h"
#include "config/kran_slot_config.h"
#include "stepper/kran_slot_settings_store.h"

namespace
{
using robot::config::kKranSlotCount;
using robot::config::kKranSlotTargetMm;
using robot::stepper::KranSlotSettingsStore;

void testLoadsDefaultsWhenEepromIsEmpty()
{
    EEPROM.clear();

    KranSlotSettingsStore store;
    int16_t slot_targets[kKranSlotCount]{};

    const bool loaded = store.loadOrDefaults(slot_targets);
    assert(!loaded);
    for (uint8_t slot = 0; slot < kKranSlotCount; ++slot)
    {
        assert(slot_targets[slot] == kKranSlotTargetMm[slot]);
    }
}

void testPersistsSlotTargetsIntoEeprom()
{
    EEPROM.clear();

    KranSlotSettingsStore store;
    int16_t slot_targets[kKranSlotCount]{};
    for (uint8_t slot = 0; slot < kKranSlotCount; ++slot)
    {
        slot_targets[slot] = static_cast<int16_t>(slot * 7);
    }

    assert(store.save(slot_targets));

    int16_t reloaded[kKranSlotCount]{};
    const bool loaded = store.loadOrDefaults(reloaded);
    assert(loaded);
    for (uint8_t slot = 0; slot < kKranSlotCount; ++slot)
    {
        assert(reloaded[slot] == slot_targets[slot]);
    }
}
} // namespace

int main()
{
    testLoadsDefaultsWhenEepromIsEmpty();
    testPersistsSlotTargetsIntoEeprom();
    return 0;
}
