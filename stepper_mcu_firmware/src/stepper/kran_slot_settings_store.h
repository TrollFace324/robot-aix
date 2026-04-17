#pragma once

#include <Arduino.h>

#include "config/kran_slot_config.h"

namespace robot
{
namespace stepper
{
class KranSlotSettingsStore
{
public:
    bool loadOrDefaults(int16_t out[config::kKranSlotCount],
                        bool *loaded_from_eeprom = nullptr) const;
    bool save(const int16_t slot_targets[config::kKranSlotCount]) const;
    bool resetToDefaults() const;

private:
    void loadDefaults(int16_t out[config::kKranSlotCount]) const;
};

} // namespace stepper
} // namespace robot
