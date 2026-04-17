#pragma once

#include "drive/heading_hold_controller.h"

namespace robot
{
namespace drive
{
class ControlSettingsStore
{
public:
    HeadingHoldConfig loadOrDefaults(bool *loaded_from_eeprom = nullptr) const;
    bool save(const HeadingHoldConfig &config) const;
    bool resetToDefaults() const;

private:
    HeadingHoldConfig defaults() const;
};

} // namespace drive
} // namespace robot
