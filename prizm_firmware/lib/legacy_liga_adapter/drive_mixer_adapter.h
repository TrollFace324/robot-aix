#pragma once

#include "drive_mecanum.h"

namespace legacy_adapter
{
class DriveMixerAdapter
{
public:
    WheelCmd mix(const ChassisCmd &cmd) const { return DriveMecanum::mix(cmd); }
};
} // namespace legacy_adapter
