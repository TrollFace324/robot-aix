#pragma once

#include <stdint.h>

namespace robot
{
namespace config
{
constexpr uint8_t kKranSlotCount = 11;

// Slot 0 is the farthest throw position; slot 10 is closest to home.
constexpr int16_t kKranSlotTargetMm[kKranSlotCount] = {
    110,
    99,
    88,
    77,
    66,
    55,
    44,
    33,
    22,
    11,
    0,
};

} // namespace config
} // namespace robot
