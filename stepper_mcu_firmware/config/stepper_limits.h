#pragma once

namespace robot
{
namespace config
{
constexpr int16_t kMinTravelMm = 0;
constexpr int16_t kMaxTravelMm = 2000;
constexpr uint8_t kDefaultStopMode = 1;
constexpr int16_t kHomeBackoffMm = 10;

// Home switch active level.
// true  = active LOW  (NO switch to GND with INPUT_PULLUP: pressed → LOW)
// false = active HIGH (NC switch to GND with INPUT_PULLUP: released → HIGH)
constexpr bool kHomeSwitchActiveLow = true;

} // namespace config
} // namespace robot
