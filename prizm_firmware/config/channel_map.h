#pragma once

namespace robot
{
namespace config
{
constexpr uint8_t kChRightX = 0; // CH1
constexpr uint8_t kChRightY = 1; // CH2
constexpr uint8_t kChConfig = 2; // CH3
constexpr uint8_t kChLeftX = 3;  // CH4
constexpr uint8_t kChM1Action = 5; // CH6
constexpr uint8_t kChM2Toggle = 6; // CH7
constexpr uint8_t kChHeadingHold = 9; // CH10

constexpr uint16_t kChannelHighThreshold = 1300;
constexpr uint16_t kChannelMid = 992;

} // namespace config
} // namespace robot
