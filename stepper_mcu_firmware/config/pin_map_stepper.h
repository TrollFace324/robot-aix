#pragma once

#include <Arduino.h>

namespace robot
{
namespace config
{
// Field wiring: STEP->PD6(D6), DIR->PD5(D5), EN->PD7(D7).
constexpr uint8_t kPinStep = 6;
constexpr uint8_t kPinDir = 5;
constexpr uint8_t kPinEnable = 7;
// MotionController treats logical +mm as travel away from the home switch at
// min_mm=0. Flipping DIR here without also remapping logical coordinates breaks
// homing/slot semantics and can drive max-slot moves into the home endstop.
constexpr bool kInvertDirectionPin = true;
// Home switch wired to PD1/TX with INPUT_PULLUP. Active level set in stepper_limits.h.
constexpr uint8_t kPinHomeSwitch = 1;
constexpr uint8_t kPinStatusLed = 13;

} // namespace config
} // namespace robot
