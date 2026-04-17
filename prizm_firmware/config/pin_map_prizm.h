#pragma once

#include <Arduino.h>

namespace robot
{
namespace config
{
// External select line is available on this hardware.
constexpr bool kSpiUseExternalCs = true;
// PRIZM external select line (user wiring): D2.
constexpr uint8_t kSpiCsPin = 2;
constexpr uint8_t kSpiHwSsPin = 10;
constexpr uint8_t kSpiMisoPin = 12;
constexpr uint8_t kSpiMosiPin = 11;
constexpr uint8_t kSpiSckPin = 13;

} // namespace config
} // namespace robot
