#pragma once

namespace robot
{
namespace config
{
constexpr uint8_t kSpiMisoPin = 12;
constexpr uint8_t kSpiMosiPin = 11;
constexpr uint8_t kSpiSckPin = 13;
constexpr uint8_t kStatusLedPin = 9;

static_assert(kStatusLedPin != 10 && kStatusLedPin != 11 &&
                  kStatusLedPin != 12 && kStatusLedPin != 13,
              "kStatusLedPin must not overlap SPI pins D10..D13 on ATmega328P");

} // namespace config
} // namespace robot
