#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

constexpr uint8_t LOW = 0x0;
constexpr uint8_t HIGH = 0x1;
constexpr uint8_t INPUT = 0x0;
constexpr uint8_t OUTPUT = 0x1;
constexpr uint8_t INPUT_PULLUP = 0x2;

namespace test
{
namespace arduino
{
inline uint8_t g_pin_modes[256]{};
inline uint8_t g_pin_values[256]{};

inline void resetPins()
{
    std::memset(g_pin_modes, 0, sizeof(g_pin_modes));
    std::memset(g_pin_values, 0, sizeof(g_pin_values));
}
} // namespace arduino
} // namespace test

inline void pinMode(uint8_t pin, uint8_t mode)
{
    test::arduino::g_pin_modes[pin] = mode;
}

inline void digitalWrite(uint8_t pin, uint8_t value)
{
    test::arduino::g_pin_values[pin] = value;
}

inline int digitalRead(uint8_t pin)
{
    return test::arduino::g_pin_values[pin];
}
