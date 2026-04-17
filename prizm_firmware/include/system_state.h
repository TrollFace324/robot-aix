#pragma once

#include <Arduino.h>

namespace robot
{
namespace startup
{
enum class StartupState : uint8_t
{
    BOOT = 0,
    INIT = 1,
    LINK_CHECK = 2,
    SENSOR_CHECK = 3,
    READY = 4,
    FAULT_LOCK = 5
};

} // namespace startup
} // namespace robot
