#pragma once

#include <Arduino.h>

#include "system_state.h"

namespace robot
{
namespace status
{
struct LedOutput
{
    bool green = false;
    bool red = false;
};

class LedStatusManager
{
public:
    LedOutput evaluate(uint32_t now_ms,
                       startup::StartupState startup_state,
                       bool rc_fresh,
                       bool autonomous_active) const;

private:
    static bool pulse(uint32_t now_ms, uint16_t period_ms, uint16_t on_ms);
};

} // namespace status
} // namespace robot
