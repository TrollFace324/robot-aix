#pragma once

#include <Arduino.h>

namespace robot
{
namespace control
{
enum class Mode : uint8_t
{
    MANUAL = 0,
    AUTONOMOUS = 1,
    SERVICE = 2
};

struct ModeInputs
{
    bool run_locked = true;
    bool ch10_high = false;
    bool ch3_high = false;
    bool sticks_neutral = true;
    bool rc_fresh = false;
    bool operator_override = false;
};

class ModeManager
{
public:
    void begin();
    void update(uint32_t now_ms, const ModeInputs &inputs);
    Mode mode() const;

private:
    Mode mode_ = Mode::MANUAL;
    uint32_t combo_start_ms_ = 0;
    bool combo_latched_ = false;
};

} // namespace control
} // namespace robot
