#include "status/led_status_manager.h"

namespace robot
{
namespace status
{
bool LedStatusManager::pulse(uint32_t now_ms, uint16_t period_ms, uint16_t on_ms)
{
    if (period_ms == 0 || on_ms == 0)
    {
        return false;
    }

    const uint16_t phase = static_cast<uint16_t>(now_ms % period_ms);
    return phase < on_ms;
}

LedOutput LedStatusManager::evaluate(uint32_t now_ms,
                                     startup::StartupState startup_state,
                                     bool rc_fresh,
                                     bool autonomous_active) const
{
    LedOutput out;

    switch (startup_state)
    {
    case startup::StartupState::BOOT:
        // Yellow short heartbeat: firmware is alive and booting.
        out.green = pulse(now_ms, 1000, 120);
        out.red = out.green;
        break;

    case startup::StartupState::INIT:
        // Slow red blink: local initialization.
        out.green = false;
        out.red = pulse(now_ms, 800, 180);
        break;

    case startup::StartupState::LINK_CHECK:
    {
        // Alternating red/green: waiting compute link handshake.
        const bool phase = pulse(now_ms, 300, 150);
        out.green = phase;
        out.red = !phase;
        break;
    }

    case startup::StartupState::SENSOR_CHECK:
        // Fast yellow blink: remote self-check in progress.
        out.green = pulse(now_ms, 220, 110);
        out.red = out.green;
        break;

    case startup::StartupState::READY:
        if (!rc_fresh)
        {
            // Green blink: system ready but RC stream stale/failsafe.
            out.green = pulse(now_ms, 500, 120);
            out.red = false;
        }
        else if (autonomous_active)
        {
            // Yellow double pulse: autonomous mode active.
            const bool p1 = pulse(now_ms, 900, 90);
            const bool p2 = pulse(now_ms + 160, 900, 90);
            out.green = p1 || p2;
            out.red = out.green;
        }
        else
        {
            // Solid green: ready + manual control.
            out.green = true;
            out.red = false;
        }
        break;

    case startup::StartupState::FAULT_LOCK:
        // Fast red blink: motion locked by startup fault.
        out.green = false;
        out.red = pulse(now_ms, 180, 90);
        break;
    }

    return out;
}

} // namespace status
} // namespace robot
