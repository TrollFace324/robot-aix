#pragma once

#include <Arduino.h>

#include "system_state.h"

namespace robot
{
namespace startup
{
struct StartupInputs
{
    bool init_done = false;
    bool transport_online = false;
    bool handshake_ok = false;
    bool sensor_check_ok = false;
    bool sensor_check_pending = false;
    bool fault_detected = false;
};

struct StartupOutputs
{
    bool run_locked = true;
    bool allow_retry = false;
};

class StartupStateMachine
{
public:
    void begin(uint32_t now_ms);
    void update(uint32_t now_ms, const StartupInputs &inputs);

    StartupState state() const;
    uint8_t faultCode() const;
    const StartupOutputs &outputs() const;

private:
    void transitionTo(StartupState next, uint32_t now_ms);

    StartupState state_ = StartupState::BOOT;
    uint32_t state_enter_ms_ = 0;
    uint8_t fault_code_ = 0;
    StartupOutputs outputs_{};
};

} // namespace startup
} // namespace robot
