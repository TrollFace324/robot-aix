#include "app/startup_state_machine.h"

namespace
{
constexpr uint32_t kInitTimeoutMs = 2000;
constexpr uint32_t kLinkCheckTimeoutMs = 3000;
constexpr uint32_t kSensorCheckTimeoutMs = 3000;
constexpr uint32_t kSensorCheckPendingTimeoutMs = 15000;
}

namespace robot
{
namespace startup
{
void StartupStateMachine::begin(uint32_t now_ms)
{
    state_ = StartupState::BOOT;
    state_enter_ms_ = now_ms;
    fault_code_ = 0;
    outputs_ = StartupOutputs{};
}

void StartupStateMachine::update(uint32_t now_ms, const StartupInputs &inputs)
{
    if (inputs.fault_detected)
    {
        fault_code_ = 1;
        transitionTo(StartupState::FAULT_LOCK, now_ms);
    }

    switch (state_)
    {
    case StartupState::BOOT:
        outputs_.run_locked = true;
        if (inputs.init_done)
        {
            transitionTo(StartupState::INIT, now_ms);
        }
        break;

    case StartupState::INIT:
        outputs_.run_locked = true;
        if (inputs.transport_online)
        {
            transitionTo(StartupState::LINK_CHECK, now_ms);
        }
        else if (now_ms - state_enter_ms_ > kInitTimeoutMs)
        {
            // Missing compute transport is recoverable. Keep retrying instead of
            // latching FAULT_LOCK, so PRIZM can reconnect automatically.
            state_enter_ms_ = now_ms;
        }
        break;

    case StartupState::LINK_CHECK:
        outputs_.run_locked = true;
        if (!inputs.transport_online)
        {
            transitionTo(StartupState::INIT, now_ms);
            break;
        }
        if (inputs.handshake_ok)
        {
            transitionTo(StartupState::SENSOR_CHECK, now_ms);
        }
        else if (now_ms - state_enter_ms_ > kLinkCheckTimeoutMs)
        {
            // Handshake timeout is also recoverable while the link is coming up.
            state_enter_ms_ = now_ms;
        }
        break;

    case StartupState::SENSOR_CHECK:
        outputs_.run_locked = true;
        if (!inputs.transport_online)
        {
            transitionTo(StartupState::INIT, now_ms);
            break;
        }
        if (!inputs.handshake_ok)
        {
            transitionTo(StartupState::LINK_CHECK, now_ms);
            break;
        }
        if (inputs.sensor_check_ok)
        {
            transitionTo(StartupState::READY, now_ms);
            outputs_.run_locked = false;
        }
        else
        {
            const uint32_t timeout_ms =
                inputs.sensor_check_pending ? kSensorCheckPendingTimeoutMs : kSensorCheckTimeoutMs;
            if (now_ms - state_enter_ms_ > timeout_ms)
            {
                // Waiting for compute self-check should not permanently fault the
                // PRIZM controller. Keep retrying until the remote side recovers.
                state_enter_ms_ = now_ms;
            }
        }
        break;

    case StartupState::READY:
        if (!inputs.transport_online)
        {
            outputs_.run_locked = true;
            transitionTo(StartupState::INIT, now_ms);
        }
        else if (!inputs.handshake_ok)
        {
            outputs_.run_locked = true;
            transitionTo(StartupState::LINK_CHECK, now_ms);
        }
        else if (!inputs.sensor_check_ok)
        {
            outputs_.run_locked = true;
            transitionTo(StartupState::SENSOR_CHECK, now_ms);
        }
        else
        {
            outputs_.run_locked = false;
        }
        break;

    case StartupState::FAULT_LOCK:
        outputs_.run_locked = true;
        outputs_.allow_retry = true;
        if (!inputs.fault_detected)
        {
            fault_code_ = 0;
            if (!inputs.transport_online)
            {
                transitionTo(StartupState::INIT, now_ms);
            }
            else if (!inputs.handshake_ok)
            {
                transitionTo(StartupState::LINK_CHECK, now_ms);
            }
            else if (!inputs.sensor_check_ok)
            {
                transitionTo(StartupState::SENSOR_CHECK, now_ms);
            }
            else
            {
                transitionTo(StartupState::READY, now_ms);
                outputs_.run_locked = false;
            }
        }
        break;
    }
}

StartupState StartupStateMachine::state() const
{
    return state_;
}

uint8_t StartupStateMachine::faultCode() const
{
    return fault_code_;
}

const StartupOutputs &StartupStateMachine::outputs() const
{
    return outputs_;
}

void StartupStateMachine::transitionTo(StartupState next, uint32_t now_ms)
{
    if (state_ == next)
    {
        return;
    }

    state_ = next;
    state_enter_ms_ = now_ms;
    if (next != StartupState::FAULT_LOCK)
    {
        outputs_.allow_retry = false;
    }
}

} // namespace startup
} // namespace robot
