#include "station/kran_fsm.h"

namespace robot
{
namespace station
{
bool KranFsm::readyForHome(const KranInputs &inputs) const
{
    // Home is the bootstrap path: we do NOT require the stepper to already be
    // homed or sitting on the switch, only that the system can accept commands.
    return inputs.start_gate_open &&
           inputs.prizm_summary_fresh &&
           inputs.stepper_online;
}

bool KranFsm::readyForCommand(const KranInputs &inputs) const
{
    return readyForHome(inputs) &&
           inputs.stepper_homed &&
           inputs.home_switch_active;
}

bool KranFsm::isIdleState() const
{
    return state_ == KranState::NotReady ||
           state_ == KranState::ReadyHome ||
           state_ == KranState::FaultLatched;
}

bool KranFsm::targetReached(const KranInputs &inputs) const
{
    return inputs.stepper_state == StepperMotionState::Idle &&
           inputs.current_position_mm == inputs.target_position_mm;
}

bool KranFsm::throwMotionObserved(const KranInputs &inputs)
{
    if (throw_motion_observed_)
    {
        return true;
    }

    throw_motion_observed_ =
        (inputs.stepper_state != StepperMotionState::Idle) ||
        (inputs.current_position_mm != throw_start_position_mm_) ||
        (inputs.target_position_mm != throw_start_target_mm_);

    return throw_motion_observed_;
}

KranOutputs KranFsm::update(const KranInputs &inputs, const KranActionRequest *request)
{
    KranOutputs out;

    if (inputs.stepper_state == StepperMotionState::Homing)
    {
        home_recovery_observed_ = true;
    }

    const bool stepper_faulted =
        inputs.stepper_fault || inputs.stepper_state == StepperMotionState::Fault;

    // After we dispatch CMD_HOME, the stepper takes a moment to clear
    // fault_flags in its main loop; the next I2C status poll may still show
    // the stale fault. Suppress the latch until the stepper observably
    // transitions into HOMING.
    const bool suppress_fault_latch =
        (state_ == KranState::HomeOnlyActive) && !home_recovery_observed_;

    if (stepper_faulted && !suppress_fault_latch)
    {
        state_ = KranState::FaultLatched;
        if (active_request_)
        {
            active_request_ = false;
            throw_motion_observed_ = false;
            out.emit_error = true;
            out.error_code = KranErrorCode::StepperFault;
            return out;
        }

        // A fresh Home request is allowed to drop through — CMD_HOME clears
        // fault_flags on the stepper, so this is the recovery path.
        const bool is_home_request =
            (request != nullptr) && (request->type == KranRequestType::Home);
        if (!is_home_request)
        {
            return out;
        }
    }

    if (!active_request_)
    {
        state_ = readyForCommand(inputs) ? KranState::ReadyHome : KranState::NotReady;
    }

    if (request != nullptr)
    {
        if (!isIdleState())
        {
            out.emit_error = true;
            out.error_code = KranErrorCode::Busy;
            return out;
        }

        const bool is_home_request =
            (request->type == KranRequestType::Home);
        const bool recovering_home = stepper_faulted && is_home_request;

        if (state_ == KranState::FaultLatched && !recovering_home)
        {
            out.emit_error = true;
            out.error_code = KranErrorCode::StepperFault;
            return out;
        }

        const bool request_ready =
            is_home_request ? readyForHome(inputs) : readyForCommand(inputs);

        if (!recovering_home && !request_ready)
        {
            state_ = KranState::NotReady;
            out.emit_error = true;
            out.error_code = KranErrorCode::NotReady;
            return out;
        }

        if (request->type == KranRequestType::Throw && request->slot > kMaxSlot)
        {
            out.emit_error = true;
            out.error_code = KranErrorCode::InvalidSlot;
            return out;
        }

        active_request_ = true;
        active_type_ = request->type;
        active_slot_ = request->slot;
        out.emit_first_response = true;

        if (request->type == KranRequestType::Home)
        {
            state_ = KranState::HomeOnlyActive;
            home_recovery_observed_ = false;
            throw_motion_observed_ = false;
            out.command = KranCommand::Home;
            return out;
        }

        state_ = KranState::ThrowExtend;
        throw_motion_observed_ = false;
        throw_start_position_mm_ = inputs.current_position_mm;
        throw_start_target_mm_ = inputs.target_position_mm;
        out.command = KranCommand::GotoSlot;
        out.command_arg = request->slot;
        return out;
    }

    switch (state_)
    {
    case KranState::HomeOnlyActive:
        if (inputs.stepper_homed && inputs.home_switch_active &&
            inputs.stepper_state == StepperMotionState::Idle)
        {
            active_request_ = false;
            state_ = KranState::ReadyHome;
            throw_motion_observed_ = false;
            out.emit_second_response = true;
        }
        break;

    case KranState::ThrowExtend:
        if (throwMotionObserved(inputs) && targetReached(inputs))
        {
            dwell_started_ms_ = inputs.now_ms;
            state_ = KranState::ThrowDwell;
        }
        break;

    case KranState::ThrowDwell:
        if (inputs.now_ms - dwell_started_ms_ >= kThrowDwellMs)
        {
            state_ = KranState::ThrowHome;
            out.command = KranCommand::Home;
        }
        break;

    case KranState::ThrowHome:
        if (inputs.stepper_homed && inputs.home_switch_active &&
            inputs.stepper_state == StepperMotionState::Idle)
        {
            active_request_ = false;
            state_ = KranState::ReadyHome;
            throw_motion_observed_ = false;
            out.emit_second_response = true;
        }
        break;

    case KranState::NotReady:
    case KranState::ReadyHome:
    case KranState::FaultLatched:
    default:
        break;
    }

    return out;
}

KranState KranFsm::state() const
{
    return state_;
}

bool KranFsm::hasActiveRequest() const
{
    return active_request_;
}

KranRequestType KranFsm::activeRequestType() const
{
    return active_type_;
}

uint8_t KranFsm::activeSlot() const
{
    return active_slot_;
}

} // namespace station
} // namespace robot
