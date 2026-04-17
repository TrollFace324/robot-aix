#pragma once

#include "station/kran_fsm.h"

namespace robot
{
namespace app
{
struct AutoHomeInputs
{
    bool prizm_summary_fresh = false;
    bool start_gate_open = false;
    bool stepper_online = false;
    bool stepper_fault = false;
    bool kran_request_active = false;
    station::StepperMotionState stepper_state = station::StepperMotionState::Idle;
};

class AutoHomeSession
{
public:
    void reset()
    {
        prev_start_gate_open_ = false;
        pending_home_ = false;
        home_issued_in_session_ = false;
    }

    void update(bool start_gate_open, bool stepper_fault,
                station::StepperMotionState stepper_state)
    {
        if (start_gate_open && !prev_start_gate_open_)
        {
            pending_home_ = true;
            home_issued_in_session_ = false;
        }
        else if (!start_gate_open)
        {
            pending_home_ = false;
            home_issued_in_session_ = false;
        }

        prev_start_gate_open_ = start_gate_open;
    }

    bool shouldIssueHome(const AutoHomeInputs &inputs) const
    {
        return pending_home_ &&
               !home_issued_in_session_ &&
               inputs.prizm_summary_fresh &&
               inputs.start_gate_open &&
               inputs.stepper_online &&
               !inputs.kran_request_active &&
               (inputs.stepper_state == station::StepperMotionState::Idle ||
                inputs.stepper_state == station::StepperMotionState::Fault);
    }

    void markHomeIssued()
    {
        pending_home_ = false;
        home_issued_in_session_ = true;
    }

private:
    bool prev_start_gate_open_ = false;
    bool pending_home_ = false;
    bool home_issued_in_session_ = false;
};

} // namespace app
} // namespace robot
