#include <cassert>

#include "app/auto_home_session.h"

namespace
{
using robot::app::AutoHomeInputs;
using robot::app::AutoHomeSession;
using robot::station::StepperMotionState;

AutoHomeInputs readyInputs()
{
    AutoHomeInputs inputs;
    inputs.prizm_summary_fresh = true;
    inputs.start_gate_open = true;
    inputs.stepper_online = true;
    inputs.stepper_fault = false;
    inputs.kran_request_active = false;
    inputs.stepper_state = StepperMotionState::Idle;
    return inputs;
}

void updateIdle(AutoHomeSession &session, bool start_gate_open)
{
    session.update(start_gate_open, false, StepperMotionState::Idle);
}

void testIssuesHomeOncePerSession()
{
    AutoHomeSession session;
    AutoHomeInputs inputs = readyInputs();

    updateIdle(session, false);
    assert(!session.shouldIssueHome(inputs));

    updateIdle(session, true);
    assert(session.shouldIssueHome(inputs));

    session.markHomeIssued();
    assert(!session.shouldIssueHome(inputs));

    updateIdle(session, true);
    assert(!session.shouldIssueHome(inputs));
}

void testDelaysHomeUntilConditionsAreSafe()
{
    AutoHomeSession session;
    AutoHomeInputs inputs = readyInputs();

    updateIdle(session, true);
    inputs.stepper_online = false;
    assert(!session.shouldIssueHome(inputs));

    inputs.stepper_online = true;
    inputs.stepper_state = StepperMotionState::Moving;
    assert(!session.shouldIssueHome(inputs));

    inputs.stepper_state = StepperMotionState::Idle;
    inputs.kran_request_active = true;
    assert(!session.shouldIssueHome(inputs));

    inputs.kran_request_active = false;
    assert(session.shouldIssueHome(inputs));
}

void testNewSessionAllowsHomeAgain()
{
    AutoHomeSession session;
    AutoHomeInputs inputs = readyInputs();

    updateIdle(session, true);
    assert(session.shouldIssueHome(inputs));
    session.markHomeIssued();

    updateIdle(session, false);
    inputs.start_gate_open = false;
    assert(!session.shouldIssueHome(inputs));

    updateIdle(session, true);
    inputs.start_gate_open = true;
    assert(session.shouldIssueHome(inputs));
}

void testDoesNotRetryAfterStepperFaultInSameSession()
{
    AutoHomeSession session;
    AutoHomeInputs inputs = readyInputs();

    updateIdle(session, true);
    assert(session.shouldIssueHome(inputs));
    session.markHomeIssued();
    assert(!session.shouldIssueHome(inputs));

    // Stepper fault should not create an endless auto-home loop inside the same
    // start-gate session. Recovery requires a new session edge or operator action.
    session.update(true, true, StepperMotionState::Fault);
    inputs.stepper_fault = true;
    inputs.stepper_state = StepperMotionState::Fault;
    assert(!session.shouldIssueHome(inputs));
}
} // namespace

int main()
{
    testIssuesHomeOncePerSession();
    testDelaysHomeUntilConditionsAreSafe();
    testNewSessionAllowsHomeAgain();
    testDoesNotRetryAfterStepperFaultInSameSession();
    return 0;
}
