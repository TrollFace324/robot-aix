#include <cassert>

#include "station/kran_fsm.h"

namespace
{
using robot::station::KranActionRequest;
using robot::station::KranCommand;
using robot::station::KranErrorCode;
using robot::station::KranFsm;
using robot::station::KranInputs;
using robot::station::KranRequestType;
using robot::station::KranState;
using robot::station::StepperMotionState;

KranInputs baseInputs()
{
    KranInputs in;
    in.now_ms = 100;
    in.start_gate_open = false;
    in.prizm_summary_fresh = false;
    in.stepper_online = false;
    in.stepper_homed = false;
    in.home_switch_active = false;
    in.stepper_fault = false;
    in.stepper_state = StepperMotionState::Idle;
    in.current_position_mm = 0;
    in.target_position_mm = 0;
    return in;
}

void testRejectsThrowBeforeReady()
{
    KranFsm fsm;
    const KranActionRequest request{KranRequestType::Throw, 4};
    const auto outputs = fsm.update(baseInputs(), &request);
    assert(outputs.emit_error);
    assert(outputs.error_code == KranErrorCode::NotReady);
    assert(fsm.state() == KranState::NotReady);
}

void testRejectsThrowWhenHomeSwitchIsNotActive()
{
    KranFsm fsm;
    KranInputs in = baseInputs();
    in.now_ms = 1000;
    in.start_gate_open = true;
    in.prizm_summary_fresh = true;
    in.stepper_online = true;
    in.stepper_homed = true;
    in.home_switch_active = false;

    const KranActionRequest request{KranRequestType::Throw, 5};
    const auto outputs = fsm.update(in, &request);
    assert(outputs.emit_error);
    assert(outputs.error_code == KranErrorCode::NotReady);
    assert(fsm.state() == KranState::NotReady);
}

void testThrowCycleTransitionsToDwellAndHome()
{
    KranFsm fsm;
    KranInputs in = baseInputs();
    in.now_ms = 1000;
    in.start_gate_open = true;
    in.prizm_summary_fresh = true;
    in.stepper_online = true;
    in.stepper_homed = true;
    in.home_switch_active = true;

    const KranActionRequest request{KranRequestType::Throw, 4};
    auto outputs = fsm.update(in, &request);
    assert(outputs.emit_first_response);
    assert(outputs.command == KranCommand::GotoSlot);
    assert(outputs.command_arg == 4);
    assert(fsm.state() == KranState::ThrowExtend);

    in.now_ms = 1300;
    in.stepper_state = StepperMotionState::Idle;
    in.home_switch_active = false;
    in.current_position_mm = 66;
    in.target_position_mm = 66;
    outputs = fsm.update(in, nullptr);
    assert(fsm.state() == KranState::ThrowDwell);
    assert(!outputs.emit_second_response);

    in.now_ms = 3305;
    outputs = fsm.update(in, nullptr);
    assert(outputs.command == KranCommand::Home);
    assert(fsm.state() == KranState::ThrowHome);

    in.now_ms = 3600;
    in.stepper_homed = true;
    in.home_switch_active = true;
    outputs = fsm.update(in, nullptr);
    assert(outputs.emit_second_response);
    assert(fsm.state() == KranState::ReadyHome);
}

void testThrowWaitsForObservedMotionBeforeStartingDwell()
{
    KranFsm fsm;
    KranInputs in = baseInputs();
    in.now_ms = 1000;
    in.start_gate_open = true;
    in.prizm_summary_fresh = true;
    in.stepper_online = true;
    in.stepper_homed = true;
    in.home_switch_active = true;

    const KranActionRequest request{KranRequestType::Throw, 4};
    auto outputs = fsm.update(in, &request);
    assert(outputs.emit_first_response);
    assert(outputs.command == KranCommand::GotoSlot);
    assert(fsm.state() == KranState::ThrowExtend);

    in.now_ms = 1100;
    outputs = fsm.update(in, nullptr);
    assert(fsm.state() == KranState::ThrowExtend);
    assert(outputs.command == KranCommand::None);
    assert(!outputs.emit_second_response);

    in.now_ms = 1300;
    in.stepper_state = StepperMotionState::Moving;
    in.home_switch_active = false;
    in.current_position_mm = 10;
    in.target_position_mm = 66;
    outputs = fsm.update(in, nullptr);
    assert(fsm.state() == KranState::ThrowExtend);

    in.now_ms = 1600;
    in.stepper_state = StepperMotionState::Idle;
    in.current_position_mm = 66;
    in.target_position_mm = 66;
    outputs = fsm.update(in, nullptr);
    assert(fsm.state() == KranState::ThrowDwell);
    assert(outputs.command == KranCommand::None);
}

void testThrowUsesConfiguredSlotMapping()
{
    KranFsm fsm;
    KranInputs in = baseInputs();
    in.now_ms = 1000;
    in.start_gate_open = true;
    in.prizm_summary_fresh = true;
    in.stepper_online = true;
    in.stepper_homed = true;
    in.home_switch_active = true;

    const KranActionRequest request{KranRequestType::Throw, 10};
    const auto outputs = fsm.update(in, &request);
    assert(outputs.emit_first_response);
    assert(outputs.command == KranCommand::GotoSlot);
    assert(outputs.command_arg == 10);
}

void testRejectsBusyRequestWhileCycleActive()
{
    KranFsm fsm;
    KranInputs in = baseInputs();
    in.now_ms = 1000;
    in.start_gate_open = true;
    in.prizm_summary_fresh = true;
    in.stepper_online = true;
    in.stepper_homed = true;
    in.home_switch_active = true;

    const KranActionRequest throw_request{KranRequestType::Throw, 3};
    (void)fsm.update(in, &throw_request);

    const KranActionRequest home_request{KranRequestType::Home, 0};
    const auto outputs = fsm.update(in, &home_request);
    assert(outputs.emit_error);
    assert(outputs.error_code == KranErrorCode::Busy);
}
} // namespace

int main()
{
    testRejectsThrowBeforeReady();
    testRejectsThrowWhenHomeSwitchIsNotActive();
    testThrowCycleTransitionsToDwellAndHome();
    testThrowWaitsForObservedMotionBeforeStartingDwell();
    testThrowUsesConfiguredSlotMapping();
    testRejectsBusyRequestWhileCycleActive();
    return 0;
}
