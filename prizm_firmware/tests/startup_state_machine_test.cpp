#include <cassert>
#include <cstdint>

#include "app/startup_state_machine.h"

namespace
{
using robot::startup::StartupInputs;
using robot::startup::StartupState;
using robot::startup::StartupStateMachine;

void expectState(const StartupStateMachine &sm,
                 StartupState expected_state,
                 bool expected_run_locked,
                 uint8_t expected_fault_code = 0)
{
    assert(sm.state() == expected_state);
    assert(sm.outputs().run_locked == expected_run_locked);
    assert(sm.faultCode() == expected_fault_code);
}

} // namespace

int main()
{
    StartupStateMachine sm;
    StartupInputs inputs;

    sm.begin(0);
    sm.update(1, inputs);
    expectState(sm, StartupState::BOOT, true);

    inputs.init_done = true;
    sm.update(10, inputs);
    expectState(sm, StartupState::INIT, true);

    sm.update(2505, inputs);
    expectState(sm, StartupState::INIT, true);

    inputs.transport_online = true;
    sm.update(2600, inputs);
    expectState(sm, StartupState::LINK_CHECK, true);

    sm.update(5705, inputs);
    expectState(sm, StartupState::LINK_CHECK, true);

    inputs.handshake_ok = true;
    sm.update(5800, inputs);
    expectState(sm, StartupState::SENSOR_CHECK, true);

    sm.update(8905, inputs);
    expectState(sm, StartupState::SENSOR_CHECK, true);

    inputs.sensor_check_ok = true;
    sm.update(9000, inputs);
    expectState(sm, StartupState::READY, false);

    inputs.transport_online = false;
    inputs.handshake_ok = false;
    inputs.sensor_check_ok = false;
    sm.update(9100, inputs);
    expectState(sm, StartupState::INIT, true);

    inputs.transport_online = true;
    inputs.handshake_ok = true;
    inputs.fault_detected = true;
    sm.update(9200, inputs);
    expectState(sm, StartupState::FAULT_LOCK, true, 1);

    inputs.fault_detected = false;
    sm.update(9300, inputs);
    expectState(sm, StartupState::SENSOR_CHECK, true);

    inputs.sensor_check_ok = true;
    sm.update(9400, inputs);
    expectState(sm, StartupState::READY, false);

    return 0;
}
