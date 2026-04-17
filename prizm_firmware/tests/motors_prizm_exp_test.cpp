#include <cassert>

#include "motors_prizm_exp.h"

namespace
{
void resetPrizmStubs()
{
    PRIZM::resetTestState();
    EXPANSION::resetTestState();
}

void testBeginMovesServo6ToOpposite90DegAndStopsCrServo1()
{
    resetPrizmStubs();

    MotorsPrizmExp motors(1, 2);
    motors.begin();

    assert(PRIZM::prizm_begin_calls == 1);
    assert(PRIZM::set_servo_positions_calls == 0);
    assert(PRIZM::set_servo_position_calls == 1);
    assert(PRIZM::last_servo_channel == 6);
    assert(PRIZM::servo_position_6 == 180);
    assert(PRIZM::set_cr_servo_state_calls == 1);
    assert(PRIZM::last_cr_servo_channel == 1);
    assert(PRIZM::last_cr_servo_speed == 0);
    assert(EXPANSION::set_motor_powers_calls == 2);
    assert(EXPANSION::reset_encoders_calls == 2);
}

void testRefreshStartupAuxOutputsReassertsServo6AndCrServo1()
{
    resetPrizmStubs();

    MotorsPrizmExp motors(1, 2);
    motors.begin();

    motors.refreshStartupAuxOutputs(249);
    assert(PRIZM::set_servo_position_calls == 1);
    assert(PRIZM::set_cr_servo_state_calls == 1);

    motors.refreshStartupAuxOutputs(250);
    assert(PRIZM::set_servo_position_calls == 2);
    assert(PRIZM::last_servo_channel == 6);
    assert(PRIZM::servo_position_6 == 180);
    assert(PRIZM::set_cr_servo_state_calls == 2);
    assert(PRIZM::last_cr_servo_channel == 1);
    assert(PRIZM::last_cr_servo_speed == 0);
}

void testStepperTargetReachedMovesServo6ToReversePositionAndHoldsIt()
{
    resetPrizmStubs();

    MotorsPrizmExp motors(1, 2);
    motors.begin();

    motors.onStepperTargetReached();
    assert(PRIZM::set_servo_position_calls == 2);
    assert(PRIZM::last_servo_channel == 6);
    assert(PRIZM::servo_position_6 == 0);
    assert(PRIZM::set_cr_servo_state_calls == 2);
    assert(PRIZM::last_cr_servo_channel == 1);
    assert(PRIZM::last_cr_servo_speed == 0);

    motors.refreshStartupAuxOutputs(250);
    assert(PRIZM::set_servo_position_calls == 3);
    assert(PRIZM::servo_position_6 == 0);
    assert(PRIZM::set_cr_servo_state_calls == 3);
    assert(PRIZM::last_cr_servo_speed == 0);
}
} // namespace

int main()
{
    testBeginMovesServo6ToOpposite90DegAndStopsCrServo1();
    testRefreshStartupAuxOutputsReassertsServo6AndCrServo1();
    testStepperTargetReachedMovesServo6ToReversePositionAndHoldsIt();
    return 0;
}
