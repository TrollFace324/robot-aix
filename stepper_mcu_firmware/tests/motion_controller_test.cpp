#include <cassert>

#include "config/kran_slot_config.h"
#include "config/pin_map_stepper.h"
#include "config/stepper_limits.h"
#include "motion_profile.h"
#include "stepper/motion_controller.h"
#include "stepper/stepper_driver_adapter.h"

namespace
{
using robot::MotionProfile;
using robot::protocol::StepperCommand;
using robot::protocol::StepperConfigReplyType;
using robot::protocol::StepperConfigResult;
using robot::protocol::StepperStatusCode;
using robot::stepper::MotionController;
using robot::stepper::StepperDriverAdapter;

constexpr int16_t kTestMinMm = 0;
constexpr int16_t kTestMaxMm = 180;
constexpr int16_t kExpectedHomeBackoffMm = 10;

MotionController makeController()
{
    MotionController controller;
    MotionProfile profile;
    profile.steps_per_mm = 1.0f;
    controller.begin(robot::config::kPinStep,
                     robot::config::kPinDir,
                     robot::config::kPinEnable,
                     robot::config::kPinHomeSwitch,
                     profile,
                     kTestMinMm,
                     kTestMaxMm);
    return controller;
}

MotionController makeControllerWithMaxMm(int16_t max_mm)
{
    MotionController controller;
    MotionProfile profile;
    profile.steps_per_mm = 1.0f;
    controller.begin(robot::config::kPinStep,
                     robot::config::kPinDir,
                     robot::config::kPinEnable,
                     robot::config::kPinHomeSwitch,
                     profile,
                     kTestMinMm,
                     max_mm);
    return controller;
}

template <typename Predicate>
void runUntil(MotionController &controller, Predicate predicate, uint32_t max_iterations = 200)
{
    for (uint32_t i = 0; i < max_iterations; ++i)
    {
        controller.update(i);
        if (predicate(controller.status()))
        {
            return;
        }
    }

    assert(false && "condition not reached");
}

void testNoEnablePinIsAlwaysEnabled()
{
    test::arduino::resetPins();

    StepperDriverAdapter driver;
    MotionProfile profile;
    driver.begin(2, 3, StepperDriverAdapter::kNoEnablePin, profile);

    assert(driver.enabled());
    driver.enable(false);
    assert(driver.enabled());
}

void testEnablePinIsActiveLowForTmc2208()
{
    test::arduino::resetPins();

    StepperDriverAdapter driver;
    MotionProfile profile;
    driver.begin(2, 3, robot::config::kPinEnable, profile);

    assert(!driver.enabled());
    driver.enable(true);
    assert(driver.enabled());
    driver.enable(false);
    assert(!driver.enabled());
}

void testPositiveTravelKeepsHomeAtMinDirectionConvention()
{
    test::arduino::resetPins();

    StepperDriverAdapter driver;
    MotionProfile profile;
    profile.steps_per_mm = 1.0f;
    driver.begin(robot::config::kPinStep,
                 robot::config::kPinDir,
                 StepperDriverAdapter::kNoEnablePin,
                 profile);

    driver.setTargetMm(5.0f);
    driver.update();

    const int expected_dir_level =
        robot::config::kInvertDirectionPin ? LOW : HIGH;
    assert(digitalRead(robot::config::kPinDir) == expected_dir_level);
}

void testHomeSeeksSwitchWhenNcLineIsLow()
{
    test::arduino::resetPins();
    digitalWrite(robot::config::kPinHomeSwitch, HIGH);

    MotionController controller = makeController();
    controller.handleCommand(StepperCommand::CMD_HOME, 0, 0);

    controller.update(0);
    auto status = controller.status();
    assert(status.status == StepperStatusCode::STATUS_HOMING);
    assert(!status.is_homed);
    assert(status.current_position_mm > status.target_position_mm);
    assert(status.driver_enabled);

    digitalWrite(robot::config::kPinHomeSwitch, LOW);
    controller.update(1);
    status = controller.status();
    assert(status.status == StepperStatusCode::STATUS_IDLE);
    assert(status.is_homed);
    assert(status.current_position_mm == kTestMinMm);
    assert(status.target_position_mm == kTestMinMm);
}

void testHomeReleasesPressedSwitchThenReapproaches()
{
    test::arduino::resetPins();
    digitalWrite(robot::config::kPinHomeSwitch, LOW);

    MotionController controller = makeController();
    controller.handleCommand(StepperCommand::CMD_HOME, 0, 0);

    controller.update(0);
    auto status = controller.status();
    assert(status.status == StepperStatusCode::STATUS_HOMING);
    assert(!status.is_homed);
    assert(status.current_position_mm > 0);

    digitalWrite(robot::config::kPinHomeSwitch, HIGH);
    runUntil(controller, [](const auto &current_status) {
        return current_status.current_position_mm >= kExpectedHomeBackoffMm;
    });

    status = controller.status();
    assert(status.status == StepperStatusCode::STATUS_HOMING);
    assert(!status.is_homed);

    runUntil(controller, [](const auto &current_status) {
        return current_status.target_position_mm == kTestMinMm &&
               current_status.current_position_mm > kTestMinMm;
    });

    digitalWrite(robot::config::kPinHomeSwitch, LOW);
    controller.update(200);

    status = controller.status();
    assert(status.status == StepperStatusCode::STATUS_IDLE);
    assert(status.is_homed);
    assert(status.current_position_mm == kTestMinMm);
    assert(status.target_position_mm == kTestMinMm);
}

void testHomeErrorsWhenPressedSwitchNeverReleases()
{
    test::arduino::resetPins();
    digitalWrite(robot::config::kPinHomeSwitch, HIGH);

    MotionController controller = makeController();
    controller.handleCommand(StepperCommand::CMD_HOME, 0, 0);

    runUntil(controller, [](const auto &status) {
        return status.status == StepperStatusCode::STATUS_ERROR;
    });

    const auto status = controller.status();
    assert(!status.is_homed);
    assert(status.fault_flags != 0);
}

void testRepeatedHomeOnActiveSwitchDoesNotBackOffAgain()
{
    test::arduino::resetPins();
    digitalWrite(robot::config::kPinHomeSwitch, HIGH);

    MotionController controller = makeController();
    controller.handleCommand(StepperCommand::CMD_HOME, 0, 0);
    controller.update(0);
    digitalWrite(robot::config::kPinHomeSwitch, LOW);
    controller.update(1);

    auto status = controller.status();
    assert(status.status == StepperStatusCode::STATUS_IDLE);
    assert(status.is_homed);

    digitalWrite(robot::config::kPinHomeSwitch, LOW);
    controller.handleCommand(StepperCommand::CMD_HOME, 0, 0);
    controller.update(2);
    status = controller.status();
    assert(status.status == StepperStatusCode::STATUS_IDLE);
    assert(status.is_homed);
    assert(status.current_position_mm == kTestMinMm);
    assert(status.target_position_mm == kTestMinMm);
}

void testGotoSlotUsesStepperConfigTargets()
{
    test::arduino::resetPins();
    digitalWrite(robot::config::kPinHomeSwitch, HIGH);

    MotionController controller = makeController();
    controller.handleCommand(StepperCommand::CMD_GOTO_SLOT, 9, 0);
    controller.update(0);

    const auto status = controller.status();
    assert(status.status == StepperStatusCode::STATUS_MOVING);
    assert(status.target_position_mm == robot::config::kKranSlotTargetMm[9]);
}

void testGetSlotMmPublishesConfigReply()
{
    test::arduino::resetPins();

    MotionController controller = makeController();
    controller.handleCommand(StepperCommand::CMD_GET_SLOT_MM, 4, 0);

    const auto status = controller.status();
    assert(status.cfg_reply_type == StepperConfigReplyType::ReadSlot);
    assert(status.cfg_slot == 4);
    assert(status.cfg_mm == robot::config::kKranSlotTargetMm[4]);
    assert(status.cfg_result == StepperConfigResult::Ok);
}

void testSetSlotMmUpdatesRuntimeMapping()
{
    test::arduino::resetPins();
    digitalWrite(robot::config::kPinHomeSwitch, HIGH);

    MotionController controller = makeController();
    controller.handleCommand(StepperCommand::CMD_SET_SLOT_MM, 4, 17);

    auto status = controller.status();
    assert(status.cfg_reply_type == StepperConfigReplyType::WriteSlot);
    assert(status.cfg_slot == 4);
    assert(status.cfg_mm == 17);
    assert(status.cfg_result == StepperConfigResult::Ok);

    controller.handleCommand(StepperCommand::CMD_GOTO_SLOT, 4, 0);
    controller.update(0);
    status = controller.status();
    assert(status.target_position_mm == 17);
}

void testSetSlotMmRejectsOutOfRangeAndBusy()
{
    test::arduino::resetPins();
    digitalWrite(robot::config::kPinHomeSwitch, HIGH);

    MotionController controller = makeController();
    controller.handleCommand(StepperCommand::CMD_SET_SLOT_MM, 4, 999);
    auto status = controller.status();
    assert(status.cfg_reply_type == StepperConfigReplyType::WriteSlot);
    assert(status.cfg_result == StepperConfigResult::InvalidMm);

    controller.handleCommand(StepperCommand::CMD_GOTO_MM, 5, 0);
    controller.update(0);
    controller.handleCommand(StepperCommand::CMD_SET_SLOT_MM, 4, 12);
    status = controller.status();
    assert(status.cfg_reply_type == StepperConfigReplyType::WriteSlot);
    assert(status.cfg_result == StepperConfigResult::Busy);
}

void testSetSlotMmAcceptsConfiguredMaxTravel()
{
    test::arduino::resetPins();

    MotionController controller =
        makeControllerWithMaxMm(robot::config::kMaxTravelMm);
    controller.handleCommand(StepperCommand::CMD_SET_SLOT_MM,
                             4,
                             robot::config::kMaxTravelMm);

    const auto status = controller.status();
    assert(status.cfg_reply_type == StepperConfigReplyType::WriteSlot);
    assert(status.cfg_slot == 4);
    assert(status.cfg_mm == robot::config::kMaxTravelMm);
    assert(status.cfg_result == StepperConfigResult::Ok);
}
} // namespace

int main()
{
    testNoEnablePinIsAlwaysEnabled();
    testEnablePinIsActiveLowForTmc2208();
    testPositiveTravelKeepsHomeAtMinDirectionConvention();
    testHomeSeeksSwitchWhenNcLineIsLow();
    testHomeReleasesPressedSwitchThenReapproaches();
    testHomeErrorsWhenPressedSwitchNeverReleases();
    testRepeatedHomeOnActiveSwitchDoesNotBackOffAgain();
    testGotoSlotUsesStepperConfigTargets();
    testGetSlotMmPublishesConfigReply();
    testSetSlotMmUpdatesRuntimeMapping();
    testSetSlotMmRejectsOutOfRangeAndBusy();
    testSetSlotMmAcceptsConfiguredMaxTravel();
    return 0;
}
