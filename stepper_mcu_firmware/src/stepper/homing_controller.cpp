#include "stepper/homing_controller.h"

namespace robot
{
namespace stepper
{
void HomingController::begin(uint8_t pin_home_switch, bool active_low)
{
    pin_home_switch_ = pin_home_switch;
    active_low_ = active_low;
    pinMode(pin_home_switch_, INPUT_PULLUP);
}

bool HomingController::homeSwitchActive() const
{
    return digitalRead(pin_home_switch_) == (active_low_ ? LOW : HIGH);
}

} // namespace stepper
} // namespace robot
