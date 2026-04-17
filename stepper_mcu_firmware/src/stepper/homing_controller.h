#pragma once

#include <Arduino.h>

namespace robot
{
namespace stepper
{
class HomingController
{
public:
    void begin(uint8_t pin_home_switch, bool active_low);
    bool homeSwitchActive() const;

private:
    uint8_t pin_home_switch_ = 5;
    bool active_low_ = true;
};

} // namespace stepper
} // namespace robot
