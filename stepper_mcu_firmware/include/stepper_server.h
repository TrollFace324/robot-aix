#pragma once

#include <Arduino.h>

#include "stepper_status.h"

namespace robot
{
class StepperServer
{
public:
    void begin();
    void loop();

    const StepperStatus &status() const;

private:
    StepperStatus status_{};
};

} // namespace robot
