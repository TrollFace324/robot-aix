#pragma once

#include <Arduino.h>

#include "protocol_types.h"

namespace robot
{
struct StepperStatus
{
    protocol::StepperStatusPayload payload;
};
} // namespace robot
