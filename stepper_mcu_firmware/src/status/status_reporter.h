#pragma once

#include <Arduino.h>

#include "protocol_types.h"

namespace robot
{
namespace status
{
class StatusReporter
{
public:
    protocol::I2cStatusFrame buildFrame(const protocol::StepperStatusPayload &status, uint8_t seq) const;
};

} // namespace status
} // namespace robot
