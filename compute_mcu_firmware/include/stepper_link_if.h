#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "protocol_types.h"

namespace robot
{
class IStepperLink
{
public:
    virtual ~IStepperLink() = default;
    virtual void begin(TwoWire &wire, uint8_t i2c_address) = 0;
    virtual bool requestStatus(uint32_t now_ms) = 0;
    virtual bool sendCommand(protocol::StepperCommand cmd, int16_t arg0, int16_t arg1) = 0;
    virtual bool online() const = 0;
    virtual protocol::StepperStatusPayload status() const = 0;
};
} // namespace robot
