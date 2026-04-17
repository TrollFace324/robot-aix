#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace robot
{
class IImuService
{
public:
    virtual ~IImuService() = default;
    virtual bool begin(TwoWire &wire, uint8_t address) = 0;
    virtual void update(uint32_t now_ms) = 0;
    virtual bool healthy() const = 0;
    virtual bool dataValid() const = 0;
    virtual bool calibrating() const = 0;
    virtual bool tiltFault() const = 0;
    virtual uint32_t lastSampleMs() const = 0;
    virtual float yawDeg() const = 0;
    virtual float yawRateDps() const = 0;
    virtual float rollDeg() const = 0;
    virtual float pitchDeg() const = 0;
};
} // namespace robot
