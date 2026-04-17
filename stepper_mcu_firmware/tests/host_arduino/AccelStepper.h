#pragma once

#include "Arduino.h"

#include <cstdint>
#include <cstdlib>

class AccelStepper
{
public:
    static constexpr uint8_t DRIVER = 1;

    AccelStepper(uint8_t = DRIVER, uint8_t pin_step = 0, uint8_t pin_dir = 0)
        : pin_step_(pin_step), pin_dir_(pin_dir)
    {
    }

    void setMaxSpeed(float)
    {
    }

    void setAcceleration(float)
    {
    }

    void setPinsInverted(bool directionInvert = false, bool stepInvert = false, bool = false)
    {
        direction_inverted_ = directionInvert;
        step_inverted_ = stepInvert;
    }

    void moveTo(long target)
    {
        target_ = target;
    }

    void setCurrentPosition(long current)
    {
        current_ = current;
    }

    long currentPosition() const
    {
        return current_;
    }

    long distanceToGo() const
    {
        return target_ - current_;
    }

    void run()
    {
        if (current_ < target_)
        {
            writeDirection(/*clockwise=*/true);
            pulseStep();
            ++current_;
        }
        else if (current_ > target_)
        {
            writeDirection(/*clockwise=*/false);
            pulseStep();
            --current_;
        }
    }

private:
    void writeDirection(bool clockwise)
    {
        const uint8_t level = (clockwise ^ direction_inverted_) ? HIGH : LOW;
        digitalWrite(pin_dir_, level);
    }

    void pulseStep()
    {
        digitalWrite(pin_step_, step_inverted_ ? LOW : HIGH);
        digitalWrite(pin_step_, step_inverted_ ? HIGH : LOW);
    }

    uint8_t pin_step_ = 0;
    uint8_t pin_dir_ = 0;
    bool direction_inverted_ = false;
    bool step_inverted_ = false;
    long current_ = 0;
    long target_ = 0;
};
