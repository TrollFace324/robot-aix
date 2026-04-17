#include "stepper/stepper_driver_adapter.h"

#include "config/pin_map_stepper.h"

namespace robot
{
namespace stepper
{
void StepperDriverAdapter::begin(uint8_t pin_step, uint8_t pin_dir, uint8_t pin_enable, const MotionProfile &profile)
{
    pin_enable_ = pin_enable;
    profile_ = profile;
    if (pin_enable_ != kNoEnablePin)
    {
        pinMode(pin_enable_, OUTPUT);
        // Keep the driver disabled until the first real motion command so the MCU
        // can bring up TWI cleanly before the power stage is energized.
        digitalWrite(pin_enable_, HIGH);
    }

    driver_ = AccelStepper(AccelStepper::DRIVER, pin_step, pin_dir);
    driver_.setPinsInverted(config::kInvertDirectionPin, false, false);
    driver_.setMaxSpeed(profile_.max_speed_steps);
    driver_.setAcceleration(profile_.accel_steps);
    target_mm_ = 0.0f;
}

void StepperDriverAdapter::update()
{
    driver_.run();
}

void StepperDriverAdapter::enable(bool on)
{
    if (pin_enable_ == kNoEnablePin)
    {
        return;
    }

    digitalWrite(pin_enable_, on ? LOW : HIGH);
}

bool StepperDriverAdapter::enabled() const
{
    if (pin_enable_ == kNoEnablePin)
    {
        return true;
    }

    return digitalRead(pin_enable_) == LOW;
}

void StepperDriverAdapter::setTargetMm(float target_mm)
{
    target_mm_ = target_mm;
    const long target_steps = static_cast<long>(target_mm * profile_.steps_per_mm);
    driver_.moveTo(target_steps);
}

void StepperDriverAdapter::setCurrentMm(float current_mm)
{
    const long current_steps = static_cast<long>(current_mm * profile_.steps_per_mm);
    driver_.setCurrentPosition(current_steps);
    target_mm_ = current_mm;
    driver_.moveTo(current_steps);
}

float StepperDriverAdapter::currentMm()
{
    return static_cast<float>(driver_.currentPosition()) / profile_.steps_per_mm;
}

float StepperDriverAdapter::targetMm() const
{
    return target_mm_;
}

bool StepperDriverAdapter::isMoving()
{
    return driver_.distanceToGo() != 0;
}

} // namespace stepper
} // namespace robot
