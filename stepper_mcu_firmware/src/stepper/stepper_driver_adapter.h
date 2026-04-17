#pragma once

#include <Arduino.h>
#include <AccelStepper.h>

#include "motion_profile.h"

namespace robot
{
namespace stepper
{
class StepperDriverAdapter
{
public:
    static constexpr uint8_t kNoEnablePin = 0xFF;

    void begin(uint8_t pin_step, uint8_t pin_dir, uint8_t pin_enable, const MotionProfile &profile);
    void update();

    void enable(bool on);
    bool enabled() const;

    void setTargetMm(float target_mm);
    void setCurrentMm(float current_mm);
    float currentMm();
    float targetMm() const;

    bool isMoving();

private:
    AccelStepper driver_{AccelStepper::DRIVER, 2, 3};
    uint8_t pin_enable_ = kNoEnablePin;
    MotionProfile profile_{};
    float target_mm_ = 0.0f;
};

} // namespace stepper
} // namespace robot
