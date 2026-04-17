#pragma once

#include <Arduino.h>

namespace robot
{
namespace autonomous
{
enum class MissionStepType : uint8_t
{
    NONE = 0,
    DRIVE_VECTOR = 1,
    ROTATE_REL = 2,
    STOP = 3,
    STEPPER_GOTO = 4
};

struct MissionStep
{
    MissionStepType type = MissionStepType::NONE;
    float a = 0.0f;
    float b = 0.0f;
    uint32_t duration_ms = 0;
    uint32_t timeout_ms = 0;
};

class MissionRunner
{
public:
    static constexpr uint8_t kMaxSteps = 16;

    void clear();
    bool push(const MissionStep &step);
    void start(uint32_t now_ms);
    void stop();
    void update(uint32_t now_ms);
    bool active() const;
    const MissionStep &currentStep() const;

private:
    MissionStep steps_[kMaxSteps]{};
    uint8_t count_ = 0;
    uint8_t index_ = 0;
    bool active_ = false;
    uint32_t step_enter_ms_ = 0;
};

} // namespace autonomous
} // namespace robot
