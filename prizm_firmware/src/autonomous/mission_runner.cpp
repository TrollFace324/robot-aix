#include "autonomous/mission_runner.h"

namespace robot
{
namespace autonomous
{
namespace
{
MissionStep kNullStep{};
}

void MissionRunner::clear()
{
    count_ = 0;
    index_ = 0;
    active_ = false;
    step_enter_ms_ = 0;
}

bool MissionRunner::push(const MissionStep &step)
{
    if (count_ >= kMaxSteps)
    {
        return false;
    }
    steps_[count_++] = step;
    return true;
}

void MissionRunner::start(uint32_t now_ms)
{
    if (count_ == 0)
    {
        return;
    }
    index_ = 0;
    active_ = true;
    step_enter_ms_ = now_ms;
}

void MissionRunner::stop()
{
    active_ = false;
}

void MissionRunner::update(uint32_t now_ms)
{
    if (!active_ || index_ >= count_)
    {
        return;
    }

    const MissionStep &step = steps_[index_];
    const uint32_t elapsed = now_ms - step_enter_ms_;
    const uint32_t limit = (step.timeout_ms > 0) ? step.timeout_ms : step.duration_ms;
    if (limit > 0 && elapsed >= limit)
    {
        ++index_;
        step_enter_ms_ = now_ms;
        if (index_ >= count_)
        {
            active_ = false;
        }
    }
}

bool MissionRunner::active() const
{
    return active_;
}

const MissionStep &MissionRunner::currentStep() const
{
    if (!active_ || index_ >= count_)
    {
        return kNullStep;
    }
    return steps_[index_];
}

} // namespace autonomous
} // namespace robot
