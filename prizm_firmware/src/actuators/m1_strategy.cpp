#include "actuators/m1_strategy.h"

namespace robot
{
namespace actuators
{
void TimedRunStrategy::trigger(int16_t power, uint32_t duration_ms, uint32_t now_ms)
{
    power_ = power;
    until_ms_ = now_ms + duration_ms;
}

int16_t TimedRunStrategy::update(uint32_t now_ms)
{
    if (power_ == 0)
    {
        return 0;
    }
    if (now_ms >= until_ms_)
    {
        power_ = 0;
    }
    return power_;
}

void CurrentThresholdStrategy::trigger(int16_t power, uint16_t threshold_ma)
{
    power_ = power;
    threshold_ma_ = threshold_ma;
}

void CurrentThresholdStrategy::setCurrent(uint16_t current_ma)
{
    current_ma_ = current_ma;
}

int16_t CurrentThresholdStrategy::update(uint32_t)
{
    if (threshold_ma_ > 0 && current_ma_ >= threshold_ma_)
    {
        power_ = 0;
    }
    return power_;
}

void ReturnToHomeStrategy::trigger(int16_t power_to_home)
{
    power_ = power_to_home;
}

void ReturnToHomeStrategy::setHomeState(bool active)
{
    home_active_ = active;
}

int16_t ReturnToHomeStrategy::update(uint32_t)
{
    if (home_active_)
    {
        power_ = 0;
    }
    return power_;
}

} // namespace actuators
} // namespace robot
