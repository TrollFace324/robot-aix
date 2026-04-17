#pragma once

#include <Arduino.h>

namespace robot
{
namespace actuators
{
class IM1Strategy
{
public:
    virtual ~IM1Strategy() = default;
    virtual int16_t update(uint32_t now_ms) = 0;
};

class TimedRunStrategy : public IM1Strategy
{
public:
    void trigger(int16_t power, uint32_t duration_ms, uint32_t now_ms);
    int16_t update(uint32_t now_ms) override;

private:
    int16_t power_ = 0;
    uint32_t until_ms_ = 0;
};

class CurrentThresholdStrategy : public IM1Strategy
{
public:
    void trigger(int16_t power, uint16_t threshold_ma);
    void setCurrent(uint16_t current_ma);
    int16_t update(uint32_t now_ms) override;

private:
    int16_t power_ = 0;
    uint16_t threshold_ma_ = 0;
    uint16_t current_ma_ = 0;
};

class ReturnToHomeStrategy : public IM1Strategy
{
public:
    void trigger(int16_t power_to_home);
    void setHomeState(bool active);
    int16_t update(uint32_t now_ms) override;

private:
    int16_t power_ = 0;
    bool home_active_ = false;
};

} // namespace actuators
} // namespace robot
