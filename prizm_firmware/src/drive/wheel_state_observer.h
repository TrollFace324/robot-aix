#pragma once

#include <Arduino.h>

class MotorsPrizmExp;

namespace robot
{
namespace drive
{
enum class WheelIndex : uint8_t
{
    FL = 0,
    FR = 1,
    RL = 2,
    RR = 3
};

struct WheelTelemetry
{
    bool valid = false;
    int32_t count = 0;
    float speed_cps = 0.0f;
    uint32_t last_sample_ms = 0;
};

class WheelStateObserver
{
public:
    void configure(uint8_t poll_interval_ms, float motion_threshold_cps);
    void reset();
    void update(MotorsPrizmExp &motors, uint32_t now_ms);

    bool ready() const;
    float rotationalSpeedCountsPerSec() const;
    float rotationalBalance() const;
    bool rotationMotionDetected() const;

private:
    static uint8_t indexOf(WheelIndex wheel);
    bool allValid() const;

    WheelTelemetry wheels_[4]{};
    uint8_t next_wheel_ = 0;
    uint8_t poll_interval_ms_ = 3;
    float motion_threshold_cps_ = 16.0f;
    uint32_t last_poll_ms_ = 0;
    float rotational_speed_cps_ = 0.0f;
    float rotational_balance_ = 0.0f;
    bool rotation_motion_detected_ = false;
};

} // namespace drive
} // namespace robot
