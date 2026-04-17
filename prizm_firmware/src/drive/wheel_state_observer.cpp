#include "drive/wheel_state_observer.h"

#include <math.h>

#include "motors_prizm_exp.h"

namespace robot
{
namespace drive
{
void WheelStateObserver::configure(uint8_t poll_interval_ms, float motion_threshold_cps)
{
    poll_interval_ms_ = (poll_interval_ms == 0) ? 1 : poll_interval_ms;
    motion_threshold_cps_ = motion_threshold_cps;
}

void WheelStateObserver::reset()
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        wheels_[i] = WheelTelemetry{};
    }
    next_wheel_ = 0;
    last_poll_ms_ = 0;
    rotational_speed_cps_ = 0.0f;
    rotational_balance_ = 0.0f;
    rotation_motion_detected_ = false;
}

void WheelStateObserver::update(MotorsPrizmExp &motors, uint32_t now_ms)
{
    if (last_poll_ms_ != 0 && (now_ms - last_poll_ms_) < poll_interval_ms_)
    {
        return;
    }
    last_poll_ms_ = now_ms;

    const WheelIndex wheel = static_cast<WheelIndex>(next_wheel_);
    next_wheel_ = static_cast<uint8_t>((next_wheel_ + 1U) % 4U);

    int32_t count = 0;
    if (!motors.readEncoder(static_cast<MotorsPrizmExp::Wheel>(wheel), count))
    {
        return;
    }

    WheelTelemetry &telemetry = wheels_[indexOf(wheel)];
    if (telemetry.valid)
    {
        const uint32_t dt_ms = now_ms - telemetry.last_sample_ms;
        if (dt_ms > 0)
        {
            const int32_t delta = count - telemetry.count;
            const float speed_cps = static_cast<float>(delta) * (1000.0f / static_cast<float>(dt_ms));
            telemetry.speed_cps = (telemetry.speed_cps * 0.65f) + (speed_cps * 0.35f);
        }
    }
    telemetry.valid = true;
    telemetry.count = count;
    telemetry.last_sample_ms = now_ms;

    if (!allValid())
    {
        return;
    }

    const float fl = wheels_[indexOf(WheelIndex::FL)].speed_cps;
    const float fr = wheels_[indexOf(WheelIndex::FR)].speed_cps;
    const float rl = wheels_[indexOf(WheelIndex::RL)].speed_cps;
    const float rr = wheels_[indexOf(WheelIndex::RR)].speed_cps;

    rotational_speed_cps_ = (fl - fr + rl - rr) * 0.25f;

    const float left_mag = fabsf(fl) + fabsf(rl);
    const float right_mag = fabsf(fr) + fabsf(rr);
    const float sum_mag = left_mag + right_mag;
    rotational_balance_ = (sum_mag > 1.0f) ? ((left_mag - right_mag) / sum_mag) : 0.0f;

    rotation_motion_detected_ = fabsf(rotational_speed_cps_) >= motion_threshold_cps_;
}

bool WheelStateObserver::ready() const
{
    return allValid();
}

float WheelStateObserver::rotationalSpeedCountsPerSec() const
{
    return rotational_speed_cps_;
}

float WheelStateObserver::rotationalBalance() const
{
    return rotational_balance_;
}

bool WheelStateObserver::rotationMotionDetected() const
{
    return rotation_motion_detected_;
}

uint8_t WheelStateObserver::indexOf(WheelIndex wheel)
{
    return static_cast<uint8_t>(wheel);
}

bool WheelStateObserver::allValid() const
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (!wheels_[i].valid)
        {
            return false;
        }
    }
    return true;
}

} // namespace drive
} // namespace robot
