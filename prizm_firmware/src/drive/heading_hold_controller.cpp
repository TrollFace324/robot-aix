#include "drive/heading_hold_controller.h"

#include <math.h>

namespace robot
{
namespace drive
{
namespace
{
float clampfLocal(float value, float lo, float hi)
{
    if (value < lo)
    {
        return lo;
    }
    if (value > hi)
    {
        return hi;
    }
    return value;
}

float signfLocal(float value)
{
    if (value > 0.0f)
    {
        return 1.0f;
    }
    if (value < 0.0f)
    {
        return -1.0f;
    }
    return 0.0f;
}
} // namespace

void HeadingHoldController::configure(const HeadingHoldConfig &config)
{
    config_ = config;
}

void HeadingHoldController::reset()
{
    setpoint_deg_ = 0.0f;
    rate_i_term_ = 0.0f;
    last_output_ = 0.0f;
    prev_abs_error_deg_ = 0.0f;
    error_growing_count_ = 0;
    runaway_detected_ = false;
}

void HeadingHoldController::capture(float current_yaw_deg)
{
    setpoint_deg_ = current_yaw_deg;
    rate_i_term_ = 0.0f;
    last_output_ = 0.0f;
    prev_abs_error_deg_ = 0.0f;
    error_growing_count_ = 0;
    runaway_detected_ = false;
}

float HeadingHoldController::update(float yaw_deg,
                                    float yaw_rate_dps,
                                    float wheel_rot_cps,
                                    float wheel_balance,
                                    float dt_s)
{
    if (dt_s <= 0.0f)
    {
        return 0.0f;
    }

    const float error_deg = wrapAngle(setpoint_deg_ - yaw_deg);
    if (fabsf(error_deg) <= config_.heading_deadband_deg &&
        fabsf(yaw_rate_dps) <= config_.rate_deadband_dps &&
        fabsf(wheel_rot_cps) <= config_.encoder_motion_threshold_cps)
    {
        rate_i_term_ *= 0.95f;
        last_output_ = slew(0.0f, dt_s);
        return last_output_;
    }

    rate_i_term_ += error_deg * config_.heading_ki * dt_s;
    rate_i_term_ = clampfLocal(rate_i_term_,
                               -config_.target_rate_limit_dps,
                               config_.target_rate_limit_dps);

    const float target_rate_dps = clampfLocal((config_.heading_kp * error_deg) + rate_i_term_,
                                              -config_.target_rate_limit_dps,
                                              config_.target_rate_limit_dps);
    const float rate_error_dps = target_rate_dps - yaw_rate_dps;

    float out = config_.rate_kp * rate_error_dps;
    out -= config_.wheel_balance_kp * wheel_balance * signfLocal(target_rate_dps);

    if (fabsf(error_deg) >= config_.static_error_deg &&
        fabsf(wheel_rot_cps) < config_.encoder_motion_threshold_cps &&
        fabsf(out) < config_.static_output)
    {
        out = signfLocal((target_rate_dps != 0.0f) ? target_rate_dps : error_deg) * config_.static_output;
    }

    out = clampfLocal(out, -config_.output_limit, config_.output_limit);

    const float abs_error = fabsf(error_deg);
    const bool pid_commanding = fabsf(out) >= kRunawayMinOutputMag;
    const bool error_is_growing = (abs_error > prev_abs_error_deg_ + 0.5f) &&
                                  (abs_error >= kRunawayMinErrorDeg);
    prev_abs_error_deg_ = abs_error;

    if (pid_commanding && error_is_growing)
    {
        if (error_growing_count_ < 255)
        {
            ++error_growing_count_;
        }
    }
    else
    {
        if (error_growing_count_ > 0)
        {
            --error_growing_count_;
        }
    }

    if (error_growing_count_ >= kRunawayThreshold)
    {
        runaway_detected_ = true;
    }

    if (runaway_detected_)
    {
        last_output_ = slew(0.0f, dt_s);
        return last_output_;
    }

    last_output_ = slew(out, dt_s);
    return last_output_;
}

bool HeadingHoldController::runawayDetected() const
{
    return runaway_detected_;
}

float HeadingHoldController::setpointDeg() const
{
    return setpoint_deg_;
}

float HeadingHoldController::wrapAngle(float deg) const
{
    while (deg > 180.0f)
    {
        deg -= 360.0f;
    }
    while (deg < -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

float HeadingHoldController::slew(float target, float dt_s)
{
    const float max_step = config_.slew_per_s * dt_s;
    const float delta = target - last_output_;
    if (delta > max_step)
    {
        return last_output_ + max_step;
    }
    if (delta < -max_step)
    {
        return last_output_ - max_step;
    }
    return target;
}

} // namespace drive
} // namespace robot
