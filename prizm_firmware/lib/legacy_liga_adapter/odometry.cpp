#include "odometry.h"

void Odometry::configure(float wheel_diam_m, float track_m, float wheelbase_m, float ticks_per_rev)
{
    if (ticks_per_rev <= 0.0f)
    {
        ticks_per_rev = 1.0f;
    }

    meters_per_tick_ = (PI * wheel_diam_m) / ticks_per_rev;
    rotation_radius_ = (track_m + wheelbase_m) * 0.5f;
    if (rotation_radius_ <= 0.0f)
    {
        rotation_radius_ = 1.0f;
    }

    reset();
}

void Odometry::reset()
{
    pose_ = {0.0f, 0.0f, 0.0f};
    has_prev_ = false;
    prev_fl_ = 0;
    prev_fr_ = 0;
    prev_rl_ = 0;
    prev_rr_ = 0;
}

void Odometry::updateFromTicks(int32_t fl, int32_t fr, int32_t rl, int32_t rr)
{
    if (!has_prev_)
    {
        prev_fl_ = fl;
        prev_fr_ = fr;
        prev_rl_ = rl;
        prev_rr_ = rr;
        has_prev_ = true;
        return;
    }

    const int32_t dfl_ticks = fl - prev_fl_;
    const int32_t dfr_ticks = fr - prev_fr_;
    const int32_t drl_ticks = rl - prev_rl_;
    const int32_t drr_ticks = rr - prev_rr_;

    prev_fl_ = fl;
    prev_fr_ = fr;
    prev_rl_ = rl;
    prev_rr_ = rr;

    const float dfl = static_cast<float>(dfl_ticks) * meters_per_tick_;
    const float dfr = static_cast<float>(dfr_ticks) * meters_per_tick_;
    const float drl = static_cast<float>(drl_ticks) * meters_per_tick_;
    const float drr = static_cast<float>(drr_ticks) * meters_per_tick_;

    const float d_forward = (dfl + dfr + drl + drr) * 0.25f;
    const float d_right = (dfl - dfr - drl + drr) * 0.25f;
    const float d_theta = (dfl - dfr + drl - drr) / (4.0f * rotation_radius_);

    const float theta_mid = pose_.theta_rad + (d_theta * 0.5f);
    const float cos_t = cos(theta_mid);
    const float sin_t = sin(theta_mid);

    pose_.x_m += d_forward * cos_t - d_right * sin_t;
    pose_.y_m += d_forward * sin_t + d_right * cos_t;
    pose_.theta_rad += d_theta;

    if (pose_.theta_rad > PI)
    {
        pose_.theta_rad -= 2.0f * PI;
    }
    else if (pose_.theta_rad < -PI)
    {
        pose_.theta_rad += 2.0f * PI;
    }
}

Pose2D Odometry::getPose() const
{
    return pose_;
}

