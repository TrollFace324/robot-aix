#pragma once

#include <Arduino.h>

struct Pose2D
{
    float x_m;
    float y_m;
    float theta_rad;
};

class Odometry
{
public:
    void configure(float wheel_diam_m, float track_m, float wheelbase_m, float ticks_per_rev);
    void reset();
    void updateFromTicks(int32_t fl, int32_t fr, int32_t rl, int32_t rr);
    Pose2D getPose() const;

private:
    Pose2D pose_{0.0f, 0.0f, 0.0f};
    float meters_per_tick_ = 0.0f;
    float rotation_radius_ = 0.0f;
    bool has_prev_ = false;
    int32_t prev_fl_ = 0;
    int32_t prev_fr_ = 0;
    int32_t prev_rl_ = 0;
    int32_t prev_rr_ = 0;
};

