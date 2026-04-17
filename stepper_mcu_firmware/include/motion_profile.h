#pragma once

namespace robot
{
struct MotionProfile
{
    float max_speed_steps = 2200.0f;
    float accel_steps = 1600.0f;
    float steps_per_mm = 40.0f;
};
} // namespace robot
