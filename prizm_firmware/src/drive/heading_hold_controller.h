#pragma once

#include <Arduino.h>

namespace robot
{
namespace drive
{
struct HeadingHoldConfig
{
    float heading_kp = 1.2f;              // reduced for safety, field tuning required
    float heading_ki = 0.08f;             // reduced for safety, field tuning required
    float rate_kp = 0.008f;              // reduced for safety, field tuning required
    float wheel_balance_kp = 0.10f;       // temporary default, field tuning required
    float heading_deadband_deg = 0.8f;
    float rate_deadband_dps = 2.5f;
    float target_rate_limit_dps = 100.0f;
    float output_limit = 0.50f;
    float slew_per_s = 2.5f;
    float static_output = 0.20f;
    float static_error_deg = 1.0f;
    float encoder_motion_threshold_cps = 16.0f; // temporary default, field tuning required
    int16_t wheel_speed_limit_dps = 420;        // temporary default, field tuning required
    uint8_t imu_fresh_age_ms = 60;
    uint8_t encoder_poll_interval_ms = 3;
    uint16_t encoder_ticks_per_rev = 269;       // provisional from marked 20-turn wheel tests
};

class HeadingHoldController
{
public:
    void configure(const HeadingHoldConfig &config);
    void reset();
    void capture(float current_yaw_deg);
    float update(float yaw_deg,
                 float yaw_rate_dps,
                 float wheel_rot_cps,
                 float wheel_balance,
                 float dt_s);
    float setpointDeg() const;
    bool runawayDetected() const;

private:
    float wrapAngle(float deg) const;
    float slew(float target, float dt_s);

    static constexpr uint8_t kRunawayThreshold = 20;
    static constexpr float kRunawayMinErrorDeg = 8.0f;
    static constexpr float kRunawayMinOutputMag = 0.15f;

    HeadingHoldConfig config_{};
    float setpoint_deg_ = 0.0f;
    float rate_i_term_ = 0.0f;
    float last_output_ = 0.0f;
    float prev_abs_error_deg_ = 0.0f;
    uint8_t error_growing_count_ = 0;
    bool runaway_detected_ = false;
};

} // namespace drive
} // namespace robot
