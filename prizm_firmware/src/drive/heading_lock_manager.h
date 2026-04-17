#pragma once

#include <Arduino.h>

namespace robot
{
namespace drive
{
enum class HeadingLockFault : uint8_t
{
    NONE = 0,
    COMPUTE_OFFLINE = 1,
    IMU_NOT_READY = 2,
    IMU_STALE = 3,
    TILT = 4,
    STEPPER_ACTIVE = 5
};

struct HeadingLockInputs
{
    bool ch10_high = false;
    bool compute_online = false;
    bool yaw_valid = false;
    bool tilt_fault = false;
    bool stepper_active = false;
    uint8_t imu_sample_age_ms = 255;
};

class HeadingLockManager
{
public:
    void reset();
    void update(const HeadingLockInputs &inputs, uint8_t imu_fresh_age_ms);

    bool active() const;
    bool faultLatched() const;
    bool captureRequested() const;
    void clearCaptureRequest();
    HeadingLockFault fault() const;

private:
    HeadingLockFault evaluateFault(const HeadingLockInputs &inputs, uint8_t imu_fresh_age_ms) const;

    bool active_ = false;
    bool prev_ch10_high_ = false;
    bool fault_latched_ = false;
    bool capture_requested_ = false;
    HeadingLockFault fault_ = HeadingLockFault::NONE;
};

} // namespace drive
} // namespace robot
