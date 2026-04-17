#include "drive/heading_lock_manager.h"

namespace robot
{
namespace drive
{
void HeadingLockManager::reset()
{
    active_ = false;
    prev_ch10_high_ = false;
    fault_latched_ = false;
    capture_requested_ = false;
    fault_ = HeadingLockFault::NONE;
}

void HeadingLockManager::update(const HeadingLockInputs &inputs, uint8_t imu_fresh_age_ms)
{
    capture_requested_ = false;

    if (!inputs.ch10_high)
    {
        active_ = false;
        prev_ch10_high_ = false;
        fault_latched_ = false;
        fault_ = HeadingLockFault::NONE;
        return;
    }

    const HeadingLockFault fault = evaluateFault(inputs, imu_fresh_age_ms);
    if (!prev_ch10_high_)
    {
        prev_ch10_high_ = true;
        if (fault == HeadingLockFault::NONE)
        {
            active_ = true;
            capture_requested_ = true;
        }
        else
        {
            active_ = false;
            fault_latched_ = true;
            fault_ = fault;
        }
        return;
    }

    prev_ch10_high_ = true;
    if (fault_latched_)
    {
        active_ = false;
        return;
    }

    if (fault != HeadingLockFault::NONE)
    {
        active_ = false;
        fault_latched_ = true;
        fault_ = fault;
        return;
    }

    active_ = true;
}

bool HeadingLockManager::active() const
{
    return active_;
}

bool HeadingLockManager::faultLatched() const
{
    return fault_latched_;
}

bool HeadingLockManager::captureRequested() const
{
    return capture_requested_;
}

void HeadingLockManager::clearCaptureRequest()
{
    capture_requested_ = false;
}

HeadingLockFault HeadingLockManager::fault() const
{
    return fault_;
}

HeadingLockFault HeadingLockManager::evaluateFault(const HeadingLockInputs &inputs, uint8_t imu_fresh_age_ms) const
{
    if (!inputs.compute_online)
    {
        return HeadingLockFault::COMPUTE_OFFLINE;
    }
    if (!inputs.yaw_valid)
    {
        return HeadingLockFault::IMU_NOT_READY;
    }
    if (inputs.tilt_fault)
    {
        return HeadingLockFault::TILT;
    }
    if (inputs.stepper_active)
    {
        return HeadingLockFault::STEPPER_ACTIVE;
    }
    if (inputs.imu_sample_age_ms > imu_fresh_age_ms)
    {
        return HeadingLockFault::IMU_STALE;
    }
    return HeadingLockFault::NONE;
}

} // namespace drive
} // namespace robot
