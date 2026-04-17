#include "app/self_check_runner.h"

namespace robot
{
namespace app
{
void SelfCheckRunner::begin(IImuService *imu, IStepperLink *stepper, bool require_stepper)
{
    imu_ = imu;
    stepper_ = stepper;
    require_stepper_ = require_stepper;
    report_ = HealthReport{};
    last_probe_ms_ = 0;
}

void SelfCheckRunner::update(uint32_t now_ms)
{
    report_.uptime_ms = now_ms;

    const bool imu_healthy = (imu_ != nullptr) && imu_->healthy();
    const bool imu_calibrating = (imu_ != nullptr) && imu_->calibrating();
    const bool imu_data_valid = (imu_ != nullptr) && imu_->dataValid();
    const bool imu_tilt_fault = (imu_ != nullptr) && imu_->tiltFault();

    report_.imu_healthy = imu_healthy;
    report_.imu_calibrating = imu_calibrating;
    // Do not promote tilt into a hard startup fault until IMU data is valid.
    report_.tilt_fault = imu_data_valid && imu_tilt_fault;
    report_.imu_ok = imu_data_valid && !report_.tilt_fault;
    report_.i2c_bus_ok = false;

    bool stepper_reply_ok = false;
    if (stepper_ && (now_ms - last_probe_ms_ >= 250))
    {
        stepper_reply_ok = stepper_->requestStatus(now_ms);
        last_probe_ms_ = now_ms;
    }

    // Report actual I2C progress, not merely that the client object exists.
    if (stepper_)
    {
        report_.i2c_bus_ok = stepper_reply_ok || stepper_->online();
    }

    report_.stepper_online = stepper_ && stepper_->online();
    report_.stepper_status_code =
        report_.stepper_online ? static_cast<uint8_t>(stepper_->status().status) : 0;

    if (report_.imu_ok)
    {
        report_.fault_flags &= static_cast<uint8_t>(~0x01);
    }
    else
    {
        report_.fault_flags |= 0x01;
    }

    if (report_.stepper_online || !require_stepper_)
    {
        report_.fault_flags &= static_cast<uint8_t>(~0x02);
    }
    else
    {
        report_.fault_flags |= 0x02;
    }

    if (report_.tilt_fault)
    {
        report_.fault_flags |= 0x04;
    }
    else
    {
        report_.fault_flags &= static_cast<uint8_t>(~0x04);
    }

    // Startup readiness is a once-booted latch: IMU data valid and stepper (if
    // required) online. Tilt is a dynamic runtime condition and is reported
    // separately via tilt_fault — it must NOT drop startup_ready, otherwise
    // PRIZM regresses READY→SENSOR_CHECK and stops manual-mode driving.
    report_.startup_ready = imu_data_valid && (!require_stepper_ || report_.stepper_online);
}

const HealthReport &SelfCheckRunner::report() const
{
    return report_;
}

} // namespace app
} // namespace robot
