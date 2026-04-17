#pragma once

#include <Arduino.h>

#include "health_report.h"
#include "imu_if.h"
#include "stepper_link_if.h"

namespace robot
{
namespace app
{
class SelfCheckRunner
{
public:
    void begin(IImuService *imu, IStepperLink *stepper, bool require_stepper);
    void update(uint32_t now_ms);

    const HealthReport &report() const;

private:
    IImuService *imu_ = nullptr;
    IStepperLink *stepper_ = nullptr;
    bool require_stepper_ = true;
    HealthReport report_{};
    uint32_t last_probe_ms_ = 0;
};

} // namespace app
} // namespace robot
