#pragma once

#include <Arduino.h>

namespace robot
{
struct HealthReport
{
    bool imu_healthy = false;
    bool imu_ok = false;
    bool imu_calibrating = false;
    bool tilt_fault = false;
    bool i2c_bus_ok = false;
    bool stepper_online = false;
    bool startup_ready = false;
    uint8_t stepper_status_code = 0;
    uint8_t fault_flags = 0;
    uint32_t uptime_ms = 0;
};
} // namespace robot
