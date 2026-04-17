#include "diag/health_monitor.h"

namespace robot
{
namespace diag
{
void HealthMonitor::begin()
{
    rx_count_ = 0;
    timeout_count_ = 0;
}

void HealthMonitor::onPacketRx(const protocol::Packet &)
{
    ++rx_count_;
}

void HealthMonitor::onTimeout()
{
    ++timeout_count_;
}

uint8_t HealthMonitor::healthByte(const HealthReport &report) const
{
    uint8_t value = 0;
    if (report.imu_ok)
    {
        value |= 0x01;
    }
    if (report.stepper_online)
    {
        value |= 0x02;
    }
    if (timeout_count_ == 0)
    {
        value |= 0x04;
    }
    if (rx_count_ > 0)
    {
        value |= 0x08;
    }
    return value;
}

} // namespace diag
} // namespace robot
