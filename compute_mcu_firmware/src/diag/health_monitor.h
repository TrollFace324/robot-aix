#pragma once

#include <Arduino.h>

#include "health_report.h"
#include "protocol_types.h"

namespace robot
{
namespace diag
{
class HealthMonitor
{
public:
    void begin();
    void onPacketRx(const protocol::Packet &packet);
    void onTimeout();
    uint8_t healthByte(const HealthReport &report) const;

private:
    uint32_t rx_count_ = 0;
    uint32_t timeout_count_ = 0;
};

} // namespace diag
} // namespace robot
