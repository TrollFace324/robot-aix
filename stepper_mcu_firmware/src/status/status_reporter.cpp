#include "status/status_reporter.h"

namespace robot
{
namespace status
{
protocol::I2cStatusFrame StatusReporter::buildFrame(const protocol::StepperStatusPayload &status, uint8_t seq) const
{
    protocol::I2cStatusFrame frame;
    frame.status = status.status;
    frame.seq = seq;
    frame.length =
        (status.cfg_reply_type == protocol::StepperConfigReplyType::None) ? 8 : 13;
    frame.payload[0] = static_cast<uint8_t>(status.current_position_mm & 0xFF);
    frame.payload[1] = static_cast<uint8_t>((status.current_position_mm >> 8) & 0xFF);
    frame.payload[2] = static_cast<uint8_t>(status.target_position_mm & 0xFF);
    frame.payload[3] = static_cast<uint8_t>((status.target_position_mm >> 8) & 0xFF);
    frame.payload[4] = static_cast<uint8_t>(status.is_homed ? 1 : 0);
    frame.payload[5] = static_cast<uint8_t>(status.driver_enabled ? 1 : 0);
    frame.payload[6] = static_cast<uint8_t>(status.home_switch_state ? 1 : 0);
    frame.payload[7] = status.fault_flags;
    if (frame.length >= 13)
    {
        frame.payload[8] = static_cast<uint8_t>(status.cfg_reply_type);
        frame.payload[9] = status.cfg_slot;
        frame.payload[10] = static_cast<uint8_t>(status.cfg_mm & 0xFF);
        frame.payload[11] = static_cast<uint8_t>((status.cfg_mm >> 8) & 0xFF);
        frame.payload[12] = static_cast<uint8_t>(status.cfg_result);
    }
    return frame;
}

} // namespace status
} // namespace robot
