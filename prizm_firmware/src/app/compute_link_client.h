#pragma once

#include <Arduino.h>

#include "protocol_types.h"
#include "transport_if.h"

namespace robot
{
namespace app
{
struct ComputeStatus
{
    bool online = false;
    bool hello_ok = false;
    bool capabilities_ok = false;
    bool self_check_ok = false;
    bool ready_to_run = false;
    bool imu_healthy = false;
    bool yaw_valid = false;
    bool imu_calibrating = false;
    bool tilt_fault = false;
    bool stepper_active = false;
    uint32_t capabilities = 0;
    uint8_t health = 0;
    uint8_t imu_sample_age_ms = 255;
    uint8_t stepper_status_code = 0;
    float yaw_deg = 0.0f;
    float yaw_rate_dps = 0.0f;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    uint32_t last_rx_ms = 0;
    uint32_t last_imu_update_ms = 0;
};

class ComputeLinkClient
{
public:
    explicit ComputeLinkClient(transport::IFrameLink &link);

    void begin(uint32_t now_ms);
    void poll(uint32_t now_ms);

    const ComputeStatus &status() const;
    bool sendSetModeSummary(bool start_gate_open, bool crsf_fresh, uint16_t battery_cV);
    bool sendEchoTest(uint8_t value);
    bool sendTestFlag(uint8_t value);

private:
    bool sendSimple(protocol::MessageType type, const uint8_t *payload = nullptr, uint8_t length = 0, bool req_ack = false);
    void handlePacket(const protocol::Packet &packet, uint32_t now_ms);

    transport::IFrameLink &link_;
    ComputeStatus status_{};
    uint8_t next_seq_ = 1;
    uint8_t last_rx_seq_ = 0;
    uint32_t last_hello_ms_ = 0;
    uint32_t last_heartbeat_ms_ = 0;
    uint32_t last_status_ms_ = 0;
    uint32_t last_diag_ms_ = 0;
    uint32_t last_drop_trace_ms_ = 0;
    uint32_t loopback_suspect_count_ = 0;
};

} // namespace app
} // namespace robot
