#include "app/compute_link_client.h"

#include <string.h>

#include "config/debug_config.h"
#include "config/timing_config.h"

namespace robot
{
namespace app
{
namespace
{
const __FlashStringHelper *messageTypeName(protocol::MessageType type)
{
    switch (type)
    {
    case protocol::MessageType::HELLO:
        return F("HELLO");
    case protocol::MessageType::HELLO_ACK:
        return F("HELLO_ACK");
    case protocol::MessageType::GET_STATUS:
        return F("GET_STATUS");
    case protocol::MessageType::STATUS:
        return F("STATUS");
    case protocol::MessageType::GET_CAPABILITIES:
        return F("GET_CAPABILITIES");
    case protocol::MessageType::CAPABILITIES:
        return F("CAPABILITIES");
    case protocol::MessageType::HEARTBEAT:
        return F("HEARTBEAT");
    case protocol::MessageType::HEARTBEAT_ACK:
        return F("HEARTBEAT_ACK");
    case protocol::MessageType::FAULT_STATE:
        return F("FAULT_STATE");
    case protocol::MessageType::ECHO_TEST:
        return F("ECHO_TEST");
    case protocol::MessageType::READY_TO_RUN:
        return F("READY_TO_RUN");
    case protocol::MessageType::RUN_LOCKED:
        return F("RUN_LOCKED");
    case protocol::MessageType::STARTUP_CHECK_RESULT:
        return F("STARTUP_CHECK_RESULT");
    case protocol::MessageType::TEST_FLAG:
        return F("TEST_FLAG");
    case protocol::MessageType::SET_MODE:
        return F("SET_MODE");
    case protocol::MessageType::MANUAL_INPUT_STATE:
        return F("MANUAL_INPUT_STATE");
    case protocol::MessageType::HEADING_REFERENCE:
        return F("HEADING_REFERENCE");
    case protocol::MessageType::AUTONOMOUS_COMMAND:
        return F("AUTONOMOUS_COMMAND");
    case protocol::MessageType::DRIVE_TARGET:
        return F("DRIVE_TARGET");
    case protocol::MessageType::STOP_NOW:
        return F("STOP_NOW");
    case protocol::MessageType::ACTUATOR_COMMAND:
        return F("ACTUATOR_COMMAND");
    case protocol::MessageType::SENSOR_STATE:
        return F("SENSOR_STATE");
    case protocol::MessageType::IMU_STATE:
        return F("IMU_STATE");
    case protocol::MessageType::STATION_COMMAND:
        return F("STATION_COMMAND");
    case protocol::MessageType::STATION_TELEMETRY:
        return F("STATION_TELEMETRY");
    default:
        return F("UNKNOWN");
    }
}

void printHexByte(uint8_t value)
{
    if (value < 0x10)
    {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

int16_t decodeInt16Le(const uint8_t *data)
{
    return static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                                (static_cast<uint16_t>(data[1]) << 8));
}

void traceRxPacket(const protocol::Packet &packet, uint32_t now_ms)
{
    if (!config::kEnableSpiUartTrace)
    {
        return;
    }

    Serial.print(F("SPI RX ms="));
    Serial.print(now_ms);
    Serial.print(F(" type="));
    Serial.print(messageTypeName(packet.type));
    Serial.print(F(" seq="));
    Serial.print(packet.seq);
    Serial.print(F(" ack="));
    Serial.print(packet.ack_seq);
    Serial.print(F(" fl=0x"));
    printHexByte(packet.flags);
    Serial.print(F(" len="));
    Serial.print(packet.length);
    Serial.print(F(" p="));
    for (uint8_t i = 0; i < packet.length && i < protocol::kPayloadMax; ++i)
    {
        printHexByte(packet.payload[i]);
        if (i + 1 < packet.length)
        {
            Serial.print(' ');
        }
    }
    Serial.println();

    if (packet.type == protocol::MessageType::STATUS && packet.length >= 2)
    {
        Serial.print(F("SPI STATUS health=0x"));
        printHexByte(packet.payload[0]);
        Serial.print(F(" ready="));
        Serial.println(packet.payload[1] ? 1 : 0);
    }

    if (packet.type == protocol::MessageType::STARTUP_CHECK_RESULT)
    {
        Serial.print(F("SPI CHECK ready="));
        Serial.print((packet.length >= 1) ? (packet.payload[0] ? 1 : 0) : 0);
        Serial.print(F(" imu="));
        Serial.print((packet.length >= 2) ? (packet.payload[1] ? 1 : 0) : 0);
        Serial.print(F(" i2c="));
        Serial.print((packet.length >= 3) ? (packet.payload[2] ? 1 : 0) : 0);
        Serial.print(F(" stepper="));
        Serial.println((packet.length >= 4) ? (packet.payload[3] ? 1 : 0) : 0);

        if (packet.length >= 8)
        {
            Serial.print(F("SPI CHECK I2C tx=0x"));
            printHexByte(packet.payload[4]);
            Serial.print(F(" req="));
            Serial.print(packet.payload[5]);
            Serial.print(F(" err="));
            Serial.print(packet.payload[6]);
            Serial.print(F(" sseq="));
            Serial.println(packet.payload[7]);
        }
    }

    if (packet.type == protocol::MessageType::RUN_LOCKED && packet.length >= 1)
    {
        Serial.print(F("SPI RUN_LOCKED fault=0x"));
        printHexByte(packet.payload[0]);
        Serial.println();
    }

    if (packet.type == protocol::MessageType::IMU_STATE && packet.length >= 10)
    {
        const uint8_t flags = packet.payload[0];
        const float yaw_deg = static_cast<float>(decodeInt16Le(&packet.payload[1])) * 0.01f;
        const float yaw_rate_dps = static_cast<float>(decodeInt16Le(&packet.payload[3])) * 0.1f;
        const float roll_deg = static_cast<float>(decodeInt16Le(&packet.payload[5])) * 0.01f;
        const float pitch_deg = static_cast<float>(decodeInt16Le(&packet.payload[7])) * 0.01f;

        Serial.print(F("SPI IMU ok="));
        Serial.print((flags & 0x01) ? 1 : 0);
        Serial.print(F(" valid="));
        Serial.print((flags & 0x02) ? 1 : 0);
        Serial.print(F(" cal="));
        Serial.print((flags & 0x04) ? 1 : 0);
        Serial.print(F(" tilt="));
        Serial.print((flags & 0x08) ? 1 : 0);
        Serial.print(F(" yaw="));
        Serial.print(yaw_deg, 2);
        Serial.print(F(" rate="));
        Serial.print(yaw_rate_dps, 1);
        Serial.print(F(" roll="));
        Serial.print(roll_deg, 1);
        Serial.print(F(" pitch="));
        Serial.print(pitch_deg, 1);
        Serial.print(F(" age="));
        Serial.println(packet.payload[9]);
    }
}
} // namespace

ComputeLinkClient::ComputeLinkClient(transport::IFrameLink &link)
    : link_(link)
{
}

void ComputeLinkClient::begin(uint32_t now_ms)
{
    status_ = ComputeStatus{};
    last_hello_ms_ = now_ms;
    last_heartbeat_ms_ = now_ms;
    last_status_ms_ = now_ms;
    last_diag_ms_ = now_ms;
    last_drop_trace_ms_ = now_ms;
    loopback_suspect_count_ = 0;
    next_seq_ = 1;
    last_rx_seq_ = 0;
}

void ComputeLinkClient::poll(uint32_t now_ms)
{
    link_.poll(now_ms);

    protocol::Packet incoming;
    while (link_.receivePacket(incoming))
    {
        if (incoming.source != protocol::NodeId::COMPUTE)
        {
            if ((incoming.source == protocol::NodeId::PRIZM) &&
                (incoming.target == protocol::NodeId::COMPUTE))
            {
                ++loopback_suspect_count_;
            }

            if (config::kEnableSpiUartTrace && (now_ms - last_drop_trace_ms_ >= 300))
            {
                Serial.print(F("SPI DROP src=0x"));
                printHexByte(static_cast<uint8_t>(incoming.source));
                Serial.print(F(" dst=0x"));
                printHexByte(static_cast<uint8_t>(incoming.target));
                Serial.print(F(" type=0x"));
                printHexByte(static_cast<uint8_t>(incoming.type));
                Serial.print(F(" seq="));
                Serial.println(incoming.seq);
                last_drop_trace_ms_ = now_ms;
            }
            continue;
        }

        if (incoming.seq == last_rx_seq_)
        {
            continue;
        }
        last_rx_seq_ = incoming.seq;
        handlePacket(incoming, now_ms);
    }

    status_.online = (status_.last_rx_ms != 0) && (now_ms - status_.last_rx_ms <= config::kLinkDeadMs);

    if (!status_.hello_ok && now_ms - last_hello_ms_ >= config::kHelloRetryMs)
    {
        sendSimple(protocol::MessageType::HELLO);
        last_hello_ms_ = now_ms;
    }

    if (status_.hello_ok && !status_.capabilities_ok && now_ms - last_status_ms_ >= 120)
    {
        sendSimple(protocol::MessageType::GET_CAPABILITIES);
        last_status_ms_ = now_ms;
    }

    if (status_.hello_ok && now_ms - last_status_ms_ >= 200)
    {
        sendSimple(protocol::MessageType::GET_STATUS);
        last_status_ms_ = now_ms;
    }

    if (status_.hello_ok && now_ms - last_heartbeat_ms_ >= config::kHeartbeatPeriodMs)
    {
        sendSimple(protocol::MessageType::HEARTBEAT);
        last_heartbeat_ms_ = now_ms;
    }

    if (config::kEnableSpiUartTrace && (now_ms - last_diag_ms_ >= 1000))
    {
        const protocol::LinkHealth &h = link_.health();
        Serial.print(F("SPI LINK tx="));
        Serial.print(h.tx_packets);
        Serial.print(F(" rx="));
        Serial.print(h.rx_packets);
        Serial.print(F(" crc="));
        Serial.print(h.crc_errors);
        Serial.print(F(" dec="));
        Serial.print(h.decode_errors);
        Serial.print(F(" stale="));
        Serial.print(h.stale_packets);
        Serial.print(F(" dup="));
        Serial.print(h.duplicates);
        Serial.print(F(" loop="));
        Serial.print(loopback_suspect_count_);
        Serial.print(F(" hello="));
        Serial.print(status_.hello_ok ? 1 : 0);
        Serial.print(F(" caps="));
        Serial.print(status_.capabilities_ok ? 1 : 0);
        Serial.print(F(" self="));
        Serial.print(status_.self_check_ok ? 1 : 0);
        Serial.print(F(" ready="));
        Serial.println(status_.ready_to_run ? 1 : 0);
        if (!status_.hello_ok && (loopback_suspect_count_ >= 4))
        {
            Serial.println(F("SPI WARN: loopback suspected (PRIZM sees its own frames on MISO). Check MOSI/MISO wiring."));
            Serial.println(F("SPI WARN: verify select wiring: PRIZM D2(CS) -> compute D10(SS), and shared GND."));
        }
        last_diag_ms_ = now_ms;
    }
}

const ComputeStatus &ComputeLinkClient::status() const
{
    return status_;
}

bool ComputeLinkClient::sendSetModeSummary(bool start_gate_open,
                                           bool crsf_fresh,
                                           uint16_t battery_cV)
{
    protocol::SetModeSummaryPayload payload;
    payload.start_gate_open = start_gate_open;
    payload.crsf_fresh = crsf_fresh;
    payload.battery_cV = battery_cV;

    uint8_t raw[protocol::kPayloadMax]{};
    const uint8_t length = protocol::encodeSetModeSummaryPayload(payload, raw);
    return sendSimple(protocol::MessageType::SET_MODE, raw, length, false);
}

bool ComputeLinkClient::sendEchoTest(uint8_t value)
{
    return sendSimple(protocol::MessageType::ECHO_TEST, &value, 1, true);
}

bool ComputeLinkClient::sendTestFlag(uint8_t value)
{
    return sendSimple(protocol::MessageType::TEST_FLAG, &value, 1, true);
}

bool ComputeLinkClient::sendSimple(protocol::MessageType type, const uint8_t *payload, uint8_t length, bool req_ack)
{
    protocol::Packet packet;
    packet.type = type;
    packet.source = protocol::NodeId::PRIZM;
    packet.target = protocol::NodeId::COMPUTE;
    packet.seq = next_seq_++;
    packet.flags = req_ack ? protocol::FLAG_REQ_ACK : 0;
    packet.length = length;
    if (payload && length > 0)
    {
        memcpy(packet.payload, payload, length);
    }

    return link_.sendPacket(packet);
}

void ComputeLinkClient::handlePacket(const protocol::Packet &packet, uint32_t now_ms)
{
    status_.last_rx_ms = now_ms;
    traceRxPacket(packet, now_ms);

    switch (packet.type)
    {
    case protocol::MessageType::HELLO_ACK:
        status_.hello_ok = true;
        break;

    case protocol::MessageType::CAPABILITIES:
        if (packet.length >= 4)
        {
            status_.capabilities = static_cast<uint32_t>(packet.payload[0]) |
                                   (static_cast<uint32_t>(packet.payload[1]) << 8) |
                                   (static_cast<uint32_t>(packet.payload[2]) << 16) |
                                   (static_cast<uint32_t>(packet.payload[3]) << 24);
            status_.capabilities_ok = true;
        }
        break;

    case protocol::MessageType::STATUS:
        if (packet.length >= 2)
        {
            status_.health = packet.payload[0];
            status_.ready_to_run = (packet.payload[1] != 0);
        }
        break;

    case protocol::MessageType::STARTUP_CHECK_RESULT:
        if (packet.length >= 1)
        {
            status_.self_check_ok = (packet.payload[0] != 0);
        }
        break;

    case protocol::MessageType::SENSOR_STATE:
        if (packet.length >= 2)
        {
            status_.stepper_status_code = packet.payload[0];
            status_.stepper_active =
                (status_.stepper_status_code == static_cast<uint8_t>(0x01)) ||
                (status_.stepper_status_code == static_cast<uint8_t>(0x02));
            status_.tilt_fault = (packet.payload[1] != 0);
        }
        break;

    case protocol::MessageType::IMU_STATE:
        if (packet.length >= 10)
        {
            const uint8_t flags = packet.payload[0];
            status_.imu_healthy = (flags & 0x01) != 0;
            status_.yaw_valid = (flags & 0x02) != 0;
            status_.imu_calibrating = (flags & 0x04) != 0;
            status_.tilt_fault = (flags & 0x08) != 0;
            status_.yaw_deg = static_cast<float>(decodeInt16Le(&packet.payload[1])) * 0.01f;
            status_.yaw_rate_dps = static_cast<float>(decodeInt16Le(&packet.payload[3])) * 0.1f;
            status_.roll_deg = static_cast<float>(decodeInt16Le(&packet.payload[5])) * 0.01f;
            status_.pitch_deg = static_cast<float>(decodeInt16Le(&packet.payload[7])) * 0.01f;
            status_.imu_sample_age_ms = packet.payload[9];
            status_.last_imu_update_ms = now_ms;
        }
        break;

    case protocol::MessageType::HEARTBEAT_ACK:
    case protocol::MessageType::ECHO_TEST:
    case protocol::MessageType::FAULT_STATE:
    case protocol::MessageType::READY_TO_RUN:
    case protocol::MessageType::RUN_LOCKED:
    case protocol::MessageType::TEST_FLAG:
    default:
        break;
    }
}

} // namespace app
} // namespace robot
