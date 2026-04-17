#pragma once

#include <Arduino.h>

namespace robot
{
namespace protocol
{
constexpr uint8_t kFrameSize = 32;
constexpr uint8_t kPayloadMax = 20;
constexpr uint8_t kSof = 0xA5;
constexpr uint8_t kVersion = 0x01;

constexpr uint8_t kI2cFrameSize = 24;
constexpr uint8_t kI2cPayloadMax = 16;
constexpr uint8_t kI2cSof = 0x5A;

enum class NodeId : uint8_t
{
    PRIZM = 0x01,
    COMPUTE = 0x02,
    STEPPER = 0x03,
    BROADCAST = 0xFF
};

enum class MessageType : uint8_t
{
    HELLO = 0x01,
    HELLO_ACK = 0x02,
    GET_STATUS = 0x03,
    STATUS = 0x04,
    GET_CAPABILITIES = 0x05,
    CAPABILITIES = 0x06,
    HEARTBEAT = 0x07,
    HEARTBEAT_ACK = 0x08,
    FAULT_STATE = 0x09,
    ECHO_TEST = 0x0A,
    READY_TO_RUN = 0x0B,
    RUN_LOCKED = 0x0C,
    STARTUP_CHECK_RESULT = 0x0D,
    TEST_FLAG = 0x0E,

    SET_MODE = 0x20,
    MANUAL_INPUT_STATE = 0x21,
    HEADING_REFERENCE = 0x22,
    AUTONOMOUS_COMMAND = 0x23,
    DRIVE_TARGET = 0x24,
    STOP_NOW = 0x25,
    ACTUATOR_COMMAND = 0x26,
    SENSOR_STATE = 0x27,
    IMU_STATE = 0x28,

    STATION_COMMAND = 0x30,
    STATION_TELEMETRY = 0x31
};

enum PacketFlags : uint8_t
{
    FLAG_ACK = 0x01,
    FLAG_NACK = 0x02,
    FLAG_REQ_ACK = 0x04
};

constexpr uint8_t kSetModeFlagStartGateOpen = 0x01;
constexpr uint8_t kSetModeFlagCrsfFresh = 0x02;

struct SetModeSummaryPayload
{
    bool start_gate_open = false;
    bool crsf_fresh = false;
    uint16_t battery_cV = 0;
};

struct Packet
{
    uint8_t version = kVersion;
    MessageType type = MessageType::HELLO;
    uint8_t flags = 0;
    NodeId source = NodeId::PRIZM;
    NodeId target = NodeId::COMPUTE;
    uint8_t seq = 0;
    uint8_t ack_seq = 0;
    uint8_t length = 0;
    uint8_t payload[kPayloadMax]{};
};

struct LinkHealth
{
    uint32_t tx_packets = 0;
    uint32_t rx_packets = 0;
    uint32_t crc_errors = 0;
    uint32_t decode_errors = 0;
    uint32_t retries = 0;
    uint32_t timeouts = 0;
    uint32_t duplicates = 0;
    uint32_t stale_packets = 0;
};

enum class StepperCommand : uint8_t
{
    CMD_HOME = 0x01,
    CMD_GOTO_MM = 0x02,
    CMD_EXTEND_MM = 0x03,
    CMD_RETRACT_MM = 0x04,
    CMD_STOP = 0x05,
    CMD_SET_LIMITS = 0x06,
    CMD_SET_STOP_MODE = 0x07,
    CMD_REQUEST_STATUS = 0x08,
    CMD_GOTO_SLOT = 0x09,
    CMD_GET_SLOT_MM = 0x0A,
    CMD_SET_SLOT_MM = 0x0B
};

enum class StepperConfigReplyType : uint8_t
{
    None = 0,
    ReadSlot = 1,
    WriteSlot = 2
};

enum class StepperConfigResult : uint8_t
{
    Ok = 0,
    InvalidSlot = 1,
    InvalidMm = 2,
    Busy = 3,
    EepromSaveFail = 4
};

enum class StepperStatusCode : uint8_t
{
    STATUS_IDLE = 0,
    STATUS_MOVING = 1,
    STATUS_HOMING = 2,
    STATUS_STOPPED = 3,
    STATUS_ERROR = 4
};

struct StepperStatusPayload
{
    int16_t current_position_mm = 0;
    int16_t target_position_mm = 0;
    bool is_homed = false;
    bool driver_enabled = false;
    bool home_switch_state = false;
    uint8_t fault_flags = 0;
    StepperStatusCode status = StepperStatusCode::STATUS_IDLE;
    StepperConfigReplyType cfg_reply_type = StepperConfigReplyType::None;
    uint8_t cfg_slot = 0xFF;
    int16_t cfg_mm = 0;
    StepperConfigResult cfg_result = StepperConfigResult::Ok;
};

struct I2cCommandFrame
{
    uint8_t sof = kI2cSof;
    uint8_t version = kVersion;
    StepperCommand cmd = StepperCommand::CMD_REQUEST_STATUS;
    uint8_t seq = 0;
    uint8_t length = 0;
    uint8_t payload[kI2cPayloadMax]{};
};

struct I2cStatusFrame
{
    uint8_t sof = kI2cSof;
    uint8_t version = kVersion;
    StepperStatusCode status = StepperStatusCode::STATUS_IDLE;
    uint8_t seq = 0;
    uint8_t length = 0;
    uint8_t payload[kI2cPayloadMax]{};
};

uint16_t crc16Ccitt(const uint8_t *data, size_t size);
uint8_t crc8(const uint8_t *data, size_t size);
bool encodePacket(const Packet &packet, uint8_t frame[kFrameSize]);
bool decodePacket(const uint8_t frame[kFrameSize], Packet &packet, LinkHealth *health = nullptr);

bool encodeI2cCommand(const I2cCommandFrame &command, uint8_t out[kI2cFrameSize]);
bool decodeI2cCommand(const uint8_t in[kI2cFrameSize], I2cCommandFrame &command);

bool encodeI2cStatus(const I2cStatusFrame &status, uint8_t out[kI2cFrameSize]);
bool decodeI2cStatus(const uint8_t in[kI2cFrameSize], I2cStatusFrame &status);

inline uint8_t encodeSetModeSummaryPayload(const SetModeSummaryPayload &payload,
                                           uint8_t out[kPayloadMax])
{
    out[0] = 0;
    if (payload.start_gate_open)
    {
        out[0] |= kSetModeFlagStartGateOpen;
    }
    if (payload.crsf_fresh)
    {
        out[0] |= kSetModeFlagCrsfFresh;
    }
    out[1] = static_cast<uint8_t>(payload.battery_cV & 0xFF);
    out[2] = static_cast<uint8_t>((payload.battery_cV >> 8) & 0xFF);
    return 3;
}

inline SetModeSummaryPayload decodeSetModeSummaryPayload(const uint8_t *payload,
                                                         uint8_t length)
{
    SetModeSummaryPayload decoded{};
    if (payload == nullptr || length < 3)
    {
        return decoded;
    }

    decoded.start_gate_open = (payload[0] & kSetModeFlagStartGateOpen) != 0;
    decoded.crsf_fresh = (payload[0] & kSetModeFlagCrsfFresh) != 0;
    decoded.battery_cV = static_cast<uint16_t>(payload[1]) |
                         (static_cast<uint16_t>(payload[2]) << 8);
    return decoded;
}

} // namespace protocol
} // namespace robot
