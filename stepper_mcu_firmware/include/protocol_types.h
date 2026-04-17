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
bool encodeI2cCommand(const I2cCommandFrame &command, uint8_t out[kI2cFrameSize]);
bool decodeI2cCommand(const uint8_t in[kI2cFrameSize], I2cCommandFrame &command);

bool encodeI2cStatus(const I2cStatusFrame &status, uint8_t out[kI2cFrameSize]);
bool decodeI2cStatus(const uint8_t in[kI2cFrameSize], I2cStatusFrame &status);

} // namespace protocol
} // namespace robot
