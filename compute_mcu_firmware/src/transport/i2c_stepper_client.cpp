#include "transport/i2c_stepper_client.h"

#include <string.h>

#include "config/transport_config.h"

namespace
{
constexpr uint8_t kScanMinAddress = 0x08;
constexpr uint8_t kScanMaxAddress = 0x77;
constexpr uint32_t kScanRetryMs = 1000;
constexpr uint8_t kConfigReplyAttempts = 24;
constexpr uint8_t kConfigReplyDelayMs = 2;
}

namespace robot
{
namespace transport
{
void I2cStepperClient::begin(TwoWire &wire, uint8_t i2c_address)
{
    wire_ = &wire;
    i2c_address_ = i2c_address;
    configured_address_ = i2c_address;
    next_seq_ = 1;
    online_ = false;
    last_ok_ms_ = 0;
    last_scan_ms_ = 0;
    status_ = protocol::StepperStatusPayload{};
    last_write_status_ = 0xFF;
    last_request_count_ = 0;
    last_diag_code_ = DIAG_OK;
    last_status_seq_ = 0;

    configureWire();
}

bool I2cStepperClient::sendCommand(protocol::StepperCommand cmd, int16_t arg0, int16_t arg1)
{
    uint8_t ignored_seq = 0;
    return sendCommandWithSeq(cmd, arg0, arg1, ignored_seq);
}

bool I2cStepperClient::readSlotMm(uint32_t now_ms,
                                  uint8_t slot,
                                  int16_t &mm_out,
                                  protocol::StepperConfigResult &result_out)
{
    uint8_t seq = 0;
    if (!sendCommandWithSeq(protocol::StepperCommand::CMD_GET_SLOT_MM,
                            slot,
                            0,
                            seq))
    {
        result_out = protocol::StepperConfigResult::EepromSaveFail;
        return false;
    }

    if (!waitForConfigReply(now_ms,
                            seq,
                            protocol::StepperConfigReplyType::ReadSlot,
                            slot))
    {
        result_out = protocol::StepperConfigResult::EepromSaveFail;
        return false;
    }

    mm_out = status_.cfg_mm;
    result_out = status_.cfg_result;
    return true;
}

bool I2cStepperClient::writeSlotMm(uint32_t now_ms,
                                   uint8_t slot,
                                   int16_t mm,
                                   protocol::StepperConfigResult &result_out)
{
    uint8_t seq = 0;
    if (!sendCommandWithSeq(protocol::StepperCommand::CMD_SET_SLOT_MM,
                            slot,
                            mm,
                            seq))
    {
        result_out = protocol::StepperConfigResult::EepromSaveFail;
        return false;
    }

    if (!waitForConfigReply(now_ms,
                            seq,
                            protocol::StepperConfigReplyType::WriteSlot,
                            slot))
    {
        result_out = protocol::StepperConfigResult::EepromSaveFail;
        return false;
    }

    result_out = status_.cfg_result;
    return true;
}

bool I2cStepperClient::sendCommandWithSeq(protocol::StepperCommand cmd,
                                          int16_t arg0,
                                          int16_t arg1,
                                          uint8_t &seq_out)
{
    if (!wire_)
    {
        last_diag_code_ = DIAG_NO_WIRE;
        last_write_status_ = 0xFF;
        return false;
    }

    protocol::I2cCommandFrame frame;
    frame.cmd = cmd;
    frame.seq = next_seq_++;
    seq_out = frame.seq;
    frame.length = 4;
    frame.payload[0] = static_cast<uint8_t>(arg0 & 0xFF);
    frame.payload[1] = static_cast<uint8_t>((arg0 >> 8) & 0xFF);
    frame.payload[2] = static_cast<uint8_t>(arg1 & 0xFF);
    frame.payload[3] = static_cast<uint8_t>((arg1 >> 8) & 0xFF);

    uint8_t raw[protocol::kI2cFrameSize]{};
    if (!protocol::encodeI2cCommand(frame, raw))
    {
        last_diag_code_ = DIAG_ENCODE_FAIL;
        last_write_status_ = 0xFE;
        return false;
    }

    wire_->beginTransmission(i2c_address_);
    wire_->write(raw, protocol::kI2cFrameSize);
    last_write_status_ = wire_->endTransmission();
    if (last_write_status_ != 0)
    {
        last_diag_code_ = DIAG_END_TX_FAIL;
        return false;
    }

    return true;
}

bool I2cStepperClient::waitForConfigReply(
    uint32_t now_ms,
    uint8_t expected_seq,
    protocol::StepperConfigReplyType expected_type,
    uint8_t expected_slot)
{
    for (uint8_t attempt = 0; attempt < kConfigReplyAttempts; ++attempt)
    {
        const uint32_t poll_now_ms = now_ms + attempt;
        if (requestStatus(poll_now_ms) &&
            last_status_seq_ == expected_seq &&
            status_.cfg_reply_type == expected_type &&
            status_.cfg_slot == expected_slot)
        {
            return true;
        }

        delay(kConfigReplyDelayMs);
    }

    return false;
}

bool I2cStepperClient::requestStatus(uint32_t now_ms)
{
    if (!wire_)
    {
        online_ = false;
        return false;
    }

    // Health probing is read-only; command writes are only used for real
    // actuator actions. Clear probe-side diagnostics so each poll reports its
    // own result instead of carrying stale write failures forward.
    last_write_status_ = 0xFF;
    last_request_count_ = 0;

    if (requestStatusOnce(now_ms))
    {
        return true;
    }

    const uint8_t first_request_count = last_request_count_;
    if (first_request_count != protocol::kI2cFrameSize)
    {
        recoverBus();
        if (requestStatusOnce(now_ms))
        {
            return true;
        }
    }

    if (now_ms - last_scan_ms_ >= kScanRetryMs && discoverAddress(now_ms))
    {
        return true;
    }

    online_ = false;
    return false;
}

void I2cStepperClient::configureWire()
{
    if (!wire_)
    {
        return;
    }

    wire_->begin();
    wire_->setClock(config::kComputeI2cClockHz);
#if defined(WIRE_TIMEOUT)
    wire_->setWireTimeout(config::kComputeI2cTimeoutUs, true);
    wire_->clearWireTimeoutFlag();
#endif
}

void I2cStepperClient::recoverBus()
{
    if (!wire_)
    {
        return;
    }

    wire_->end();

    pinMode(SDA, INPUT_PULLUP);
    pinMode(SCL, INPUT_PULLUP);
    delayMicroseconds(config::kComputeI2cRecoveryPulseUs);

    for (uint8_t i = 0; i < config::kComputeI2cRecoveryClocks; ++i)
    {
        pinMode(SCL, OUTPUT);
        digitalWrite(SCL, LOW);
        delayMicroseconds(config::kComputeI2cRecoveryPulseUs);
        pinMode(SCL, INPUT_PULLUP);
        delayMicroseconds(config::kComputeI2cRecoveryPulseUs);
    }

    pinMode(SDA, OUTPUT);
    digitalWrite(SDA, LOW);
    delayMicroseconds(config::kComputeI2cRecoveryPulseUs);
    pinMode(SCL, INPUT_PULLUP);
    delayMicroseconds(config::kComputeI2cRecoveryPulseUs);
    pinMode(SDA, INPUT_PULLUP);
    delayMicroseconds(config::kComputeI2cRecoveryPulseUs);

    configureWire();
}

bool I2cStepperClient::requestStatusOnce(uint32_t now_ms)
{
    // Match the canonical Wire master-reader flow: health polling is a plain
    // fixed-length requestFrom() against the continuously published status.
    return readStatus(now_ms);
}

bool I2cStepperClient::readFrameFromAddress(uint8_t address, protocol::I2cStatusFrame &frame)
{
    const uint8_t req = wire_->requestFrom(address, static_cast<uint8_t>(protocol::kI2cFrameSize));
    last_request_count_ = req;
    if (req != protocol::kI2cFrameSize)
    {
        last_diag_code_ = DIAG_REQ_COUNT_FAIL;
        while (wire_->available() > 0)
        {
            static_cast<void>(wire_->read());
        }
        return false;
    }

    uint8_t raw[protocol::kI2cFrameSize]{};
    for (uint8_t i = 0; i < protocol::kI2cFrameSize; ++i)
    {
        if (wire_->available() <= 0)
        {
            last_diag_code_ = DIAG_READ_UNDERRUN;
            return false;
        }
        raw[i] = static_cast<uint8_t>(wire_->read());
    }

    if (!protocol::decodeI2cStatus(raw, frame))
    {
        last_diag_code_ = DIAG_DECODE_FAIL;
        return false;
    }

    return true;
}

void I2cStepperClient::applyStatusFrame(const protocol::I2cStatusFrame &frame, uint32_t now_ms)
{
    status_ = decodePayload(frame);
    online_ = true;
    last_ok_ms_ = now_ms;
    last_status_seq_ = frame.seq;
    last_diag_code_ = DIAG_OK;
    last_write_status_ = 0;
}

bool I2cStepperClient::discoverAddress(uint32_t now_ms)
{
    if (!wire_)
    {
        return false;
    }

    last_scan_ms_ = now_ms;

    protocol::I2cStatusFrame frame;
    if (configured_address_ != i2c_address_ && readFrameFromAddress(configured_address_, frame))
    {
        i2c_address_ = configured_address_;
        applyStatusFrame(frame, now_ms);
        return true;
    }

    for (uint8_t address = kScanMinAddress; address <= kScanMaxAddress; ++address)
    {
        if (address == i2c_address_)
        {
            continue;
        }

        if (!readFrameFromAddress(address, frame))
        {
            continue;
        }

        i2c_address_ = address;
        applyStatusFrame(frame, now_ms);
        return true;
    }

    online_ = false;
    return false;
}

bool I2cStepperClient::readStatus(uint32_t now_ms)
{
    if (!wire_)
    {
        online_ = false;
        last_diag_code_ = DIAG_NO_WIRE;
        return false;
    }

    protocol::I2cStatusFrame frame;
    if (!readFrameFromAddress(i2c_address_, frame))
    {
        online_ = false;
        return false;
    }

    applyStatusFrame(frame, now_ms);
    return true;
}

bool I2cStepperClient::online() const
{
    return online_;
}

protocol::StepperStatusPayload I2cStepperClient::status() const
{
    return status_;
}

uint8_t I2cStepperClient::lastWriteStatus() const
{
    return last_write_status_;
}

uint8_t I2cStepperClient::lastRequestCount() const
{
    return last_request_count_;
}

uint8_t I2cStepperClient::lastDiagCode() const
{
    return last_diag_code_;
}

uint8_t I2cStepperClient::lastStatusSeq() const
{
    return last_status_seq_;
}

protocol::StepperStatusPayload I2cStepperClient::decodePayload(const protocol::I2cStatusFrame &frame) const
{
    protocol::StepperStatusPayload out;
    out.status = frame.status;

    if (frame.length >= 8)
    {
        out.current_position_mm = static_cast<int16_t>(frame.payload[0] | (frame.payload[1] << 8));
        out.target_position_mm = static_cast<int16_t>(frame.payload[2] | (frame.payload[3] << 8));
        out.is_homed = frame.payload[4] != 0;
        out.driver_enabled = frame.payload[5] != 0;
        out.home_switch_state = frame.payload[6] != 0;
        out.fault_flags = frame.payload[7];
    }

    if (frame.length >= 13)
    {
        out.cfg_reply_type =
            static_cast<protocol::StepperConfigReplyType>(frame.payload[8]);
        out.cfg_slot = frame.payload[9];
        out.cfg_mm = static_cast<int16_t>(frame.payload[10] |
                                          (frame.payload[11] << 8));
        out.cfg_result =
            static_cast<protocol::StepperConfigResult>(frame.payload[12]);
    }

    return out;
}

} // namespace transport
} // namespace robot
