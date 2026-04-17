#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "stepper_link_if.h"

namespace robot
{
namespace transport
{
class I2cStepperClient : public IStepperLink
{
public:
    enum DiagCode : uint8_t
    {
        DIAG_OK = 0,
        DIAG_NO_WIRE = 1,
        DIAG_ENCODE_FAIL = 2,
        DIAG_END_TX_FAIL = 3,
        DIAG_REQ_COUNT_FAIL = 4,
        DIAG_READ_UNDERRUN = 5,
        DIAG_DECODE_FAIL = 6
    };

    void begin(TwoWire &wire, uint8_t i2c_address) override;
    bool requestStatus(uint32_t now_ms) override;
    bool sendCommand(protocol::StepperCommand cmd, int16_t arg0, int16_t arg1) override;
    bool readSlotMm(uint32_t now_ms,
                    uint8_t slot,
                    int16_t &mm_out,
                    protocol::StepperConfigResult &result_out);
    bool writeSlotMm(uint32_t now_ms,
                     uint8_t slot,
                     int16_t mm,
                     protocol::StepperConfigResult &result_out);
    bool online() const override;
    protocol::StepperStatusPayload status() const override;
    uint8_t lastWriteStatus() const;
    uint8_t lastRequestCount() const;
    uint8_t lastDiagCode() const;
    uint8_t lastStatusSeq() const;

private:
    bool sendCommandWithSeq(protocol::StepperCommand cmd,
                            int16_t arg0,
                            int16_t arg1,
                            uint8_t &seq_out);
    bool waitForConfigReply(uint32_t now_ms,
                            uint8_t expected_seq,
                            protocol::StepperConfigReplyType expected_type,
                            uint8_t expected_slot);
    void applyStatusFrame(const protocol::I2cStatusFrame &frame, uint32_t now_ms);
    void configureWire();
    void recoverBus();
    bool discoverAddress(uint32_t now_ms);
    bool readFrameFromAddress(uint8_t address, protocol::I2cStatusFrame &frame);
    bool requestStatusOnce(uint32_t now_ms);
    bool readStatus(uint32_t now_ms);
    protocol::StepperStatusPayload decodePayload(const protocol::I2cStatusFrame &frame) const;

    TwoWire *wire_ = nullptr;
    uint8_t i2c_address_ = 0x13;
    uint8_t configured_address_ = 0x13;
    uint8_t next_seq_ = 1;
    bool online_ = false;
    uint32_t last_ok_ms_ = 0;
    uint32_t last_scan_ms_ = 0;
    protocol::StepperStatusPayload status_{};
    uint8_t last_write_status_ = 0xFF;
    uint8_t last_request_count_ = 0;
    uint8_t last_diag_code_ = DIAG_NO_WIRE;
    uint8_t last_status_seq_ = 0;
};

} // namespace transport
} // namespace robot
