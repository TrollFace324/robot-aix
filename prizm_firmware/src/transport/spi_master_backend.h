#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "transport_if.h"

namespace robot
{
namespace transport
{
class SpiMasterBackend : public IFrameLink
{
public:
    explicit SpiMasterBackend(uint8_t cs_pin);

    void begin() override;
    void poll(uint32_t now_ms) override;
    bool sendPacket(const protocol::Packet &packet) override;
    bool receivePacket(protocol::Packet &packet) override;
    const protocol::LinkHealth &health() const override;

private:
    uint8_t cs_pin_;
    uint8_t tx_frame_[protocol::kFrameSize]{};
    uint8_t rx_frame_[protocol::kFrameSize]{};
    bool tx_pending_ = false;
    bool rx_pending_ = false;
    protocol::LinkHealth health_{};
};

} // namespace transport
} // namespace robot
