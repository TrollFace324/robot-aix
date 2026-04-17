#pragma once

#include <Arduino.h>

#include "transport_if.h"

namespace robot
{
namespace transport
{
class SpiSlaveBackend : public IFrameLink
{
public:
    SpiSlaveBackend();

    void begin() override;
    void poll(uint32_t now_ms) override;
    bool sendPacket(const protocol::Packet &packet) override;
    bool receivePacket(protocol::Packet &packet) override;
    const protocol::LinkHealth &health() const override;

    static void onSpiByteIsr();

private:
    void handleSpiByte();

    static SpiSlaveBackend *instance_;

    volatile uint8_t rx_frame_[protocol::kFrameSize]{};
    volatile uint8_t tx_frame_[protocol::kFrameSize]{};
    volatile uint8_t rx_index_ = 0;
    volatile uint8_t tx_index_ = 0;
    volatile bool frame_ready_ = false;
    volatile bool transfer_active_ = false;

    protocol::LinkHealth health_{};
};

} // namespace transport
} // namespace robot
