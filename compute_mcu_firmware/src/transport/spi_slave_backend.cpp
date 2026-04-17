#include "transport/spi_slave_backend.h"

#include <avr/interrupt.h>
#include <string.h>
#include <SPI.h>

#include "config/transport_config.h"

namespace robot
{
namespace transport
{
SpiSlaveBackend *SpiSlaveBackend::instance_ = nullptr;

ISR(SPI_STC_vect)
{
    SpiSlaveBackend::onSpiByteIsr();
}

SpiSlaveBackend::SpiSlaveBackend()
{
}

void SpiSlaveBackend::begin()
{
    instance_ = this;

    pinMode(MISO, OUTPUT);
    if (config::kSpiUseExternalSs)
    {
        // AVR SPI slave samples the dedicated SS pin as a hardware select input.
        // Keep it as a plain input and let the master drive the level.
        pinMode(SS, INPUT);
        digitalWrite(SS, LOW); // ensure pull-up is disabled
    }
    else
    {
        if (config::kForceSsOutputLowInNoSsMode)
        {
            // Fallback for no-SS bench: hold SS low locally.
            pinMode(SS, OUTPUT);
            digitalWrite(SS, LOW);
        }
        else
        {
            // AVR SPI slave requires SS pin as input held LOW.
            // In no-external-SS mode, this requires local hardware strap D10->GND.
            pinMode(SS, INPUT);
            digitalWrite(SS, LOW); // ensure pull-up is disabled
        }
    }

    memset((void *)rx_frame_, 0, sizeof(rx_frame_));
    memset((void *)tx_frame_, 0, sizeof(tx_frame_));
    rx_index_ = 0;
    tx_index_ = 0;
    frame_ready_ = false;
    transfer_active_ = false;

    if (config::kSpiUseFrameSync)
    {
        pinMode(config::kSpiFrameSyncPin, INPUT_PULLUP);
    }

    // AVR151 / datasheet style slave init:
    // MISO as output, hardware SS as input, then enable SPI + SPI interrupt.
    SPCR = _BV(SPE) | _BV(SPIE);
    SPSR = 0;
    (void)SPSR;
    (void)SPDR;
    SPDR = tx_frame_[0];
}

void SpiSlaveBackend::poll(uint32_t)
{
    const bool transfer_idle =
        (config::kSpiUseFrameSync && digitalRead(config::kSpiFrameSyncPin) == HIGH) ||
        (config::kSpiUseExternalSs && digitalRead(SS) == HIGH);

    if (transfer_idle)
    {
        noInterrupts();
        transfer_active_ = false;
        rx_index_ = 0;
        tx_index_ = 0;
        SPDR = tx_frame_[0];
        interrupts();
    }
}

bool SpiSlaveBackend::sendPacket(const protocol::Packet &packet)
{
    uint8_t encoded[protocol::kFrameSize]{};
    if (!protocol::encodePacket(packet, encoded))
    {
        ++health_.decode_errors;
        return false;
    }

    noInterrupts();
    memcpy((void *)tx_frame_, encoded, sizeof(encoded));
    tx_index_ = 0;
    SPDR = tx_frame_[0];
    interrupts();

    ++health_.tx_packets;
    return true;
}

bool SpiSlaveBackend::receivePacket(protocol::Packet &packet)
{
    if (!frame_ready_)
    {
        return false;
    }

    uint8_t local[protocol::kFrameSize]{};
    noInterrupts();
    memcpy(local, (const void *)rx_frame_, sizeof(local));
    frame_ready_ = false;
    interrupts();

    return protocol::decodePacket(local, packet, &health_);
}

const protocol::LinkHealth &SpiSlaveBackend::health() const
{
    return health_;
}

void SpiSlaveBackend::onSpiByteIsr()
{
    if (instance_)
    {
        instance_->handleSpiByte();
    }
}

void SpiSlaveBackend::handleSpiByte()
{
    if (config::kSpiUseExternalSs && !config::kSpiUseFrameSync)
    {
        // Tight external-SS hot path for 32-byte fixed frames.
        // On AVR slave SPI the next transmit byte must be loaded into SPDR before
        // the following byte starts shifting. Keep the ISR minimal here.
        const uint8_t incoming = SPDR;
        const uint8_t rx_index = rx_index_;
        rx_frame_[rx_index] = incoming;

        uint8_t next_tx_index = static_cast<uint8_t>(tx_index_ + 1);
        if (next_tx_index >= protocol::kFrameSize)
        {
            next_tx_index = 0;
        }
        tx_index_ = next_tx_index;
        SPDR = tx_frame_[next_tx_index];

        const uint8_t next_rx_index = static_cast<uint8_t>(rx_index + 1);
        if (next_rx_index >= protocol::kFrameSize)
        {
            rx_index_ = 0;
            frame_ready_ = true;
        }
        else
        {
            rx_index_ = next_rx_index;
        }
        return;
    }

    if (config::kSpiUseFrameSync)
    {
        const bool sync_active = (digitalRead(config::kSpiFrameSyncPin) == LOW);
        if (!sync_active)
        {
            transfer_active_ = false;
            rx_index_ = 0;
            tx_index_ = 0;
            SPDR = tx_frame_[0];
            return;
        }

        if (!transfer_active_)
        {
            // Align first SPI byte to frame offset 0 on frame-sync assertion.
            transfer_active_ = true;
            rx_index_ = 0;
            tx_index_ = 0;
        }
    }

    const uint8_t incoming = SPDR;

    rx_frame_[rx_index_] = incoming;
    ++rx_index_;

    ++tx_index_;
    if (tx_index_ >= protocol::kFrameSize)
    {
        tx_index_ = 0;
    }
    SPDR = tx_frame_[tx_index_];

    if (rx_index_ >= protocol::kFrameSize)
    {
        rx_index_ = 0;
        frame_ready_ = true;
    }
}

} // namespace transport
} // namespace robot
