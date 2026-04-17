#include "transport/spi_master_backend.h"

#include "config/pin_map_prizm.h"
#include "config/debug_config.h"
#include <string.h>

namespace robot
{
namespace transport
{
namespace
{
constexpr uint8_t kSpiCsSetupUs = 4;
constexpr uint8_t kSpiInterByteGapUs = 8;
constexpr uint8_t kSpiCsHoldUs = 4;

void printHexByte(uint8_t value)
{
    if (value < 0x10)
    {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

void traceRawFrame(const __FlashStringHelper *label, const uint8_t *frame)
{
    if (!config::kEnableSpiUartTrace)
    {
        return;
    }

    Serial.print(label);
    for (uint8_t i = 0; i < protocol::kFrameSize; ++i)
    {
        if (i != 0)
        {
            Serial.print(' ');
        }
        printHexByte(frame[i]);
    }
    Serial.println();
}
} // namespace

SpiMasterBackend::SpiMasterBackend(uint8_t cs_pin)
    : cs_pin_(cs_pin)
{
}

void SpiMasterBackend::begin()
{
    pinMode(config::kSpiHwSsPin, OUTPUT);
    // Master mode safety: keep HW SS high.
    // Driving SS low on master side is unnecessary and may create board-specific side effects.
    digitalWrite(config::kSpiHwSsPin, HIGH);

    if (config::kSpiUseExternalCs)
    {
        pinMode(cs_pin_, OUTPUT);
        digitalWrite(cs_pin_, HIGH);
    }
    SPI.begin();
    tx_pending_ = false;
    rx_pending_ = false;
}

void SpiMasterBackend::poll(uint32_t)
{
    if (!tx_pending_)
    {
        return;
    }

    // Conservative clock for bench bring-up with long wires / no-CS mode.
    SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
    if (config::kSpiUseExternalCs)
    {
        digitalWrite(cs_pin_, LOW);
        delayMicroseconds(kSpiCsSetupUs);
    }
    for (uint8_t i = 0; i < protocol::kFrameSize; ++i)
    {
        rx_frame_[i] = SPI.transfer(tx_frame_[i]);
        if (i + 1 < protocol::kFrameSize)
        {
            // AVR slave firmware loads the next outbound byte from SPI_STC_vect.
            // Leave a small inter-byte service window so the slave does not echo
            // the previous MOSI byte back on MISO.
            delayMicroseconds(kSpiInterByteGapUs);
        }
    }
    if (config::kSpiUseExternalCs)
    {
        delayMicroseconds(kSpiCsHoldUs);
        digitalWrite(cs_pin_, HIGH);
    }
    SPI.endTransaction();

    traceRawFrame(F("SPI RAW TX "), tx_frame_);
    traceRawFrame(F("SPI RAW RX "), rx_frame_);

    tx_pending_ = false;
    rx_pending_ = true;
}

bool SpiMasterBackend::sendPacket(const protocol::Packet &packet)
{
    if (tx_pending_)
    {
        ++health_.retries;
        return false;
    }

    if (!protocol::encodePacket(packet, tx_frame_))
    {
        ++health_.decode_errors;
        return false;
    }

    tx_pending_ = true;
    ++health_.tx_packets;
    return true;
}

bool SpiMasterBackend::receivePacket(protocol::Packet &packet)
{
    if (!rx_pending_)
    {
        return false;
    }

    rx_pending_ = false;
    return protocol::decodePacket(rx_frame_, packet, &health_);
}

const protocol::LinkHealth &SpiMasterBackend::health() const
{
    return health_;
}

} // namespace transport
} // namespace robot
