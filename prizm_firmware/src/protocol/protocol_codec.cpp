#include "protocol_types.h"

#include <string.h>

namespace robot
{
namespace protocol
{
namespace
{
constexpr uint8_t kCrcOffset = 29;

bool decodeAlignedFrame(const uint8_t frame[kFrameSize], Packet &packet)
{
    if (frame[0] != kSof)
    {
        return false;
    }

    const uint16_t rx_crc = static_cast<uint16_t>(frame[kCrcOffset] << 8) | frame[kCrcOffset + 1];
    const uint16_t calc_crc = crc16Ccitt(frame, kCrcOffset);
    if (rx_crc != calc_crc)
    {
        return false;
    }

    const uint8_t len = frame[8];
    if (len > kPayloadMax)
    {
        return false;
    }

    packet.version = frame[1];
    packet.type = static_cast<MessageType>(frame[2]);
    packet.flags = frame[3];
    packet.source = static_cast<NodeId>(frame[4]);
    packet.target = static_cast<NodeId>(frame[5]);
    packet.seq = frame[6];
    packet.ack_seq = frame[7];
    packet.length = len;
    memset(packet.payload, 0, kPayloadMax);
    memcpy(packet.payload, &frame[9], len);
    return true;
}
}

uint16_t crc16Ccitt(const uint8_t *data, size_t size)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < size; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t b = 0; b < 8; ++b)
        {
            if (crc & 0x8000)
            {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

bool encodePacket(const Packet &packet, uint8_t frame[kFrameSize])
{
    if (packet.length > kPayloadMax)
    {
        return false;
    }

    memset(frame, 0, kFrameSize);
    frame[0] = kSof;
    frame[1] = packet.version;
    frame[2] = static_cast<uint8_t>(packet.type);
    frame[3] = packet.flags;
    frame[4] = static_cast<uint8_t>(packet.source);
    frame[5] = static_cast<uint8_t>(packet.target);
    frame[6] = packet.seq;
    frame[7] = packet.ack_seq;
    frame[8] = packet.length;
    memcpy(&frame[9], packet.payload, packet.length);

    const uint16_t crc = crc16Ccitt(frame, kCrcOffset);
    frame[kCrcOffset] = static_cast<uint8_t>(crc >> 8);
    frame[kCrcOffset + 1] = static_cast<uint8_t>(crc & 0xFF);
    return true;
}

bool decodePacket(const uint8_t frame[kFrameSize], Packet &packet, LinkHealth *health)
{
    if (decodeAlignedFrame(frame, packet))
    {
        if (health)
        {
            ++health->rx_packets;
        }
        return true;
    }

    // No external SS mode may rotate the 32-byte frame.
    // Try all offsets and accept frame when SOF+CRC match.
    uint8_t rotated[kFrameSize];
    for (uint8_t offset = 1; offset < kFrameSize; ++offset)
    {
        for (uint8_t i = 0; i < kFrameSize; ++i)
        {
            rotated[i] = frame[(offset + i) % kFrameSize];
        }

        if (!decodeAlignedFrame(rotated, packet))
        {
            continue;
        }

        if (health)
        {
            ++health->rx_packets;
            ++health->stale_packets;
        }
        return true;
    }

    if (health)
    {
        if (frame[0] == kSof)
        {
            ++health->crc_errors;
        }
        else
        {
            ++health->decode_errors;
        }
    }
    return false;
}

} // namespace protocol
} // namespace robot
