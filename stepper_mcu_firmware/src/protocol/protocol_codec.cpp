#include "protocol_types.h"

#include <string.h>

namespace robot
{
namespace protocol
{
namespace
{
constexpr uint8_t kI2cCrcOffset = 22;
}

uint16_t crc16Ccitt(const uint8_t *data, size_t size)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < size; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint8_t crc8(const uint8_t *data, size_t size)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < size; ++i)
    {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; ++b)
        {
            if (crc & 0x80)
            {
                crc = static_cast<uint8_t>((crc << 1) ^ 0xD5);
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

bool encodeI2cCommand(const I2cCommandFrame &command, uint8_t out[kI2cFrameSize])
{
    if (command.length > kI2cPayloadMax)
    {
        return false;
    }

    memset(out, 0, kI2cFrameSize);
    out[0] = kI2cSof;
    out[1] = command.version;
    out[2] = static_cast<uint8_t>(command.cmd);
    out[3] = command.seq;
    out[4] = command.length;
    memcpy(&out[5], command.payload, command.length);
    out[kI2cCrcOffset] = crc8(out, kI2cCrcOffset);
    return true;
}

bool decodeI2cCommand(const uint8_t in[kI2cFrameSize], I2cCommandFrame &command)
{
    if (in[0] != kI2cSof)
    {
        return false;
    }
    if (in[kI2cCrcOffset] != crc8(in, kI2cCrcOffset))
    {
        return false;
    }

    const uint8_t len = in[4];
    if (len > kI2cPayloadMax)
    {
        return false;
    }

    command.sof = in[0];
    command.version = in[1];
    command.cmd = static_cast<StepperCommand>(in[2]);
    command.seq = in[3];
    command.length = len;
    memset(command.payload, 0, kI2cPayloadMax);
    memcpy(command.payload, &in[5], len);
    return true;
}

bool encodeI2cStatus(const I2cStatusFrame &status, uint8_t out[kI2cFrameSize])
{
    if (status.length > kI2cPayloadMax)
    {
        return false;
    }

    memset(out, 0, kI2cFrameSize);
    out[0] = kI2cSof;
    out[1] = status.version;
    out[2] = static_cast<uint8_t>(status.status);
    out[3] = status.seq;
    out[4] = status.length;
    memcpy(&out[5], status.payload, status.length);
    out[kI2cCrcOffset] = crc8(out, kI2cCrcOffset);
    return true;
}

bool decodeI2cStatus(const uint8_t in[kI2cFrameSize], I2cStatusFrame &status)
{
    if (in[0] != kI2cSof)
    {
        return false;
    }
    if (in[kI2cCrcOffset] != crc8(in, kI2cCrcOffset))
    {
        return false;
    }

    const uint8_t len = in[4];
    if (len > kI2cPayloadMax)
    {
        return false;
    }

    status.sof = in[0];
    status.version = in[1];
    status.status = static_cast<StepperStatusCode>(in[2]);
    status.seq = in[3];
    status.length = len;
    memset(status.payload, 0, kI2cPayloadMax);
    memcpy(status.payload, &in[5], len);
    return true;
}

} // namespace protocol
} // namespace robot
