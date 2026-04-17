#pragma once

#include <Arduino.h>

#include "protocol_types.h"

namespace robot
{
namespace transport
{
class IFrameLink
{
public:
    virtual ~IFrameLink() = default;

    virtual void begin() = 0;
    virtual void poll(uint32_t now_ms) = 0;
    virtual bool sendPacket(const protocol::Packet &packet) = 0;
    virtual bool receivePacket(protocol::Packet &packet) = 0;
    virtual const protocol::LinkHealth &health() const = 0;
};

} // namespace transport
} // namespace robot
