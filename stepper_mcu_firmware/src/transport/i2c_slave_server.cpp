#include "transport/i2c_slave_server.h"

#include <string.h>
#include <Wire.h>

namespace robot
{
namespace transport
{
I2cSlaveServer *I2cSlaveServer::instance_ = nullptr;

void I2cSlaveServer::begin(uint8_t address)
{
    instance_ = this;
    address_ = address;
    memset((void *)rx_raw_, 0, sizeof(rx_raw_));
    memset((void *)tx_raw_, 0, sizeof(tx_raw_));
    command_ready_ = false;
    rx_events_ = 0;
    tx_events_ = 0;

    Wire.begin(address_);
    Wire.onReceive(handleReceiveThunk);
    Wire.onRequest(handleRequestThunk);
}

void I2cSlaveServer::poll()
{
}

bool I2cSlaveServer::popCommand(protocol::I2cCommandFrame &command)
{
    if (!command_ready_)
    {
        return false;
    }

    uint8_t raw[protocol::kI2cFrameSize]{};
    noInterrupts();
    memcpy(raw, (const void *)rx_raw_, sizeof(raw));
    command_ready_ = false;
    interrupts();

    return protocol::decodeI2cCommand(raw, command);
}

void I2cSlaveServer::publishStatus(const protocol::I2cStatusFrame &status)
{
    uint8_t raw[protocol::kI2cFrameSize]{};
    if (!protocol::encodeI2cStatus(status, raw))
    {
        return;
    }

    noInterrupts();
    memcpy((void *)tx_raw_, raw, sizeof(raw));
    interrupts();
}

uint16_t I2cSlaveServer::rxEventCount() const
{
    return rx_events_;
}

uint16_t I2cSlaveServer::txEventCount() const
{
    return tx_events_;
}

void I2cSlaveServer::handleReceiveThunk(int num_bytes)
{
    if (instance_)
    {
        instance_->handleReceive(num_bytes);
    }
}

void I2cSlaveServer::handleRequestThunk()
{
    if (instance_)
    {
        instance_->handleRequest();
    }
}

void I2cSlaveServer::handleReceive(int num_bytes)
{
    uint8_t idx = 0;
    while (idx < protocol::kI2cFrameSize && Wire.available() && idx < static_cast<uint8_t>(num_bytes))
    {
        rx_raw_[idx] = static_cast<uint8_t>(Wire.read());
        ++idx;
    }
    // drain any surplus bytes
    while (Wire.available())
    {
        Wire.read();
    }
    // zero-pad remainder of frame buffer
    while (idx < protocol::kI2cFrameSize)
    {
        rx_raw_[idx++] = 0;
    }

    ++rx_events_;
    if (num_bytes > 0)
    {
        command_ready_ = true;
    }
}

void I2cSlaveServer::handleRequest()
{
    // tx_raw_ is safe to read directly: the TWI ISR fires with interrupts
    // masked, so the main-loop noInterrupts()/interrupts() critical section
    // in publishStatus() cannot interleave with this callback.
    ++tx_events_;
    Wire.write((const uint8_t *)tx_raw_, protocol::kI2cFrameSize);
}

} // namespace transport
} // namespace robot
