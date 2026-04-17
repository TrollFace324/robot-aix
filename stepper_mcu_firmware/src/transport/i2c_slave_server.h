#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "protocol_types.h"

namespace robot
{
namespace transport
{
class I2cSlaveServer
{
public:
    void begin(uint8_t address);
    void poll();

    bool popCommand(protocol::I2cCommandFrame &command);
    void publishStatus(const protocol::I2cStatusFrame &status);
    uint16_t rxEventCount() const;
    uint16_t txEventCount() const;

private:
    static void handleReceiveThunk(int num_bytes);
    static void handleRequestThunk();
    void handleReceive(int num_bytes);
    void handleRequest();

    static I2cSlaveServer *instance_;

    volatile bool command_ready_ = false;
    volatile uint8_t rx_raw_[protocol::kI2cFrameSize]{};
    volatile uint8_t tx_raw_[protocol::kI2cFrameSize]{};
    volatile uint16_t rx_events_ = 0;
    volatile uint16_t tx_events_ = 0;
    uint8_t address_ = 0;
};

} // namespace transport
} // namespace robot
