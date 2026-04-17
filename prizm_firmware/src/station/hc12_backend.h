#pragma once

#include <Arduino.h>

namespace robot
{
namespace station
{
class Hc12Backend
{
public:
    void begin(Stream &stream);
    void poll();
    bool available() const;

private:
    Stream *stream_ = nullptr;
    bool available_ = false;
};

} // namespace station
} // namespace robot
