#pragma once

#include <Arduino.h>

namespace robot
{
namespace station
{
class Hc12Stub
{
public:
    void begin(Stream *stream) { stream_ = stream; }
    void update() {}

private:
    Stream *stream_ = nullptr;
};

} // namespace station
} // namespace robot
