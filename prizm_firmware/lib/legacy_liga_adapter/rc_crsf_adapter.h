#pragma once

#include "rc_crsf.h"

namespace legacy_adapter
{
class RcCrsfAdapter
{
public:
    void begin(HardwareSerial &serial, uint32_t baud) { rc_.begin(serial, baud); }
    bool poll(RcChannels &out) { return rc_.poll(out); }

private:
    RcCrsf rc_;
};
} // namespace legacy_adapter
