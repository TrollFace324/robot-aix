#pragma once

#include "motors_prizm_exp.h"

namespace legacy_adapter
{
class MotorsPrizmAdapter
{
public:
    explicit MotorsPrizmAdapter(uint8_t right_id, uint8_t left_id)
        : motors_(right_id, left_id)
    {
    }

    MotorsPrizmExp &impl() { return motors_; }

private:
    MotorsPrizmExp motors_;
};
} // namespace legacy_adapter
