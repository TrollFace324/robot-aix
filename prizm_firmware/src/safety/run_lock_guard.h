#pragma once

#include <Arduino.h>

namespace robot
{
namespace safety
{
class RunLockGuard
{
public:
    void setRunLocked(bool locked);
    bool runLocked() const;

private:
    bool run_locked_ = true;
};

} // namespace safety
} // namespace robot
