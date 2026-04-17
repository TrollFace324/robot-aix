#include "safety/run_lock_guard.h"

namespace robot
{
namespace safety
{
void RunLockGuard::setRunLocked(bool locked)
{
    run_locked_ = locked;
}

bool RunLockGuard::runLocked() const
{
    return run_locked_;
}

} // namespace safety
} // namespace robot
