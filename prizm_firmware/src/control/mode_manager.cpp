#include "control/mode_manager.h"

namespace robot
{
namespace control
{
void ModeManager::begin()
{
    mode_ = Mode::MANUAL;
    combo_start_ms_ = 0;
    combo_latched_ = false;
}

void ModeManager::update(uint32_t now_ms, const ModeInputs &inputs)
{
    static_cast<void>(now_ms);
    static_cast<void>(inputs);
    mode_ = Mode::MANUAL;
    combo_start_ms_ = 0;
    combo_latched_ = false;
}

Mode ModeManager::mode() const
{
    return mode_;
}

} // namespace control
} // namespace robot
