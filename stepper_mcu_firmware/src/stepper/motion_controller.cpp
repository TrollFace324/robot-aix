#include "stepper/motion_controller.h"

#include <string.h>

#include "config/kran_slot_config.h"
#include "config/stepper_limits.h"

namespace robot
{
namespace stepper
{
namespace
{
constexpr uint8_t kFaultHomeSwitchReleaseTimeout = 0x01;
constexpr uint8_t kFaultHomeSwitchNotFound = 0x02;
}

void MotionController::begin(uint8_t pin_step, uint8_t pin_dir, uint8_t pin_enable, uint8_t pin_home, const MotionProfile &profile,
                             int16_t min_mm, int16_t max_mm,
                             const int16_t *slot_targets_mm)
{
    min_mm_ = min_mm;
    max_mm_ = max_mm;
    loadSlotTargets(slot_targets_mm);
    driver_.begin(pin_step, pin_dir, pin_enable, profile);
    homing_.begin(pin_home, config::kHomeSwitchActiveLow);

    status_ = protocol::StepperStatusPayload{};
    status_.status = protocol::StepperStatusCode::STATUS_IDLE;
    status_.driver_enabled = driver_.enabled();
    homing_phase_ = HomingPhase::Idle;
    homing_phase_target_mm_ = min_mm_;
    has_completed_home_ = false;
}

void MotionController::update(uint32_t)
{
    driver_.update();
    refreshStatusSnapshot();

    if (status_.status == protocol::StepperStatusCode::STATUS_HOMING)
    {
        processHoming();
        refreshStatusSnapshot();
        return;
    }

    if (driver_.isMoving())
    {
        if (status_.status != protocol::StepperStatusCode::STATUS_HOMING)
        {
            status_.status = protocol::StepperStatusCode::STATUS_MOVING;
        }
    }
    else if (status_.status == protocol::StepperStatusCode::STATUS_MOVING)
    {
        status_.status = protocol::StepperStatusCode::STATUS_IDLE;
    }
}

void MotionController::handleCommand(protocol::StepperCommand cmd, int16_t arg0, int16_t arg1)
{
    if (cmd != protocol::StepperCommand::CMD_GET_SLOT_MM &&
        cmd != protocol::StepperCommand::CMD_SET_SLOT_MM)
    {
        setConfigReply(protocol::StepperConfigReplyType::None,
                       -1,
                       0,
                       protocol::StepperConfigResult::Ok);
    }

    switch (cmd)
    {
    case protocol::StepperCommand::CMD_HOME:
        beginHomeSequence();
        break;

    case protocol::StepperCommand::CMD_GOTO_MM:
        cancelHoming();
        status_.fault_flags = 0;
        driver_.enable(true);
        driver_.setTargetMm(static_cast<float>(clampMm(arg0)));
        status_.status = protocol::StepperStatusCode::STATUS_MOVING;
        break;

    case protocol::StepperCommand::CMD_GOTO_SLOT:
    {
        int16_t target_mm = 0;
        if (!tryResolveSlotTarget(arg0, target_mm))
        {
            break;
        }

        cancelHoming();
        status_.fault_flags = 0;
        driver_.enable(true);
        driver_.setTargetMm(static_cast<float>(target_mm));
        status_.status = protocol::StepperStatusCode::STATUS_MOVING;
        break;
    }

    case protocol::StepperCommand::CMD_GET_SLOT_MM:
    {
        int16_t target_mm = 0;
        if (!tryResolveSlotTarget(arg0, target_mm))
        {
            setConfigReply(protocol::StepperConfigReplyType::ReadSlot,
                           arg0,
                           0,
                           protocol::StepperConfigResult::InvalidSlot);
            break;
        }

        setConfigReply(protocol::StepperConfigReplyType::ReadSlot,
                       arg0,
                       target_mm,
                       protocol::StepperConfigResult::Ok);
        break;
    }

    case protocol::StepperCommand::CMD_SET_SLOT_MM:
        if (!canEditSlotConfig())
        {
            setConfigReply(protocol::StepperConfigReplyType::WriteSlot,
                           arg0,
                           arg1,
                           protocol::StepperConfigResult::Busy);
            break;
        }

        if (arg0 < 0 ||
            arg0 >= static_cast<int16_t>(config::kKranSlotCount))
        {
            setConfigReply(protocol::StepperConfigReplyType::WriteSlot,
                           arg0,
                           arg1,
                           protocol::StepperConfigResult::InvalidSlot);
            break;
        }

        if (!isValidTravelMm(arg1))
        {
            setConfigReply(protocol::StepperConfigReplyType::WriteSlot,
                           arg0,
                           arg1,
                           protocol::StepperConfigResult::InvalidMm);
            break;
        }

        slot_targets_mm_[arg0] = arg1;
        setConfigReply(protocol::StepperConfigReplyType::WriteSlot,
                       arg0,
                       arg1,
                       protocol::StepperConfigResult::Ok);
        break;

    case protocol::StepperCommand::CMD_EXTEND_MM:
        cancelHoming();
        status_.fault_flags = 0;
        driver_.enable(true);
        driver_.setTargetMm(static_cast<float>(clampMm(status_.current_position_mm + arg0)));
        status_.status = protocol::StepperStatusCode::STATUS_MOVING;
        break;

    case protocol::StepperCommand::CMD_RETRACT_MM:
        cancelHoming();
        status_.fault_flags = 0;
        driver_.enable(true);
        driver_.setTargetMm(static_cast<float>(clampMm(status_.current_position_mm - arg0)));
        status_.status = protocol::StepperStatusCode::STATUS_MOVING;
        break;

    case protocol::StepperCommand::CMD_STOP:
        cancelHoming();
        driver_.setTargetMm(static_cast<float>(status_.current_position_mm));
        if (stop_mode_ == 0)
        {
            driver_.enable(false);
        }
        else
        {
            driver_.enable(true);
        }
        status_.status = protocol::StepperStatusCode::STATUS_STOPPED;
        break;

    case protocol::StepperCommand::CMD_SET_LIMITS:
        min_mm_ = arg0;
        max_mm_ = arg1;
        break;

    case protocol::StepperCommand::CMD_SET_STOP_MODE:
        stop_mode_ = static_cast<uint8_t>(arg0);
        break;

    case protocol::StepperCommand::CMD_REQUEST_STATUS:
    default:
        break;
    }
}

protocol::StepperStatusPayload MotionController::status() const
{
    return status_;
}

bool MotionController::tryGetSlotTarget(uint8_t slot, int16_t &target_mm) const
{
    if (slot >= config::kKranSlotCount)
    {
        return false;
    }

    target_mm = slot_targets_mm_[slot];
    return true;
}

bool MotionController::copySlotTargets(
    int16_t out[config::kKranSlotCount]) const
{
    if (out == nullptr)
    {
        return false;
    }

    memcpy(out, slot_targets_mm_, sizeof(slot_targets_mm_));
    return true;
}

void MotionController::setSlotTargetSilently(uint8_t slot, int16_t target_mm)
{
    if (slot >= config::kKranSlotCount)
    {
        return;
    }

    slot_targets_mm_[slot] = clampMm(target_mm);
}

void MotionController::setConfigReply(protocol::StepperConfigReplyType type,
                                      int16_t slot,
                                      int16_t mm,
                                      protocol::StepperConfigResult result)
{
    status_.cfg_reply_type = type;
    status_.cfg_slot =
        (slot < 0) ? 0xFF : static_cast<uint8_t>((slot > 255) ? 255 : slot);
    status_.cfg_mm = mm;
    status_.cfg_result = result;
}

void MotionController::loadSlotTargets(const int16_t *slot_targets_mm)
{
    if (slot_targets_mm == nullptr)
    {
        memcpy(slot_targets_mm_,
               config::kKranSlotTargetMm,
               sizeof(slot_targets_mm_));
        return;
    }

    memcpy(slot_targets_mm_, slot_targets_mm, sizeof(slot_targets_mm_));
}

bool MotionController::canEditSlotConfig() const
{
    return status_.status != protocol::StepperStatusCode::STATUS_MOVING &&
           status_.status != protocol::StepperStatusCode::STATUS_HOMING;
}

bool MotionController::isValidTravelMm(int16_t value) const
{
    return value >= min_mm_ && value <= max_mm_;
}

bool MotionController::tryResolveSlotTarget(int16_t slot, int16_t &target_mm) const
{
    if (slot < 0 ||
        slot >= static_cast<int16_t>(config::kKranSlotCount))
    {
        return false;
    }

    target_mm = clampMm(slot_targets_mm_[slot]);
    return true;
}

void MotionController::refreshStatusSnapshot()
{
    status_.current_position_mm = static_cast<int16_t>(driver_.currentMm());
    status_.target_position_mm = static_cast<int16_t>(driver_.targetMm());
    status_.home_switch_state = homing_.homeSwitchActive();
    status_.driver_enabled = driver_.enabled();
}

void MotionController::beginHomeSequence()
{
    driver_.enable(true);
    status_.fault_flags = 0;
    const bool home_switch_active = homing_.homeSwitchActive();

    if (home_switch_active && has_completed_home_)
    {
        completeHoming();
        return;
    }

    status_.is_homed = false;
    status_.status = protocol::StepperStatusCode::STATUS_HOMING;

    if (home_switch_active)
    {
        enterReleaseSwitchPhase();
        return;
    }

    enterSeekHomePhase(!has_completed_home_);
}

void MotionController::processHoming()
{
    switch (homing_phase_)
    {
    case HomingPhase::ReleaseSwitch:
        if (!status_.home_switch_state)
        {
            homing_phase_ = HomingPhase::Clearance;
            homing_phase_target_mm_ = clampMm(status_.current_position_mm + config::kHomeBackoffMm);
            driver_.setTargetMm(static_cast<float>(homing_phase_target_mm_));
            return;
        }

        if (!driver_.isMoving() || status_.current_position_mm >= homing_phase_target_mm_)
        {
            failHoming(kFaultHomeSwitchReleaseTimeout);
        }
        return;

    case HomingPhase::Clearance:
        if (driver_.isMoving())
        {
            return;
        }

        enterSeekHomePhase(false);
        return;

    case HomingPhase::SeekHome:
        if (status_.home_switch_state)
        {
            completeHoming();
            return;
        }

        if (!driver_.isMoving())
        {
            failHoming(kFaultHomeSwitchNotFound);
        }
        return;

    case HomingPhase::Idle:
    default:
        failHoming(kFaultHomeSwitchNotFound);
        return;
    }
}

void MotionController::enterReleaseSwitchPhase()
{
    homing_phase_ = HomingPhase::ReleaseSwitch;
    homing_phase_target_mm_ = clampMm(status_.current_position_mm + config::kHomeBackoffMm);
    driver_.setTargetMm(static_cast<float>(homing_phase_target_mm_));
}

void MotionController::enterSeekHomePhase(bool allow_cold_start_seed)
{
    homing_phase_ = HomingPhase::SeekHome;
    homing_phase_target_mm_ = min_mm_;

    if (allow_cold_start_seed &&
        !status_.home_switch_state &&
        !status_.is_homed &&
        driver_.currentMm() <= static_cast<float>(min_mm_))
    {
        // Logical position after boot can match min even when the mechanics are
        // physically away from the switch. Seed an away-from-home position so a
        // real sweep into the switch happens.
        driver_.setCurrentMm(static_cast<float>(max_mm_));
    }

    driver_.setTargetMm(static_cast<float>(min_mm_));
}

void MotionController::failHoming(uint8_t fault_flag)
{
    homing_phase_ = HomingPhase::Idle;
    homing_phase_target_mm_ = status_.current_position_mm;
    driver_.setTargetMm(static_cast<float>(status_.current_position_mm));
    status_.fault_flags = fault_flag;
    status_.status = protocol::StepperStatusCode::STATUS_ERROR;
    status_.is_homed = false;
}

void MotionController::completeHoming()
{
    homing_phase_ = HomingPhase::Idle;
    homing_phase_target_mm_ = min_mm_;
    driver_.setCurrentMm(static_cast<float>(min_mm_));
    status_.current_position_mm = min_mm_;
    status_.target_position_mm = min_mm_;
    status_.is_homed = true;
    status_.status = protocol::StepperStatusCode::STATUS_IDLE;
    status_.fault_flags = 0;
    has_completed_home_ = true;
}

void MotionController::cancelHoming()
{
    homing_phase_ = HomingPhase::Idle;
    homing_phase_target_mm_ = status_.current_position_mm;
}

int16_t MotionController::clampMm(int16_t value) const
{
    if (value < min_mm_)
    {
        return min_mm_;
    }
    if (value > max_mm_)
    {
        return max_mm_;
    }
    return value;
}

} // namespace stepper
} // namespace robot
