#pragma once

#include <Arduino.h>

#include "config/kran_slot_config.h"
#include "config/stepper_limits.h"
#include "motion_profile.h"
#include "protocol_types.h"
#include "stepper/homing_controller.h"
#include "stepper/stepper_driver_adapter.h"

namespace robot
{
namespace stepper
{
class MotionController
{
public:
    void begin(uint8_t pin_step, uint8_t pin_dir, uint8_t pin_enable, uint8_t pin_home, const MotionProfile &profile,
               int16_t min_mm, int16_t max_mm,
               const int16_t *slot_targets_mm = nullptr);
    void update(uint32_t now_ms);

    void handleCommand(protocol::StepperCommand cmd, int16_t arg0, int16_t arg1);
    protocol::StepperStatusPayload status() const;
    bool tryGetSlotTarget(uint8_t slot, int16_t &target_mm) const;
    bool copySlotTargets(int16_t out[config::kKranSlotCount]) const;
    void setSlotTargetSilently(uint8_t slot, int16_t target_mm);
    void setConfigReply(protocol::StepperConfigReplyType type,
                        int16_t slot,
                        int16_t mm,
                        protocol::StepperConfigResult result);

private:
    enum class HomingPhase : uint8_t
    {
        Idle = 0,
        ReleaseSwitch = 1,
        Clearance = 2,
        SeekHome = 3
    };

    int16_t clampMm(int16_t value) const;
    void loadSlotTargets(const int16_t *slot_targets_mm);
    bool canEditSlotConfig() const;
    bool isValidTravelMm(int16_t value) const;
    bool tryResolveSlotTarget(int16_t slot, int16_t &target_mm) const;
    void refreshStatusSnapshot();
    void beginHomeSequence();
    void processHoming();
    void enterReleaseSwitchPhase();
    void enterSeekHomePhase(bool allow_cold_start_seed);
    void failHoming(uint8_t fault_flag);
    void completeHoming();
    void cancelHoming();

    StepperDriverAdapter driver_;
    HomingController homing_;
    protocol::StepperStatusPayload status_{};
    int16_t slot_targets_mm_[config::kKranSlotCount]{};

    int16_t min_mm_ = 0;
    int16_t max_mm_ = config::kMaxTravelMm;
    uint8_t stop_mode_ = 1;
    HomingPhase homing_phase_ = HomingPhase::Idle;
    int16_t homing_phase_target_mm_ = 0;
    bool has_completed_home_ = false;
};

} // namespace stepper
} // namespace robot
