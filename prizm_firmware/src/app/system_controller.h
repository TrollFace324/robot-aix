#pragma once

#include <Arduino.h>

#include "app/compute_link_client.h"
#include "app/startup_state_machine.h"
#include "autonomous/mission_runner.h"
#include "control/mode_manager.h"
#include "drive/control_settings_store.h"
#include "drive/heading_hold_controller.h"
#include "drive/heading_lock_manager.h"
#include "drive/wheel_state_observer.h"
#include "rc_crsf.h"
#include "safety/run_lock_guard.h"
#include "status/led_status_manager.h"
#include "transport/spi_master_backend.h"

class MotorsPrizmExp;

namespace robot
{
namespace app
{
class SystemController
{
public:
    SystemController();

    void begin();
    void loop();

private:
    bool encoderCalibrationMode() const;
    void runEncoderCalibration(uint32_t now_ms);
    void setRawPrizmLeds(bool green, bool red);
    void traceBoot(const __FlashStringHelper *msg);
    void updateStartup(uint32_t now_ms);
    void updateControl(uint32_t now_ms);
    void applyRunLock();

    transport::SpiMasterBackend link_;
    ComputeLinkClient compute_link_;
    startup::StartupStateMachine startup_sm_;
    safety::RunLockGuard run_lock_;
    control::ModeManager mode_manager_;
    drive::ControlSettingsStore control_settings_;
    drive::HeadingHoldConfig hold_config_{};
    drive::HeadingLockManager heading_lock_manager_;
    drive::HeadingHoldController heading_hold_;
    drive::WheelStateObserver wheel_observer_;
    autonomous::MissionRunner mission_;
    status::LedStatusManager led_status_;

    RcCrsf rc_;
    RcChannels rc_state_{};
    MotorsPrizmExp *motors_ = nullptr;
    bool rc_enabled_ = true;

    uint32_t last_loop_ms_ = 0;
    uint32_t last_encoder_calibration_print_ms_ = 0;
    uint32_t last_compute_summary_tx_ms_ = 0;
    bool init_done_ = false;
    bool operator_was_overriding_ = false;
    bool last_stepper_active_ = false;
    bool last_sent_start_gate_open_ = false;
    bool last_sent_crsf_fresh_ = false;
    uint16_t last_sent_battery_cV_ = 0;
};

} // namespace app
} // namespace robot
