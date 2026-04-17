#include "app/system_controller.h"

#include "config/channel_map.h"
#include "config/debug_config.h"
#include "config/pin_map_prizm.h"
#include "config/timing_config.h"
#include "drive_mecanum.h"
#include "motors_prizm_exp.h"

namespace
{
constexpr uint8_t kExpanderRightId = 1;
constexpr uint8_t kExpanderLeftId = 2;
constexpr uint32_t kRcBaudDefault = 115200;
constexpr uint32_t kEncoderCalibrationPrintPeriodMs = 200;
constexpr uint32_t kComputeSummaryPeriodMs = 100;
constexpr uint8_t kStepperStatusIdle = 0;
constexpr uint8_t kPrizmRedLedPin = 6;
constexpr uint8_t kPrizmGreenLedPin = 7;

MotorsPrizmExp g_motors(kExpanderRightId, kExpanderLeftId);

bool isHigh(const RcChannels &ch, uint8_t idx)
{
    return ch.valid && ch.raw[idx] >= robot::config::kChannelHighThreshold;
}

bool sticksNeutral(const RcChannels &ch)
{
    return fabsf(ch.right_x) < 0.08f && fabsf(ch.right_y) < 0.08f && fabsf(ch.left_x) < 0.08f;
}

} // namespace

namespace robot
{
namespace app
{
SystemController::SystemController()
    : link_(config::kSpiCsPin),
      compute_link_(link_)
{
}

void SystemController::begin()
{
    const bool serial_console_enabled =
        encoderCalibrationMode() || config::kEnableBootSerialTrace || config::kEnableSpiUartTrace;
    if (serial_console_enabled)
    {
        Serial.begin(config::kBootSerialBaud);
        delay(20);
    }

    traceBoot(F("BOOT: enter begin()"));
    setRawPrizmLeds(true, true);
    delay(80);
    setRawPrizmLeds(false, false);

    rc_enabled_ = true;
    if (config::kEnableSpiUartTrace && config::kDisableRcWhenSpiTrace)
    {
        rc_enabled_ = false;
        if (serial_console_enabled)
        {
            Serial.println(F("BOOT: rc disabled (SPI UART trace mode)"));
        }
    }
    else
    {
        rc_.begin(Serial, kRcBaudDefault);
        traceBoot(F("BOOT: rc begin ok"));
    }

    motors_ = &g_motors;
    setRawPrizmLeds(false, true); // if freeze happens in motors_->begin(), red stays ON
    traceBoot(F("BOOT: before motors begin"));
    motors_->begin();
    bool settings_loaded_from_eeprom = false;
    hold_config_ = control_settings_.loadOrDefaults(&settings_loaded_from_eeprom);
    motors_->setWheelSpeedLimitDps(hold_config_.wheel_speed_limit_dps);
    traceBoot(F("BOOT: motors begin ok"));
    motors_->setStatusLeds(false, true);
    motors_->stopAll();

    if (encoderCalibrationMode())
    {
        rc_enabled_ = false;
        heading_lock_manager_.reset();
        heading_hold_.reset();
        wheel_observer_.reset();
        last_encoder_calibration_print_ms_ = 0;
        motors_->resetEncoders();
        motors_->setStatusLeds(true, false);
        Serial.println(F("ENCODER CALIBRATION SESSION"));
        Serial.println(F("Robot drive is disabled. Rotate wheels by hand."));
        Serial.println(F("Commands: z=reset encoders, h=help"));
        Serial.println(F("Fields: FL FR RL RR raw counts"));
        init_done_ = true;
        last_loop_ms_ = millis();
        return;
    }

    traceBoot(F("BOOT: before spi begin"));
    if (config::kEnableBootSerialTrace)
    {
        Serial.print(F("BOOT: spi cs mode="));
        Serial.print(config::kSpiUseExternalCs ? F("external") : F("none"));
        Serial.print(F(" cs_pin="));
        Serial.println(config::kSpiCsPin);
    }
    link_.begin();
    compute_link_.begin(millis());
    startup_sm_.begin(millis());
    mode_manager_.begin();
    heading_lock_manager_.reset();
    heading_hold_.configure(hold_config_);
    heading_hold_.reset();
    wheel_observer_.configure(hold_config_.encoder_poll_interval_ms,
                              hold_config_.encoder_motion_threshold_cps);
    wheel_observer_.reset();
    mission_.clear();

    rc_state_.valid = false;
    init_done_ = true;
    last_loop_ms_ = millis();
    traceBoot(F("BOOT: begin done"));
}

void SystemController::loop()
{
    const uint32_t now_ms = millis();

    if (encoderCalibrationMode())
    {
        motors_->refreshStartupAuxOutputs(now_ms);
        runEncoderCalibration(now_ms);
        return;
    }

    RcChannels parsed{};
    if (rc_enabled_ && rc_.poll(parsed))
    {
        rc_state_ = parsed;
    }

    compute_link_.poll(now_ms);
    const ComputeStatus &compute = compute_link_.status();
    if (compute.online &&
        last_stepper_active_ &&
        (compute.stepper_status_code == kStepperStatusIdle))
    {
        motors_->onStepperTargetReached();
    }
    last_stepper_active_ = compute.stepper_active;
    motors_->refreshStartupAuxOutputs(now_ms);
    wheel_observer_.update(*motors_, now_ms);

    updateStartup(now_ms);
    applyRunLock();
    updateControl(now_ms);

    const bool rc_fresh = rc_state_.valid && ((now_ms - rc_state_.last_update_ms) <= config::kRcFlsafeMs);
    const bool autonomous_active = (mode_manager_.mode() == control::Mode::AUTONOMOUS);
    const status::LedOutput led = led_status_.evaluate(now_ms, startup_sm_.state(), rc_fresh, autonomous_active);
    motors_->setStatusLeds(led.green, led.red);

    const bool start_gate_open = !run_lock_.runLocked();
    const uint16_t battery_cV = motors_ ? motors_->readMainBatteryCentivolts() : 0;
    const bool summary_changed =
        (start_gate_open != last_sent_start_gate_open_) ||
        (rc_fresh != last_sent_crsf_fresh_) ||
        (battery_cV != last_sent_battery_cV_);
    if (summary_changed || (now_ms - last_compute_summary_tx_ms_ >= kComputeSummaryPeriodMs))
    {
        if (compute_link_.sendSetModeSummary(start_gate_open, rc_fresh, battery_cV))
        {
            last_compute_summary_tx_ms_ = now_ms;
            last_sent_start_gate_open_ = start_gate_open;
            last_sent_crsf_fresh_ = rc_fresh;
            last_sent_battery_cV_ = battery_cV;
        }
    }

    last_loop_ms_ = now_ms;
}

void SystemController::updateStartup(uint32_t now_ms)
{
    const ComputeStatus &compute = compute_link_.status();

    startup::StartupInputs in;
    in.init_done = init_done_;
    in.transport_online = compute.online;
    in.handshake_ok = compute.hello_ok && compute.capabilities_ok;
    in.sensor_check_ok = compute.self_check_ok && compute.ready_to_run;
    in.sensor_check_pending = compute.imu_healthy && compute.imu_calibrating && !compute.tilt_fault;
    // A missing compute link is recoverable and should drive automatic retry,
    // not a latched FAULT_LOCK. Reserve fault_detected for explicit hard faults.
    // Tilt guard only latches FAULT_LOCK in autonomous mode; manual/RC driving
    // must stay under the operator's control even past the tilt threshold.
    const bool autonomous_active = (mode_manager_.mode() == control::Mode::AUTONOMOUS);
    in.fault_detected = compute.online && compute.tilt_fault && autonomous_active;

    startup_sm_.update(now_ms, in);
    run_lock_.setRunLocked(startup_sm_.outputs().run_locked);
}

void SystemController::updateControl(uint32_t now_ms)
{
    const bool rc_fresh = rc_state_.valid && ((now_ms - rc_state_.last_update_ms) <= config::kRcFlsafeMs);
    const ComputeStatus &compute = compute_link_.status();
    float dt_s = 0.005f;
    if (last_loop_ms_ != 0)
    {
        const uint32_t dt_ms = now_ms - last_loop_ms_;
        const uint32_t clamped_dt_ms = (dt_ms < 1U) ? 1U : ((dt_ms > 40U) ? 40U : dt_ms);
        dt_s = static_cast<float>(clamped_dt_ms) * 0.001f;
    }

    control::ModeInputs mode_inputs;
    mode_inputs.run_locked = run_lock_.runLocked();
    mode_inputs.rc_fresh = rc_fresh;
    mode_inputs.ch10_high = isHigh(rc_state_, config::kChHeadingHold);
    mode_inputs.ch3_high = isHigh(rc_state_, config::kChConfig);
    mode_inputs.sticks_neutral = sticksNeutral(rc_state_);
    mode_inputs.operator_override = fabsf(rc_state_.left_x) > 0.20f;
    mode_manager_.update(now_ms, mode_inputs);

    if (run_lock_.runLocked() || !rc_fresh)
    {
        heading_lock_manager_.reset();
        heading_hold_.reset();
        operator_was_overriding_ = false;
        motors_->stopAll();
        return;
    }

    drive::HeadingLockInputs hold_inputs;
    hold_inputs.ch10_high = mode_inputs.ch10_high;
    hold_inputs.compute_online = compute.online;
    hold_inputs.yaw_valid = compute.yaw_valid;
    hold_inputs.tilt_fault = compute.tilt_fault;
    hold_inputs.stepper_active = compute.stepper_active;
    hold_inputs.imu_sample_age_ms = compute.imu_sample_age_ms;
    heading_lock_manager_.update(hold_inputs, hold_config_.imu_fresh_age_ms);

    if (heading_lock_manager_.faultLatched())
    {
        heading_hold_.reset();
        // Fall through to manual — corrected_wz is already rc_state_.left_x
    }

    float corrected_wz = rc_state_.left_x;
    if (heading_lock_manager_.captureRequested())
    {
        const uint32_t imu_age = (compute.last_imu_update_ms != 0)
                                     ? (now_ms - compute.last_imu_update_ms)
                                     : 255U;
        if (imu_age <= hold_config_.imu_fresh_age_ms && compute.yaw_valid)
        {
            heading_hold_.capture(compute.yaw_deg);
        }
        else
        {
            heading_lock_manager_.reset();
        }
        heading_lock_manager_.clearCaptureRequest();
    }
    if (heading_lock_manager_.active())
    {
        const bool stick_override = fabsf(rc_state_.left_x) > 0.20f;
        if (stick_override)
        {
            corrected_wz = rc_state_.left_x;
            heading_hold_.reset();
            operator_was_overriding_ = true;
        }
        else
        {
            if (operator_was_overriding_)
            {
                heading_hold_.capture(compute.yaw_deg);
                operator_was_overriding_ = false;
            }
            corrected_wz = heading_hold_.update(compute.yaw_deg,
                                                compute.yaw_rate_dps,
                                                wheel_observer_.rotationalSpeedCountsPerSec(),
                                                wheel_observer_.rotationalBalance(),
                                                dt_s);
            if (heading_hold_.runawayDetected())
            {
                corrected_wz = rc_state_.left_x;
                heading_hold_.reset();
            }
        }
    }

    if (heading_lock_manager_.active())
    {
        constexpr float kMaxHoldWz = 0.50f;
        if (corrected_wz > kMaxHoldWz)
        {
            corrected_wz = kMaxHoldWz;
        }
        if (corrected_wz < -kMaxHoldWz)
        {
            corrected_wz = -kMaxHoldWz;
        }
    }

    ChassisCmd cmd{};
    cmd.vx = rc_state_.right_x;
    cmd.vy = rc_state_.right_y;
    cmd.wz = corrected_wz;

    const WheelCmd wheel = DriveMecanum::mix(cmd);
    motors_->setWheelPower(wheel);
}

void SystemController::applyRunLock()
{
    if (!run_lock_.runLocked())
    {
        return;
    }

    motors_->stopAll();
    motors_->setMainMotorPower(1, 0);
    motors_->setMainMotorPower(2, 0);
}

bool SystemController::encoderCalibrationMode() const
{
    return config::kEnableEncoderCalibrationSession;
}

void SystemController::runEncoderCalibration(uint32_t now_ms)
{
    while (Serial.available() > 0)
    {
        const int raw = Serial.read();
        if (raw < 0)
        {
            break;
        }

        const char cmd = static_cast<char>(raw);
        if (cmd == 'z' || cmd == 'Z')
        {
            motors_->stopAll();
            motors_->resetEncoders();
            last_encoder_calibration_print_ms_ = 0;
            Serial.println(F("ENC RESET"));
        }
        else if (cmd == 'h' || cmd == 'H')
        {
            Serial.println(F("Commands: z=reset encoders, h=help"));
            Serial.println(F("Rotate one wheel by an exact number of turns and read delta counts."));
        }
    }

    if (last_encoder_calibration_print_ms_ != 0 &&
        (now_ms - last_encoder_calibration_print_ms_) < kEncoderCalibrationPrintPeriodMs)
    {
        return;
    }

    last_encoder_calibration_print_ms_ = now_ms;

    int32_t fl = 0;
    int32_t fr = 0;
    int32_t rl = 0;
    int32_t rr = 0;
    if (!motors_->readEncoders(fl, fr, rl, rr))
    {
        Serial.println(F("ENC READ ERROR"));
        return;
    }

    Serial.print(F("ENC FL="));
    Serial.print(fl);
    Serial.print(F(" FR="));
    Serial.print(fr);
    Serial.print(F(" RL="));
    Serial.print(rl);
    Serial.print(F(" RR="));
    Serial.println(rr);
}

void SystemController::setRawPrizmLeds(bool green, bool red)
{
    pinMode(kPrizmGreenLedPin, OUTPUT);
    pinMode(kPrizmRedLedPin, OUTPUT);
    digitalWrite(kPrizmGreenLedPin, green ? HIGH : LOW);
    digitalWrite(kPrizmRedLedPin, red ? HIGH : LOW);
}

void SystemController::traceBoot(const __FlashStringHelper *msg)
{
    if (!config::kEnableBootSerialTrace)
    {
        return;
    }

    Serial.println(msg);
}

} // namespace app
} // namespace robot
