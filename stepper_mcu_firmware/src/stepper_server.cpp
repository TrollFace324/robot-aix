#include "stepper_server.h"

#include "config/i2c_config.h"
#include "config/kran_slot_config.h"
#include "config/pin_map_stepper.h"
#include "config/stepper_limits.h"
#include "motion_profile.h"
#include "status/status_reporter.h"
#include "stepper/kran_slot_settings_store.h"
#include "stepper/motion_controller.h"
#include "transport/i2c_slave_server.h"

namespace
{
robot::transport::I2cSlaveServer g_i2c;
robot::stepper::MotionController g_motion;
robot::stepper::KranSlotSettingsStore g_slot_settings;
robot::status::StatusReporter g_reporter;
uint8_t g_last_seq = 0;

bool statusLedLevel(const robot::protocol::StepperStatusPayload &status, uint32_t now_ms)
{
    if (status.status == robot::protocol::StepperStatusCode::STATUS_ERROR)
    {
        return true;
    }

    const bool i2c_seen = g_i2c.rxEventCount() != 0 || g_i2c.txEventCount() != 0;
    if (!i2c_seen)
    {
        return ((now_ms / 200U) & 0x01U) != 0;
    }

    return false;
}
}

namespace robot
{
void StepperServer::begin()
{
    pinMode(config::kPinStatusLed, OUTPUT);
    digitalWrite(config::kPinStatusLed, LOW);

    // Arm the TWI slave before any motion-related setup so address ACK works
    // even if the stepper stage is the part that is unstable.
    g_i2c.begin(config::kStepperI2cAddress);
    g_i2c.publishStatus(g_reporter.buildFrame(status_.payload, 0));

    // Boot signature: 5 rapid blinks confirm the Wire-based firmware is running
    // and that Wire.begin() has been reached. Without this sequence the 200 ms
    // idle blink is visually identical to the old twi.h firmware.
    for (uint8_t i = 0; i < 5; ++i)
    {
        digitalWrite(config::kPinStatusLed, HIGH);
        delay(60);
        digitalWrite(config::kPinStatusLed, LOW);
        delay(60);
    }

    MotionProfile profile;
    int16_t slot_targets_mm[config::kKranSlotCount]{};
    g_slot_settings.loadOrDefaults(slot_targets_mm);
    g_motion.begin(config::kPinStep, config::kPinDir, config::kPinEnable, config::kPinHomeSwitch,
                   profile, config::kMinTravelMm, config::kMaxTravelMm, slot_targets_mm);
    status_.payload = g_motion.status();
    g_i2c.publishStatus(g_reporter.buildFrame(status_.payload, 0));
}

void StepperServer::loop()
{
    const uint32_t now_ms = millis();

    g_motion.update(now_ms);

    protocol::I2cCommandFrame command;
    if (g_i2c.popCommand(command))
    {
        const int16_t arg0 = (command.length >= 2) ? static_cast<int16_t>(command.payload[0] | (command.payload[1] << 8)) : 0;
        const int16_t arg1 = (command.length >= 4) ? static_cast<int16_t>(command.payload[2] | (command.payload[3] << 8)) : 0;
        int16_t previous_slot_target_mm = 0;
        const bool had_previous_slot_target =
            (arg0 >= 0) &&
            g_motion.tryGetSlotTarget(static_cast<uint8_t>(arg0),
                                      previous_slot_target_mm);

        g_motion.handleCommand(command.cmd, arg0, arg1);

        if (command.cmd == protocol::StepperCommand::CMD_SET_SLOT_MM)
        {
            const auto motion_status = g_motion.status();
            if (motion_status.cfg_reply_type ==
                    protocol::StepperConfigReplyType::WriteSlot &&
                motion_status.cfg_result ==
                    protocol::StepperConfigResult::Ok)
            {
                int16_t slot_targets_mm[config::kKranSlotCount]{};
                if (!g_motion.copySlotTargets(slot_targets_mm) ||
                    !g_slot_settings.save(slot_targets_mm))
                {
                    if (had_previous_slot_target)
                    {
                        g_motion.setSlotTargetSilently(
                            static_cast<uint8_t>(arg0),
                            previous_slot_target_mm);
                    }
                    g_motion.setConfigReply(
                        protocol::StepperConfigReplyType::WriteSlot,
                        arg0,
                        arg1,
                        protocol::StepperConfigResult::EepromSaveFail);
                }
            }
        }

        g_last_seq = command.seq;
    }

    status_.payload = g_motion.status();
    const protocol::I2cStatusFrame frame = g_reporter.buildFrame(status_.payload, g_last_seq);
    g_i2c.publishStatus(frame);

    digitalWrite(config::kPinStatusLed, statusLedLevel(status_.payload, now_ms) ? HIGH : LOW);
}

const StepperStatus &StepperServer::status() const
{
    return status_;
}

} // namespace robot
