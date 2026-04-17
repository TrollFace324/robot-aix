#include "motors_prizm_exp.h"

namespace
{
PRIZM prizm;
EXPANSION expansion;
constexpr int kStartupServo6Opposite90DegPosition = 180;
constexpr int kStepperTargetReachedServo6ReversePosition = 0;
constexpr int kStartupCrServo1Stop = 0;
constexpr int kPrizmServoCenterPosition = 90;
constexpr uint32_t kStartupAuxRefreshIntervalMs = 250;

void applyStartupAuxOutputs(int servo6_position_deg)
{
    // The vendor PRIZM library suppresses duplicate servo writes by caching the
    // last angle. Reset the cache so periodic refreshes always hit the bus and
    // keep the servo latched at the requested position.
    prizm.lastPosition_6 = kPrizmServoCenterPosition;
    prizm.setServoPosition(6, servo6_position_deg);
    prizm.setCRServoState(1, kStartupCrServo1Stop);
}

int16_t applyInvert(int16_t value, bool invert)
{
    return invert ? static_cast<int16_t>(-value) : value;
}

int32_t applyInvert32(int32_t value, bool invert)
{
    return invert ? -value : value;
}

int16_t clampPower(int16_t value)
{
    if (value > 100)
    {
        return 100;
    }
    if (value < -100)
    {
        return -100;
    }
    return value;
}

uint16_t clampUnsigned16(int value)
{
    if (value < 0)
    {
        return 0;
    }
    if (value > 65535)
    {
        return 65535;
    }
    return static_cast<uint16_t>(value);
}

long scaleToSpeedDps(int16_t normalized_power, int16_t limit_dps)
{
    int16_t clamped = clampPower(normalized_power);
    if (clamped > -2 && clamped < 2)
    {
        return 0;
    }
    if (limit_dps < 0)
    {
        limit_dps = static_cast<int16_t>(-limit_dps);
    }
    return static_cast<long>(clamped) * static_cast<long>(limit_dps) / 100L;
}
} // namespace

MotorsPrizmExp::MotorsPrizmExp(uint8_t right_id, uint8_t left_id)
    : right_id_(right_id), left_id_(left_id)
{
}

void MotorsPrizmExp::begin()
{
    prizm.PrizmBegin();
    servo6_hold_position_deg_ = kStartupServo6Opposite90DegPosition;
    applyStartupAuxOutputs(servo6_hold_position_deg_);

    stopAll();
    resetEncoders();
}

void MotorsPrizmExp::onStepperTargetReached()
{
    servo6_hold_position_deg_ = kStepperTargetReachedServo6ReversePosition;
    applyStartupAuxOutputs(servo6_hold_position_deg_);
}

void MotorsPrizmExp::refreshStartupAuxOutputs(uint32_t now_ms)
{
    if ((now_ms - last_startup_aux_refresh_ms_) < kStartupAuxRefreshIntervalMs)
    {
        return;
    }

    applyStartupAuxOutputs(servo6_hold_position_deg_);
    last_startup_aux_refresh_ms_ = now_ms;
}

void MotorsPrizmExp::setWheelSpeedLimitDps(int16_t limit_dps)
{
    if (limit_dps < 0)
    {
        limit_dps = static_cast<int16_t>(-limit_dps);
    }
    wheel_speed_limit_dps_ = (limit_dps > 0) ? limit_dps : 1;
}

void MotorsPrizmExp::setWheelPower(const WheelCmd &cmd)
{
    const int16_t fl = applyInvert(cmd.fl, invert_fl_);
    const int16_t fr = applyInvert(cmd.fr, invert_fr_);
    const int16_t rl = applyInvert(cmd.rl, invert_rl_);
    const int16_t rr = applyInvert(cmd.rr, invert_rr_);
    const long fl_speed = scaleToSpeedDps(fl, wheel_speed_limit_dps_);
    const long fr_speed = scaleToSpeedDps(fr, wheel_speed_limit_dps_);
    const long rl_speed = scaleToSpeedDps(rl, wheel_speed_limit_dps_);
    const long rr_speed = scaleToSpeedDps(rr, wheel_speed_limit_dps_);

    // Physical mapping confirmed on hardware with robot front at the top:
    // ID1.M1 -> RR (right lower), ID1.M2 -> FR (right upper)
    // ID2.M1 -> RL (left lower),  ID2.M2 -> FL (left upper)
    expansion.setMotorSpeeds(right_id_, rr_speed, fr_speed);
    expansion.setMotorSpeeds(left_id_, rl_speed, fl_speed);
}

void MotorsPrizmExp::setMainMotorPower(uint8_t channel, int16_t power)
{
    const uint8_t safe_channel = (channel == 2) ? 2 : 1;
    prizm.setMotorPower(safe_channel, clampPower(power));
}

void MotorsPrizmExp::stopAll()
{
    expansion.setMotorPowers(right_id_, 0, 0);
    expansion.setMotorPowers(left_id_, 0, 0);
}

void MotorsPrizmExp::resetEncoders()
{
    expansion.resetEncoders(right_id_);
    expansion.resetEncoders(left_id_);
}

bool MotorsPrizmExp::readEncoder(Wheel wheel, int32_t &count)
{
    switch (wheel)
    {
    case Wheel::FL:
        count = applyInvert32(static_cast<int32_t>(expansion.readEncoderCount(left_id_, 2)), invert_fl_);
        return true;
    case Wheel::FR:
        count = applyInvert32(static_cast<int32_t>(expansion.readEncoderCount(right_id_, 2)), invert_fr_);
        return true;
    case Wheel::RL:
        count = applyInvert32(static_cast<int32_t>(expansion.readEncoderCount(left_id_, 1)), invert_rl_);
        return true;
    case Wheel::RR:
        count = applyInvert32(static_cast<int32_t>(expansion.readEncoderCount(right_id_, 1)), invert_rr_);
        return true;
    default:
        break;
    }
    count = 0;
    return false;
}

bool MotorsPrizmExp::readEncoders(int32_t &fl, int32_t &fr, int32_t &rl, int32_t &rr)
{
    return readEncoder(Wheel::FL, fl) &&
           readEncoder(Wheel::FR, fr) &&
           readEncoder(Wheel::RL, rl) &&
           readEncoder(Wheel::RR, rr);
}

uint16_t MotorsPrizmExp::readMainBatteryCentivolts()
{
    return clampUnsigned16(prizm.readBatteryVoltage());
}

void MotorsPrizmExp::readMainMotorCurrentsMilliAmps(uint16_t &m1_ma, uint16_t &m2_ma)
{
    // PRIZM returns motor current in milliamps.
    m1_ma = clampUnsigned16(prizm.readMotorCurrent(1));
    m2_ma = clampUnsigned16(prizm.readMotorCurrent(2));
}

void MotorsPrizmExp::setStatusLeds(bool green_on, bool red_on)
{
    prizm.setGreenLED(green_on ? HIGH : LOW);
    prizm.setRedLED(red_on ? HIGH : LOW);
}
