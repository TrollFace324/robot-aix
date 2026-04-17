#pragma once

#include <Arduino.h>
#include <PRIZM.h>

#include "drive_mecanum.h"

class MotorsPrizmExp
{
public:
    enum class Wheel : uint8_t
    {
        FL = 0,
        FR = 1,
        RL = 2,
        RR = 3
    };

    MotorsPrizmExp(uint8_t right_id, uint8_t left_id);

    void begin();
    void onStepperTargetReached();
    void refreshStartupAuxOutputs(uint32_t now_ms);
    void setWheelSpeedLimitDps(int16_t limit_dps);
    void setWheelPower(const WheelCmd &cmd);
    void setMainMotorPower(uint8_t channel, int16_t power);
    void stopAll();
    void resetEncoders();
    bool readEncoder(Wheel wheel, int32_t &count);
    bool readEncoders(int32_t &fl, int32_t &fr, int32_t &rl, int32_t &rr);
    uint16_t readMainBatteryCentivolts();
    void readMainMotorCurrentsMilliAmps(uint16_t &m1_ma, uint16_t &m2_ma);
    void setStatusLeds(bool green_on, bool red_on);

private:
    uint8_t right_id_;
    uint8_t left_id_;
    int16_t wheel_speed_limit_dps_ = 420;
    uint32_t last_startup_aux_refresh_ms_ = 0;
    int16_t servo6_hold_position_deg_ = 180;

    bool invert_fl_ = false;
    bool invert_fr_ = true;  // Right side inversion for correct forward/turn kinematics
    bool invert_rl_ = false;
    bool invert_rr_ = true;  // Right side inversion for correct forward/turn kinematics
};
