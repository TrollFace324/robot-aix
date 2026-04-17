#pragma once

#include <stdint.h>

class PRIZM
{
public:
    int lastPosition_1 = 90;
    int lastPosition_2 = 90;
    int lastPosition_3 = 90;
    int lastPosition_4 = 90;
    int lastPosition_5 = 90;
    int lastPosition_6 = 90;

    static inline uint32_t prizm_begin_calls = 0;
    static inline uint32_t set_servo_position_calls = 0;
    static inline uint32_t set_servo_positions_calls = 0;
    static inline int last_servo_channel = -1;
    static inline int servo_position_1 = -1;
    static inline int servo_position_2 = -1;
    static inline int servo_position_3 = -1;
    static inline int servo_position_4 = -1;
    static inline int servo_position_5 = -1;
    static inline int servo_position_6 = -1;
    static inline uint32_t set_cr_servo_state_calls = 0;
    static inline int last_cr_servo_channel = -1;
    static inline int last_cr_servo_speed = 999;
    static inline int last_main_motor_channel = -1;
    static inline int last_main_motor_power = 0;
    static inline int green_led_state = 0;
    static inline int red_led_state = 0;

    static void resetTestState()
    {
        prizm_begin_calls = 0;
        set_servo_position_calls = 0;
        set_servo_positions_calls = 0;
        last_servo_channel = -1;
        servo_position_1 = -1;
        servo_position_2 = -1;
        servo_position_3 = -1;
        servo_position_4 = -1;
        servo_position_5 = -1;
        servo_position_6 = -1;
        set_cr_servo_state_calls = 0;
        last_cr_servo_channel = -1;
        last_cr_servo_speed = 999;
        last_main_motor_channel = -1;
        last_main_motor_power = 0;
        green_led_state = 0;
        red_led_state = 0;
    }

    void PrizmBegin(void)
    {
        ++prizm_begin_calls;
    }

    void setServoPosition(int channel, int servoposition)
    {
        ++set_servo_position_calls;
        last_servo_channel = channel;
        if (channel == 1)
        {
            lastPosition_1 = servoposition;
            servo_position_1 = servoposition;
        }
        else if (channel == 2)
        {
            lastPosition_2 = servoposition;
            servo_position_2 = servoposition;
        }
        else if (channel == 3)
        {
            lastPosition_3 = servoposition;
            servo_position_3 = servoposition;
        }
        else if (channel == 4)
        {
            lastPosition_4 = servoposition;
            servo_position_4 = servoposition;
        }
        else if (channel == 5)
        {
            lastPosition_5 = servoposition;
            servo_position_5 = servoposition;
        }
        else if (channel == 6)
        {
            lastPosition_6 = servoposition;
            servo_position_6 = servoposition;
        }
    }

    void setServoPositions(int servoposition1,
                           int servoposition2,
                           int servoposition3,
                           int servoposition4,
                           int servoposition5,
                           int servoposition6)
    {
        ++set_servo_positions_calls;
        lastPosition_1 = servoposition1;
        lastPosition_2 = servoposition2;
        lastPosition_3 = servoposition3;
        lastPosition_4 = servoposition4;
        lastPosition_5 = servoposition5;
        lastPosition_6 = servoposition6;
        servo_position_1 = servoposition1;
        servo_position_2 = servoposition2;
        servo_position_3 = servoposition3;
        servo_position_4 = servoposition4;
        servo_position_5 = servoposition5;
        servo_position_6 = servoposition6;
    }

    void setCRServoState(int channel, int servospeed)
    {
        ++set_cr_servo_state_calls;
        last_cr_servo_channel = channel;
        last_cr_servo_speed = servospeed;
    }

    void setMotorPower(int channel, int power)
    {
        last_main_motor_channel = channel;
        last_main_motor_power = power;
    }

    int readBatteryVoltage(void)
    {
        return 0;
    }

    int readMotorCurrent(int)
    {
        return 0;
    }

    void setGreenLED(int state)
    {
        green_led_state = state;
    }

    void setRedLED(int state)
    {
        red_led_state = state;
    }
};

class EXPANSION
{
public:
    static inline uint32_t set_motor_powers_calls = 0;
    static inline uint32_t reset_encoders_calls = 0;
    static inline int last_motor_power_address = -1;
    static inline int last_motor_power_1 = 0;
    static inline int last_motor_power_2 = 0;
    static inline int last_reset_encoders_address = -1;
    static inline int last_motor_speed_address = -1;
    static inline long last_motor_speed_1 = 0;
    static inline long last_motor_speed_2 = 0;

    static void resetTestState()
    {
        set_motor_powers_calls = 0;
        reset_encoders_calls = 0;
        last_motor_power_address = -1;
        last_motor_power_1 = 0;
        last_motor_power_2 = 0;
        last_reset_encoders_address = -1;
        last_motor_speed_address = -1;
        last_motor_speed_1 = 0;
        last_motor_speed_2 = 0;
    }

    void setMotorPowers(int address, int power1, int power2)
    {
        ++set_motor_powers_calls;
        last_motor_power_address = address;
        last_motor_power_1 = power1;
        last_motor_power_2 = power2;
    }

    void resetEncoders(int address)
    {
        ++reset_encoders_calls;
        last_reset_encoders_address = address;
    }

    void setMotorSpeeds(int address, long speed1, long speed2)
    {
        last_motor_speed_address = address;
        last_motor_speed_1 = speed1;
        last_motor_speed_2 = speed2;
    }

    long readEncoderCount(int, int)
    {
        return 0;
    }
};
