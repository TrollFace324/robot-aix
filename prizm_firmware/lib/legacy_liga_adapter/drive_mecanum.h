#pragma once

#include <Arduino.h>

struct ChassisCmd
{
    float vx;
    float vy;
    float wz;
};

struct WheelCmd
{
    int16_t fl;
    int16_t fr;
    int16_t rl;
    int16_t rr;
};

class DriveMecanum
{
public:
    static WheelCmd mix(const ChassisCmd &cmd);
    static float applyDeadband(float value, float deadband = 0.05f);
};
