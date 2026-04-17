#include "drive_mecanum.h"

namespace
{
float clampf(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}
} // namespace

float DriveMecanum::applyDeadband(float value, float deadband)
{
    return (fabs(value) < deadband) ? 0.0f : value;
}

WheelCmd DriveMecanum::mix(const ChassisCmd &cmd)
{
    WheelCmd out{0, 0, 0, 0};

    float vx = applyDeadband(clampf(cmd.vx, -1.0f, 1.0f));
    float vy = applyDeadband(clampf(cmd.vy, -1.0f, 1.0f));
    float wz = applyDeadband(clampf(cmd.wz, -1.0f, 1.0f));

    const float planar_norm = sqrtf(vx * vx + vy * vy);
    if (planar_norm > 1.0f)
    {
        vx /= planar_norm;
        vy /= planar_norm;
    }

    float fl = vy + vx + wz;
    float fr = vy - vx - wz;
    float rl = vy - vx + wz;
    float rr = vy + vx - wz;

    float max_abs = fabs(fl);
    if (fabs(fr) > max_abs)
    {
        max_abs = fabs(fr);
    }
    if (fabs(rl) > max_abs)
    {
        max_abs = fabs(rl);
    }
    if (fabs(rr) > max_abs)
    {
        max_abs = fabs(rr);
    }
    if (max_abs < 1.0f)
    {
        max_abs = 1.0f;
    }

    fl /= max_abs;
    fr /= max_abs;
    rl /= max_abs;
    rr /= max_abs;

    out.fl = static_cast<int16_t>(fl * 100.0f);
    out.fr = static_cast<int16_t>(fr * 100.0f);
    out.rl = static_cast<int16_t>(rl * 100.0f);
    out.rr = static_cast<int16_t>(rr * 100.0f);
    return out;
}
