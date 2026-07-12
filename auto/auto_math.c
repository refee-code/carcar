#include "auto_math.h"

#include <math.h>

#define AUTO_RAD_TO_DEG (57.29577951308232f)

float auto_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

float auto_absf(float value)
{
    return value < 0.0f ? -value : value;
}

float auto_distance_cm(float ax, float ay, float bx, float by)
{
    const float dx = bx - ax;
    const float dy = by - ay;
    return sqrtf(dx * dx + dy * dy);
}

float auto_heading_to_deg(float from_x, float from_y, float to_x, float to_y)
{
    return atan2f(to_y - from_y, to_x - from_x) * AUTO_RAD_TO_DEG;
}

float auto_normalize_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

bool auto_angle_near_deg(float a_deg, float b_deg, float tolerance_deg)
{
    return auto_absf(auto_normalize_angle_deg(a_deg - b_deg)) <= tolerance_deg;
}
