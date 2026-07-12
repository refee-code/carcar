#ifndef AUTO_MATH_H
#define AUTO_MATH_H

#include <stdbool.h>

float auto_clampf(float value, float min_value, float max_value);
float auto_absf(float value);
float auto_distance_cm(float ax, float ay, float bx, float by);
float auto_heading_to_deg(float from_x, float from_y, float to_x, float to_y);
float auto_normalize_angle_deg(float angle_deg);
bool auto_angle_near_deg(float a_deg, float b_deg, float tolerance_deg);

#endif
