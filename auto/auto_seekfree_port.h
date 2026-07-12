#ifndef AUTO_SEEKFREE_PORT_H
#define AUTO_SEEKFREE_PORT_H

#include <stdint.h>

#include "auto_platform.h"

void auto_seekfree_platform_init(void);
auto_platform_t auto_seekfree_platform_create(void);

void auto_seekfree_imu_set_heading(float heading_deg);
float auto_seekfree_imu_get_heading(void);
void auto_seekfree_imu_reset_heading_from_mag(void);
void auto_seekfree_encoder_reset_distance(void);
void auto_seekfree_encoder_get_debug_counts(int16_t *left_count, int16_t *right_count);

#endif
