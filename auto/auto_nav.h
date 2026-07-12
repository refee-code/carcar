#ifndef AUTO_NAV_H
#define AUTO_NAV_H

#include <stdbool.h>
#include <stddef.h>

#include "auto_route.h"
#include "auto_types.h"

typedef struct {
    size_t nearest_index;
    size_t target_index;
    bool initialized;
    unsigned dynamic_cte_abort_ticks;
    bool prev_driving_reverse;  /* 滞回：记录上一帧方向，防止前进/倒车频繁切换 */
} auto_nav_t;

typedef struct {
    auto_drive_cmd_t cmd;
    size_t nearest_index;
    size_t target_index;
    float target_distance_cm;
    float heading_error_deg;
    float cross_track_error_cm;
    float cross_track_steer_percent;
    float target_x_cm;
    float target_y_cm;
    float target_heading_deg;
    float segment_t;
    bool route_done;
} auto_nav_result_t;

void auto_nav_init(auto_nav_t *nav);
auto_status_t auto_nav_follow(auto_nav_t *nav,
                              const auto_route_t *route,
                              const auto_pose_t *pose,
                              auto_nav_result_t *out);

#endif
