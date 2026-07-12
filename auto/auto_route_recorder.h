#ifndef AUTO_ROUTE_RECORDER_H
#define AUTO_ROUTE_RECORDER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "auto_types.h"
#include "auto_config.h"

typedef struct {
    auto_pose_t pose;
    float speed_percent;
    uint8_t flags;
} auto_recorded_waypoint_t;

typedef struct {
    auto_recorded_waypoint_t points[AUTO_ROUTE_RECORDER_MAX_POINTS];
    uint32_t point_count;
    bool active;
    bool dirty;
} auto_route_recorder_t;

void auto_route_recorder_init(auto_route_recorder_t *recorder);
void auto_route_recorder_begin(auto_route_recorder_t *recorder);
void auto_route_recorder_end(auto_route_recorder_t *recorder);
void auto_route_recorder_clear(auto_route_recorder_t *recorder);
void auto_route_recorder_undo(auto_route_recorder_t *recorder);
void auto_route_recorder_capture(auto_route_recorder_t *recorder,
                                 const auto_pose_t *pose,
                                 float speed_percent,
                                 uint8_t flags);
bool auto_route_recorder_is_active(const auto_route_recorder_t *recorder);
bool auto_route_recorder_take_dirty(auto_route_recorder_t *recorder);
void auto_route_recorder_format_line(const auto_route_recorder_t *recorder,
                                     const auto_pose_t *live_pose,
                                     unsigned line_index,
                                     char *buffer,
                                     size_t buffer_size);
void auto_route_recorder_print_c_array(const auto_route_recorder_t *recorder);

#endif
