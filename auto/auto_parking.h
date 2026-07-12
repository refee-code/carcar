#ifndef AUTO_PARKING_H
#define AUTO_PARKING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "auto_types.h"

typedef struct {
    float speed_percent;
    float steer_percent;
    float distance_cm;
    uint32_t timeout_ms;
} auto_parking_stage_t;

typedef struct {
    const auto_parking_stage_t *stages;
    size_t stage_count;
    size_t current_stage;
    uint32_t stage_start_ms;
    float stage_start_distance_cm;
    bool active;
    bool done;
} auto_parking_t;

void auto_parking_init(auto_parking_t *parking);
void auto_parking_start(auto_parking_t *parking,
                        uint32_t now_ms,
                        float current_distance_cm);
auto_status_t auto_parking_update(auto_parking_t *parking,
                                  const auto_pose_t *pose,
                                  uint32_t now_ms,
                                  auto_drive_cmd_t *cmd_out);
bool auto_parking_is_done(const auto_parking_t *parking);

#endif
