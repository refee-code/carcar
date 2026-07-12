#ifndef AUTO_POSE_H
#define AUTO_POSE_H

#include <stdbool.h>

#include "auto_platform.h"
#include "auto_types.h"

typedef struct {
    auto_pose_t pose;
    double origin_lat_deg;
    double origin_lon_deg;
    float last_encoder_distance_cm;
    bool origin_set;
    bool encoder_seen;
} auto_pose_estimator_t;

void auto_pose_init(auto_pose_estimator_t *estimator);
void auto_pose_reset(auto_pose_estimator_t *estimator, uint32_t now_ms);
auto_status_t auto_pose_update(auto_pose_estimator_t *estimator,
                               const auto_platform_t *port,
                               uint32_t now_ms);
const auto_pose_t *auto_pose_get(const auto_pose_estimator_t *estimator);

#endif
