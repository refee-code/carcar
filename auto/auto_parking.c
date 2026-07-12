#include "auto_parking.h"

#include <string.h>

#include "auto_config.h"
#include "auto_math.h"

static const auto_parking_stage_t s_default_stages[] = {
    { AUTO_REVERSE_SPEED_PERCENT,  70.0f, AUTO_PARK_REVERSE_TURN_CM,    AUTO_PARK_STAGE_TIMEOUT_MS },
    { AUTO_REVERSE_SPEED_PERCENT, -35.0f, AUTO_PARK_REVERSE_COUNTER_CM, AUTO_PARK_STAGE_TIMEOUT_MS },
    { AUTO_REVERSE_SPEED_PERCENT,   0.0f, AUTO_PARK_REVERSE_FINAL_CM,   AUTO_PARK_STAGE_TIMEOUT_MS },
    { 0.0f,                         0.0f, 0.0f,                         300u }
};

void auto_parking_init(auto_parking_t *parking)
{
    if (parking == 0) {
        return;
    }

    memset(parking, 0, sizeof(*parking));
    parking->stages = s_default_stages;
    parking->stage_count = sizeof(s_default_stages) / sizeof(s_default_stages[0]);
}

void auto_parking_start(auto_parking_t *parking,
                        uint32_t now_ms,
                        float current_distance_cm)
{
    if (parking == 0) {
        return;
    }

    parking->current_stage = 0u;
    parking->stage_start_ms = now_ms;
    parking->stage_start_distance_cm = current_distance_cm;
    parking->active = true;
    parking->done = false;
}

auto_status_t auto_parking_update(auto_parking_t *parking,
                                  const auto_pose_t *pose,
                                  uint32_t now_ms,
                                  auto_drive_cmd_t *cmd_out)
{
    const auto_parking_stage_t *stage;
    float traveled_cm;
    uint32_t elapsed_ms;
    bool distance_done;
    bool timeout_done;

    if (parking == 0 || pose == 0 || cmd_out == 0) {
        return AUTO_ERR_INVALID_ARG;
    }

    if (!parking->active) {
        auto_parking_start(parking, now_ms, pose->distance_cm);
    }

    if (parking->done || parking->current_stage >= parking->stage_count) {
        cmd_out->speed_percent = 0.0f;
        cmd_out->steer_percent = 0.0f;
        cmd_out->brake = true;
        parking->done = true;
        return AUTO_OK;
    }

    stage = &parking->stages[parking->current_stage];
    traveled_cm = auto_absf(pose->distance_cm - parking->stage_start_distance_cm);
    elapsed_ms = now_ms - parking->stage_start_ms;

    distance_done = (stage->distance_cm > 0.0f) && (traveled_cm >= stage->distance_cm);
    timeout_done = (stage->timeout_ms > 0u) && (elapsed_ms >= stage->timeout_ms);

    if (distance_done || timeout_done) {
        parking->current_stage++;
        parking->stage_start_ms = now_ms;
        parking->stage_start_distance_cm = pose->distance_cm;

        if (parking->current_stage >= parking->stage_count) {
            parking->done = true;
            cmd_out->speed_percent = 0.0f;
            cmd_out->steer_percent = 0.0f;
            cmd_out->brake = true;
            return AUTO_OK;
        }

        stage = &parking->stages[parking->current_stage];
    }

    cmd_out->speed_percent = stage->speed_percent;
    cmd_out->steer_percent = stage->steer_percent;
    cmd_out->brake = false;
    return AUTO_OK;
}

bool auto_parking_is_done(const auto_parking_t *parking)
{
    return parking != 0 && parking->done;
}
