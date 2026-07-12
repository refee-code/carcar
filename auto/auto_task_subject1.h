#ifndef AUTO_TASK_SUBJECT1_H
#define AUTO_TASK_SUBJECT1_H

#include <stdbool.h>
#include <stdint.h>

#include "auto_nav.h"
#include "auto_parking.h"
#include "auto_safety.h"
#include "auto_types.h"

typedef struct {
    auto_subject1_state_t state;
    uint32_t state_enter_ms;
    auto_nav_t slalom_nav;
    auto_nav_t garage_nav;
    auto_parking_t parking;
    auto_drive_cmd_t last_cmd;
    auto_nav_result_t last_nav_result;
    bool last_nav_valid;
    float heading_error_deg;
    auto_status_t fault;
} auto_subject1_t;

void auto_subject1_init(auto_subject1_t *task);
void auto_subject1_start(auto_subject1_t *task, uint32_t now_ms);
void auto_subject1_stop(auto_subject1_t *task, uint32_t now_ms);
auto_status_t auto_subject1_update(auto_subject1_t *task,
                                   const auto_pose_t *pose,
                                   auto_status_t safety_status,
                                   uint32_t now_ms,
                                   auto_drive_cmd_t *cmd_out);
auto_subject1_state_t auto_subject1_state(const auto_subject1_t *task);
uint32_t auto_subject1_target_index(const auto_subject1_t *task);
float auto_subject1_heading_error_deg(const auto_subject1_t *task);
bool auto_subject1_last_nav_result(const auto_subject1_t *task,
                                   auto_nav_result_t *result);
uint32_t auto_subject1_parking_stage(const auto_subject1_t *task);
auto_status_t auto_subject1_fault(const auto_subject1_t *task);

#endif
