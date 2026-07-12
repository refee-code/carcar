#include "auto_task_subject1.h"

#include <string.h>

#include "auto_config.h"
#include "auto_math.h"
#include "auto_route.h"

static auto_drive_cmd_t make_stop_cmd(bool brake)
{
    auto_drive_cmd_t cmd;
    cmd.speed_percent = 0.0f;
    cmd.steer_percent = 0.0f;
    cmd.brake = brake;
    return cmd;
}

static void enter_state(auto_subject1_t *task,
                        auto_subject1_state_t state,
                        uint32_t now_ms)
{
    task->state = state;
    task->state_enter_ms = now_ms;
}

void auto_subject1_init(auto_subject1_t *task)
{
    if (task == 0) {
        return;
    }

    memset(task, 0, sizeof(*task));
    task->state = AUTO_STATE_IDLE;
    task->last_cmd = make_stop_cmd(true);
    auto_nav_init(&task->slalom_nav);
    auto_nav_init(&task->garage_nav);
    auto_parking_init(&task->parking);
}

void auto_subject1_start(auto_subject1_t *task, uint32_t now_ms)
{
    if (task == 0) {
        return;
    }

    auto_nav_init(&task->slalom_nav);
    auto_nav_init(&task->garage_nav);
    auto_parking_init(&task->parking);
    task->fault = AUTO_OK;
    task->last_cmd = make_stop_cmd(true);
    task->last_nav_result = (auto_nav_result_t){0};
    task->last_nav_valid = false;
    enter_state(task, AUTO_STATE_START, now_ms);
}

void auto_subject1_stop(auto_subject1_t *task, uint32_t now_ms)
{
    if (task == 0) {
        return;
    }

    task->last_cmd = make_stop_cmd(true);
    enter_state(task, AUTO_STATE_STOP, now_ms);
}

auto_status_t auto_subject1_update(auto_subject1_t *task,
                                   const auto_pose_t *pose,
                                   auto_status_t safety_status,
                                   uint32_t now_ms,
                                   auto_drive_cmd_t *cmd_out)
{
    auto_nav_result_t nav_result;
    auto_status_t status;
    float heading_error;

    if (task == 0 || pose == 0 || cmd_out == 0) {
        return AUTO_ERR_INVALID_ARG;
    }

    if (safety_status != AUTO_OK) {
        task->fault = safety_status;
        enter_state(task, AUTO_STATE_FAULT, now_ms);
    }

    switch (task->state) {
    case AUTO_STATE_IDLE:
        task->last_cmd = make_stop_cmd(true);
        break;

    case AUTO_STATE_START:
        task->last_cmd = make_stop_cmd(false);
        if ((now_ms - task->state_enter_ms) >= AUTO_START_SETTLE_MS) {
            enter_state(task, AUTO_STATE_SLALOM, now_ms);
        }
        break;

    case AUTO_STATE_SLALOM:
        status = auto_nav_follow(&task->slalom_nav,
                                 auto_route_subject1_slalom(),
                                 pose,
                                 &nav_result);
        if (status != AUTO_OK) {
            task->fault = status;
            enter_state(task, AUTO_STATE_FAULT, now_ms);
            task->last_cmd = make_stop_cmd(true);
            break;
        }
        task->last_cmd = nav_result.cmd;
        task->last_nav_result = nav_result;
        task->last_nav_valid = true;
        task->heading_error_deg = nav_result.heading_error_deg;
        if (nav_result.route_done) {
            if (auto_route_is_dynamic()) {
                /* Recorded route: stop after all points done */
                enter_state(task, AUTO_STATE_STOP, now_ms);
            } else {
                /* Static route: continue to garage approach */
                enter_state(task, AUTO_STATE_GARAGE_APPROACH, now_ms);
            }
        }
        break;

    case AUTO_STATE_GARAGE_APPROACH:
        status = auto_nav_follow(&task->garage_nav,
                                 auto_route_subject1_garage_approach(),
                                 pose,
                                 &nav_result);
        if (status != AUTO_OK) {
            task->fault = status;
            enter_state(task, AUTO_STATE_FAULT, now_ms);
            task->last_cmd = make_stop_cmd(true);
            break;
        }
        task->last_cmd = nav_result.cmd;
        task->last_nav_result = nav_result;
        task->last_nav_valid = true;
        task->heading_error_deg = nav_result.heading_error_deg;
        if (nav_result.route_done) {
            enter_state(task, AUTO_STATE_ALIGN_GARAGE, now_ms);
        }
        break;

    case AUTO_STATE_ALIGN_GARAGE:
        heading_error = auto_normalize_angle_deg(AUTO_GARAGE_APPROACH_HEADING_DEG -
                                                 pose->heading_deg);
        if (auto_absf(heading_error) <= AUTO_GARAGE_ALIGN_TOL_DEG) {
            auto_parking_start(&task->parking, now_ms, pose->distance_cm);
            enter_state(task, AUTO_STATE_REVERSE_PARK, now_ms);
            task->last_cmd = make_stop_cmd(false);
            break;
        }
        if ((now_ms - task->state_enter_ms) > AUTO_GARAGE_ALIGN_TIMEOUT_MS) {
            task->fault = AUTO_ERR_ROUTE_TIMEOUT;
            enter_state(task, AUTO_STATE_FAULT, now_ms);
            task->last_cmd = make_stop_cmd(true);
            break;
        }
        task->last_cmd.speed_percent = AUTO_ALIGN_SPEED_PERCENT;
        task->last_cmd.steer_percent =
            auto_clampf(heading_error * AUTO_HEADING_KP, -100.0f, 100.0f);
        task->last_cmd.brake = false;
        task->heading_error_deg = heading_error;
        break;

    case AUTO_STATE_REVERSE_PARK:
        status = auto_parking_update(&task->parking, pose, now_ms, &task->last_cmd);
        if (status != AUTO_OK) {
            task->fault = status;
            enter_state(task, AUTO_STATE_FAULT, now_ms);
            task->last_cmd = make_stop_cmd(true);
            break;
        }
        if (auto_parking_is_done(&task->parking)) {
            enter_state(task, AUTO_STATE_STOP, now_ms);
            task->last_cmd = make_stop_cmd(true);
        }
        break;

    case AUTO_STATE_STOP:
        task->last_cmd = make_stop_cmd(true);
        break;

    case AUTO_STATE_FAULT:
    default:
        task->last_cmd = make_stop_cmd(true);
        break;
    }

    *cmd_out = task->last_cmd;
    return task->fault;
}

auto_subject1_state_t auto_subject1_state(const auto_subject1_t *task)
{
    return task == 0 ? AUTO_STATE_FAULT : task->state;
}

uint32_t auto_subject1_target_index(const auto_subject1_t *task)
{
    if (task == 0) {
        return 0u;
    }

    if (task->state == AUTO_STATE_GARAGE_APPROACH) {
        return (uint32_t)task->garage_nav.target_index;
    }

    return (uint32_t)task->slalom_nav.target_index;
}

float auto_subject1_heading_error_deg(const auto_subject1_t *task)
{
    return task == 0 ? 0.0f : task->heading_error_deg;
}

bool auto_subject1_last_nav_result(const auto_subject1_t *task,
                                   auto_nav_result_t *result)
{
    if (task == 0 || result == 0) {
        return false;
    }

    if (!task->last_nav_valid) {
        return false;
    }

    *result = task->last_nav_result;
    return true;
}

uint32_t auto_subject1_parking_stage(const auto_subject1_t *task)
{
    return task == 0 ? 0u : (uint32_t)task->parking.current_stage;
}

auto_status_t auto_subject1_fault(const auto_subject1_t *task)
{
    return task == 0 ? AUTO_ERR_INVALID_ARG : task->fault;
}
