#include "auto_app.h"

#include <string.h>

#include "auto_chassis.h"

auto_status_t Auto_Init(auto_app_t *app, const auto_platform_t *platform)
{
    if (app == 0 || platform == 0) {
        return AUTO_ERR_INVALID_ARG;
    }

    if (!auto_platform_has_required_outputs(platform)) {
        return AUTO_ERR_PLATFORM;
    }

    memset(app, 0, sizeof(*app));
    app->port = *platform;
    auto_pose_init(&app->pose_estimator);
    auto_safety_init(&app->safety);
    auto_subject1_init(&app->subject1);
    app->initialized = true;
    app->running = false;
    return AUTO_OK;
}

auto_status_t Auto_Start(auto_app_t *app)
{
    uint32_t now_ms;

    if (app == 0 || !app->initialized) {
        return AUTO_ERR_INVALID_STATE;
    }

    now_ms = app->port.now_ms(app->port.ctx);
    auto_pose_reset(&app->pose_estimator, now_ms);
    auto_safety_start(&app->safety, now_ms);
    auto_subject1_start(&app->subject1, now_ms);
    app->running = true;
    return AUTO_OK;
}

auto_status_t Auto_Stop(auto_app_t *app)
{
    uint32_t now_ms;

    if (app == 0 || !app->initialized) {
        return AUTO_ERR_INVALID_STATE;
    }

    now_ms = app->port.now_ms(app->port.ctx);
    auto_subject1_stop(&app->subject1, now_ms);
    auto_chassis_stop(&app->port);
    app->running = false;
    return AUTO_OK;
}

auto_status_t Auto_Update10ms(auto_app_t *app)
{
    uint32_t now_ms;
    auto_status_t pose_status;
    auto_status_t safety_status;
    auto_status_t task_status;
    const auto_pose_t *pose;

    if (app == 0 || !app->initialized) {
        return AUTO_ERR_INVALID_STATE;
    }

    now_ms = app->port.now_ms(app->port.ctx);
    pose_status = auto_pose_update(&app->pose_estimator, &app->port, now_ms);
    if (pose_status != AUTO_OK) {
        auto_chassis_stop(&app->port);
        return pose_status;
    }

    pose = auto_pose_get(&app->pose_estimator);
    safety_status = auto_safety_check(&app->safety, &app->port, pose, now_ms);

    task_status = auto_subject1_update(&app->subject1,
                                       pose,
                                       safety_status,
                                       now_ms,
                                       &app->last_cmd);

    auto_chassis_apply(&app->port, &app->last_cmd);

    if (auto_subject1_state(&app->subject1) == AUTO_STATE_STOP ||
        auto_subject1_state(&app->subject1) == AUTO_STATE_FAULT) {
        app->running = false;
    }

    return task_status;
}

auto_status_t Auto_UpdatePoseOnly(auto_app_t *app)
{
    uint32_t now_ms;
    auto_status_t pose_status;

    if (app == 0 || !app->initialized) {
        return AUTO_ERR_INVALID_STATE;
    }

    now_ms = app->port.now_ms(app->port.ctx);
    pose_status = auto_pose_update(&app->pose_estimator, &app->port, now_ms);
    auto_chassis_stop(&app->port);
    app->last_cmd.speed_percent = 0.0f;
    app->last_cmd.steer_percent = 0.0f;
    app->last_cmd.brake = true;
    app->running = false;

    return pose_status;
}

void Auto_ResetPose(auto_app_t *app)
{
    uint32_t now_ms;

    if (app == 0 || !app->initialized) {
        return;
    }

    /* 录点开始前调用：重置 x_cm/y_cm 为(0,0)，保留当前航向。
     * 确保录下的路径点坐标和回放时的起始坐标一致。 */
    now_ms = app->port.now_ms(app->port.ctx);
    auto_pose_reset(&app->pose_estimator, now_ms);
}

auto_subject1_state_t Auto_GetState(const auto_app_t *app)
{
    return app == 0 ? AUTO_STATE_FAULT : auto_subject1_state(&app->subject1);
}

const auto_pose_t *Auto_GetPose(const auto_app_t *app)
{
    return app == 0 ? 0 : auto_pose_get(&app->pose_estimator);
}

auto_status_t Auto_GetDiag(const auto_app_t *app, auto_diag_t *diag)
{
    const auto_pose_t *pose;
    auto_nav_result_t nav_result;

    if (app == 0 || diag == 0) {
        return AUTO_ERR_INVALID_ARG;
    }

    memset(diag, 0, sizeof(*diag));

    pose = auto_pose_get(&app->pose_estimator);
    diag->state = auto_subject1_state(&app->subject1);
    diag->fault = auto_subject1_fault(&app->subject1);
    diag->pose = pose == 0 ? (auto_pose_t){0} : *pose;
    diag->cmd = app->last_cmd;
    diag->target_index = auto_subject1_target_index(&app->subject1);
    diag->heading_error_deg = auto_subject1_heading_error_deg(&app->subject1);
    if (auto_subject1_last_nav_result(&app->subject1, &nav_result)) {
        diag->nearest_index = (uint32_t)nav_result.nearest_index;
        diag->target_index = (uint32_t)nav_result.target_index;
        diag->target_distance_cm = nav_result.target_distance_cm;
        diag->heading_error_deg = nav_result.heading_error_deg;
        diag->cross_track_error_cm = nav_result.cross_track_error_cm;
        diag->cross_track_steer_percent = nav_result.cross_track_steer_percent;
        diag->nav_target_x_cm = nav_result.target_x_cm;
        diag->nav_target_y_cm = nav_result.target_y_cm;
        diag->nav_target_heading_deg = nav_result.target_heading_deg;
        diag->nav_segment_t = nav_result.segment_t;
    }
    diag->wheel_heading_delta_deg = diag->pose.wheel_heading_delta_deg;
    diag->steer_angle_deg = diag->pose.steer_angle_deg;
    diag->steer_heading_delta_deg = diag->pose.steer_heading_delta_deg;
    diag->heading_correction_deg = diag->pose.heading_correction_deg;
    diag->heading_consistency = diag->pose.heading_consistency;
    diag->park_stage = auto_subject1_parking_stage(&app->subject1);
    diag->running = app->running;

    return AUTO_OK;
}
