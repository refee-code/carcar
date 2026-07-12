#ifndef AUTO_APP_H
#define AUTO_APP_H

#include <stdbool.h>

#include "auto_platform.h"
#include "auto_pose.h"
#include "auto_safety.h"
#include "auto_task_subject1.h"
#include "auto_types.h"

typedef struct {
    auto_platform_t port;
    auto_pose_estimator_t pose_estimator;
    auto_safety_t safety;
    auto_subject1_t subject1;
    auto_drive_cmd_t last_cmd;
    bool initialized;
    bool running;
} auto_app_t;

auto_status_t Auto_Init(auto_app_t *app, const auto_platform_t *platform);
auto_status_t Auto_Start(auto_app_t *app);
auto_status_t Auto_Stop(auto_app_t *app);
auto_status_t Auto_Update10ms(auto_app_t *app);
auto_status_t Auto_UpdatePoseOnly(auto_app_t *app);
void          Auto_ResetPose(auto_app_t *app);   /* 录点开始前调用，确保路径点坐标从(0,0)开始 */
auto_subject1_state_t Auto_GetState(const auto_app_t *app);
const auto_pose_t *Auto_GetPose(const auto_app_t *app);
auto_status_t Auto_GetDiag(const auto_app_t *app, auto_diag_t *diag);

#endif
