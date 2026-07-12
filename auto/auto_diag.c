#include "auto_diag.h"

#include <stdio.h>

const char *auto_diag_state_name(auto_subject1_state_t state)
{
    switch (state) {
    case AUTO_STATE_IDLE:            return "DAIJI";
    case AUTO_STATE_START:           return "QIDONG";
    case AUTO_STATE_SLALOM:          return "RAOZHUANG";
    case AUTO_STATE_GARAGE_APPROACH: return "JIERU";
    case AUTO_STATE_ALIGN_GARAGE:    return "DUIQI";
    case AUTO_STATE_REVERSE_PARK:    return "DAOCHE";
    case AUTO_STATE_STOP:            return "TINGZHI";
    case AUTO_STATE_FAULT:           return "CUOWU";
    default:                         return "WEIZHI";
    }
}

const char *auto_diag_status_name(auto_status_t status)
{
    switch (status) {
    case AUTO_OK:                 return "OK";
    case AUTO_ERR_INVALID_ARG:    return "BAD_ARG";
    case AUTO_ERR_INVALID_STATE:  return "BAD_STATE";
    case AUTO_ERR_PLATFORM:       return "PLATFORM";
    case AUTO_ERR_SENSOR_TIMEOUT: return "SENSOR_TO";
    case AUTO_ERR_ESTOP:          return "ESTOP";
    case AUTO_ERR_ROUTE_DONE:     return "ROUTE_DONE";
    case AUTO_ERR_ROUTE_TIMEOUT:  return "ROUTE_TO";
    default:                      return "UNKNOWN";
    }
}

void auto_diag_format_line(const auto_diag_t *diag,
                           unsigned line_index,
                           char *buffer,
                           size_t buffer_size)
{
    if (buffer == 0 || buffer_size == 0u) {
        return;
    }

    if (diag == 0) {
        snprintf(buffer, buffer_size, "AUTO diag null");
        return;
    }

    switch (line_index) {
    case 0u:
        snprintf(buffer, buffer_size, "[%s] %s",
                 auto_diag_state_name(diag->state),
                 auto_diag_status_name(diag->fault));
        break;
    case 1u:
        snprintf(buffer, buffer_size, "MUDIAN:%lu RUN:%u",
                 (unsigned long)diag->target_index,
                 diag->running ? 1u : 0u);
        break;
    case 2u:
        snprintf(buffer, buffer_size, "X:%.0fcm Y:%.0fcm",
                 diag->pose.x_cm,
                 diag->pose.y_cm);
        break;
    case 3u:
        snprintf(buffer, buffer_size, "HX:%.1f JL:%.0fcm",
                 diag->pose.heading_deg,
                 diag->pose.distance_cm);
        break;
    case 4u:
        snprintf(buffer, buffer_size, "SD:%.0f%% ZX:%.0f%%",
                 diag->cmd.speed_percent,
                 diag->cmd.steer_percent);
        break;
    case 5u:
        snprintf(buffer, buffer_size, "GPS:%s IMU:%s ENC:%s",
                 diag->pose.gps_valid ? "OK" : "--",
                 diag->pose.imu_valid ? "OK" : "--",
                 diag->pose.encoder_valid ? "OK" : "--");
        break;
    default:
        snprintf(buffer, buffer_size, "");
        break;
    }
}

void auto_diag_render(const auto_diag_display_t *display,
                      const auto_diag_t *diag)
{
    char line[32];

    if (display == 0 || display->draw_text == 0) {
        return;
    }

    if (display->clear != 0) {
        display->clear(display->ctx);
    }

    for (unsigned i = 0u; i < 6u; ++i) {
        auto_diag_format_line(diag, i, line, sizeof(line));
        display->draw_text(display->ctx, i, 0u, line);
    }
}
