#include "auto_chassis.h"

#include "auto_math.h"

void auto_chassis_apply(const auto_platform_t *port, const auto_drive_cmd_t *cmd)
{
    float speed_percent;
    float steer_percent;

    if (port == 0 || cmd == 0) {
        return;
    }

    speed_percent = auto_clampf(cmd->speed_percent, -100.0f, 100.0f);
    steer_percent = auto_clampf(cmd->steer_percent, -100.0f, 100.0f);

    if (cmd->brake) {
        port->set_drive_percent(port->ctx, 0.0f);
        port->set_steer_percent(port->ctx, 0.0f);
        port->set_brake(port->ctx, true);
        return;
    }

    port->set_brake(port->ctx, false);
    port->set_steer_percent(port->ctx, steer_percent);
    if (port->set_drive_steer_percent != 0) {
        port->set_drive_steer_percent(port->ctx, speed_percent, steer_percent);
    } else {
        port->set_drive_percent(port->ctx, speed_percent);
    }
}

void auto_chassis_stop(const auto_platform_t *port)
{
    auto_drive_cmd_t stop_cmd;

    if (port == 0) {
        return;
    }

    stop_cmd.speed_percent = 0.0f;
    stop_cmd.steer_percent = 0.0f;
    stop_cmd.brake = true;
    auto_chassis_apply(port, &stop_cmd);
}
