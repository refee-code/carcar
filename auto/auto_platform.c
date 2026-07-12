#include "auto_platform.h"

bool auto_platform_has_required_outputs(const auto_platform_t *port)
{
    return port != 0 &&
           port->now_ms != 0 &&
           port->set_drive_percent != 0 &&
           port->set_steer_percent != 0 &&
           port->set_brake != 0;
}
