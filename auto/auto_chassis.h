#ifndef AUTO_CHASSIS_H
#define AUTO_CHASSIS_H

#include "auto_platform.h"
#include "auto_types.h"

void auto_chassis_apply(const auto_platform_t *port, const auto_drive_cmd_t *cmd);
void auto_chassis_stop(const auto_platform_t *port);

#endif
