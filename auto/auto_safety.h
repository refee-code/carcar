#ifndef AUTO_SAFETY_H
#define AUTO_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#include "auto_platform.h"
#include "auto_types.h"

typedef struct {
    uint32_t start_ms;
    auto_status_t fault;
    bool active;
} auto_safety_t;

void auto_safety_init(auto_safety_t *safety);
void auto_safety_start(auto_safety_t *safety, uint32_t now_ms);
auto_status_t auto_safety_check(auto_safety_t *safety,
                                const auto_platform_t *port,
                                const auto_pose_t *pose,
                                uint32_t now_ms);
auto_status_t auto_safety_fault(const auto_safety_t *safety);

#endif
