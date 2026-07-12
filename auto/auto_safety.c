#include "auto_safety.h"

#include <string.h>

#include "auto_config.h"

void auto_safety_init(auto_safety_t *safety)
{
    if (safety == 0) {
        return;
    }
    memset(safety, 0, sizeof(*safety));
    safety->fault = AUTO_OK;
}

void auto_safety_start(auto_safety_t *safety, uint32_t now_ms)
{
    if (safety == 0) {
        return;
    }
    safety->start_ms = now_ms;
    safety->fault = AUTO_OK;
    safety->active = true;
}

auto_status_t auto_safety_check(auto_safety_t *safety,
                                const auto_platform_t *port,
                                const auto_pose_t *pose,
                                uint32_t now_ms)
{
    if (safety == 0 || pose == 0) {
        return AUTO_ERR_INVALID_ARG;
    }

    if (port != 0 && port->is_emergency_stop != 0 &&
        port->is_emergency_stop(port->ctx)) {
        safety->fault = AUTO_ERR_ESTOP;
        return safety->fault;
    }

    if (!safety->active) {
        return AUTO_OK;
    }

#if AUTO_ROUTE_TIMEOUT_MS > 0u
    if ((now_ms - safety->start_ms) > AUTO_ROUTE_TIMEOUT_MS) {
        safety->fault = AUTO_ERR_ROUTE_TIMEOUT;
        return safety->fault;
    }
#endif

#if AUTO_REQUIRE_IMU
    if (!pose->imu_valid || (now_ms - pose->last_imu_ms) > AUTO_IMU_TIMEOUT_MS) {
        safety->fault = AUTO_ERR_SENSOR_TIMEOUT;
        return safety->fault;
    }
#endif

#if AUTO_REQUIRE_GPS
    if (!pose->gps_valid || (now_ms - pose->last_gps_ms) > AUTO_GPS_TIMEOUT_MS) {
        safety->fault = AUTO_ERR_SENSOR_TIMEOUT;
        return safety->fault;
    }
#endif

#if AUTO_REQUIRE_ENCODER
    if (!pose->encoder_valid ||
        (now_ms - pose->last_encoder_ms) > AUTO_ENCODER_TIMEOUT_MS) {
        safety->fault = AUTO_ERR_SENSOR_TIMEOUT;
        return safety->fault;
    }
#endif

    safety->fault = AUTO_OK;
    return safety->fault;
}

auto_status_t auto_safety_fault(const auto_safety_t *safety)
{
    return safety == 0 ? AUTO_ERR_INVALID_ARG : safety->fault;
}
