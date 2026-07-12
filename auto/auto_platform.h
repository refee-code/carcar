#ifndef AUTO_PLATFORM_H
#define AUTO_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float distance_cm;
    float delta_center_cm;
    float delta_left_cm;
    float delta_right_cm;
    float wheel_heading_delta_deg;
    float steer_angle_deg;
    float steer_heading_delta_deg;
} auto_motion_sample_t;

typedef struct {
    void *ctx;

    uint32_t (*now_ms)(void *ctx);

    bool (*read_imu_heading_deg)(void *ctx, float *heading_deg);
    bool (*read_gps_lat_lon)(void *ctx, double *lat_deg, double *lon_deg);
    bool (*read_encoder_distance_cm)(void *ctx, float *distance_cm);
    bool (*read_motion_sample)(void *ctx, auto_motion_sample_t *sample);
    bool (*is_emergency_stop)(void *ctx);

    void (*set_drive_percent)(void *ctx, float speed_percent);
    void (*set_drive_steer_percent)(void *ctx,
                                    float speed_percent,
                                    float steer_percent);
    void (*set_steer_percent)(void *ctx, float steer_percent);
    void (*set_brake)(void *ctx, bool enable);
    void (*log_text)(void *ctx, const char *text);
} auto_platform_t;

bool auto_platform_has_required_outputs(const auto_platform_t *port);

#endif
