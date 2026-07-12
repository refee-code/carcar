#ifndef AUTO_TYPES_H
#define AUTO_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    AUTO_OK = 0,
    AUTO_ERR_INVALID_ARG,
    AUTO_ERR_INVALID_STATE,
    AUTO_ERR_PLATFORM,
    AUTO_ERR_SENSOR_TIMEOUT,
    AUTO_ERR_ESTOP,
    AUTO_ERR_ROUTE_DONE,
    AUTO_ERR_ROUTE_TIMEOUT
} auto_status_t;

typedef enum {
    AUTO_STATE_IDLE = 0,
    AUTO_STATE_START,
    AUTO_STATE_SLALOM,
    AUTO_STATE_GARAGE_APPROACH,
    AUTO_STATE_ALIGN_GARAGE,
    AUTO_STATE_REVERSE_PARK,
    AUTO_STATE_STOP,
    AUTO_STATE_FAULT
} auto_subject1_state_t;

typedef struct {
    float x_cm;
    float y_cm;
    float heading_deg;
    float speed_cms;
    float distance_cm;
    uint32_t time_ms;
    uint32_t last_gps_ms;
    uint32_t last_imu_ms;
    uint32_t last_encoder_ms;
    float wheel_heading_delta_deg;
    float steer_angle_deg;
    float steer_heading_delta_deg;
    float heading_correction_deg;
    uint8_t heading_consistency;
    bool gps_valid;
    bool imu_valid;
    bool encoder_valid;
} auto_pose_t;

typedef struct {
    float speed_percent;
    float steer_percent;
    bool brake;
} auto_drive_cmd_t;

typedef struct {
    auto_subject1_state_t state;
    auto_status_t fault;
    auto_pose_t pose;
    auto_drive_cmd_t cmd;
    uint32_t nearest_index;
    uint32_t target_index;
    float target_distance_cm;
    float heading_error_deg;
    float cross_track_error_cm;
    float cross_track_steer_percent;
    float nav_target_x_cm;
    float nav_target_y_cm;
    float nav_target_heading_deg;
    float nav_segment_t;
    float wheel_heading_delta_deg;
    float steer_angle_deg;
    float steer_heading_delta_deg;
    float heading_correction_deg;
    uint8_t heading_consistency;
    uint32_t park_stage;
    bool running;
} auto_diag_t;

typedef struct {
    float x_cm;
    float y_cm;
    float speed_percent;
    uint8_t flags;
} auto_waypoint_t;

#define AUTO_WAYPOINT_FLAG_NONE       (0u)
#define AUTO_WAYPOINT_FLAG_SLOW       (1u << 0)
#define AUTO_WAYPOINT_FLAG_GARAGE     (1u << 1)
#define AUTO_WAYPOINT_FLAG_REVERSE    (1u << 2)  /* 采点时为倒车段，回放时使用倒车速度 */

#endif
