#include "auto_seekfree_runtime.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "auto_config.h"
#include "auto_math.h"
#include "auto_route.h"
#include "auto_route_recorder.h"
#include "auto_screen_seekfree.h"
#include "auto_seekfree_port.h"
#include "PID.h"
#include "drive.h"
#include "motor.h"
#include "zf_device_lora3a22.h"
#include "zf_common_headfile.h"

static auto_app_t s_auto_app;
static auto_platform_t s_auto_platform;
static auto_route_recorder_t s_route_recorder;
static bool s_runtime_initialized = false;
static uint32_t s_last_recorder_render_ms = 0u;
#if AUTO_ENABLE_UART_TELEMETRY
static uint32_t s_last_telemetry_ms = 0u;
#endif
static bool s_show_end_diag = false;

/* 自动采点：上次采点的位置和编码器里程（用于方向判断） */
static float s_last_capture_x_cm    = 0.0f;
static float s_last_capture_y_cm    = 0.0f;
static float s_last_capture_dist_cm = 0.0f;
static float s_last_capture_heading_deg = 0.0f;
static bool  s_capture_initialized  = false;
static float s_subject3_record_max_segment_cm = 0.0f;
static uint32_t s_subject3_record_bad_segment_index = 0u;

/* 转换后的路径点缓存（录点结束和自动启动时更新，地图显示用） */
static auto_waypoint_t s_converted_waypoints[AUTO_ROUTE_RECORDER_MAX_POINTS];
static float s_converted_headings_deg[AUTO_ROUTE_RECORDER_MAX_POINTS];
static float s_converted_steers_percent[AUTO_ROUTE_RECORDER_MAX_POINTS];
static size_t s_converted_count = 0u;
static size_t s_converted_heading_count = 0u;
static size_t s_converted_steer_count = 0u;
static float s_subject3_return_frame_heading_deg = 0.0f;
static float s_subject3_finish_heading_deg = 0.0f;
static auto_waypoint_t s_subject3_actual_trace[AUTO_ROUTE_RECORDER_MAX_POINTS];
static size_t s_subject3_actual_trace_count = 0u;
static float s_subject3_trace_last_x_cm = 0.0f;
static float s_subject3_trace_last_y_cm = 0.0f;

#define AUTO_SUBJECT3_DRIVE_DEAD_ZONE       (300)
#define AUTO_SUBJECT3_REMOTE_DRIVE_PWM_MAX  (2700)
#define AUTO_SUBJECT3_DRIVE_PWM_MIN         (700)
#define AUTO_SUBJECT3_DRIVE_PWM_STEP        (400)
#define AUTO_SUBJECT3_DRIVE_SPEED_MAX_CM_S  (220.0f)
#define AUTO_SUBJECT3_DRIVE_KP              (8.0f)
#define AUTO_SUBJECT3_DRIVE_KI              (0.4f)
#define AUTO_SUBJECT3_DRIVE_I_LIMIT         (2500.0f)
#define AUTO_SUBJECT3_ABS_COUNT_THRESHOLD   (20.0f)
#define AUTO_SUBJECT3_ABS_HOLD_TICKS        (5u)
#define AUTO_SUBJECT3_ABS_RELEASE_STEP      (800)
#define AUTO_SUBJECT3_STEER_DEAD_ZONE       (250)
#define AUTO_SUBJECT3_STEER_PERCENT_STEP    (100)
#define AUTO_SUBJECT3_STEER_PWM_MIN         (3000)
#define AUTO_SUBJECT3_STEER_TEST_DELTA_ADC  (260)
#define AUTO_SUBJECT3_STEER_TEST_PWM_LIMIT  (1800)
#define AUTO_SUBJECT3_STEER_TEST_MS         (1200u)
#define AUTO_SUBJECT3_CURVE_SLOW_DEG        (18.0f)
#define AUTO_SUBJECT3_CURVE_SPEED_PERCENT   (25.0f)
#define AUTO_SUBJECT3_STOP_SLOW_POINTS      (5u)
#define AUTO_SUBJECT3_STOP_SPEED_PERCENT    (25.0f)
#define AUTO_SUBJECT3_TRACE_INTERVAL_CM     (AUTO_RECORD_INTERVAL_CM)
#define AUTO_SUBJECT3_DEG_TO_RAD            (0.017453292519943295f)

#define AUTO_SUBJECT3_STOP_REASON_NONE      (0u)
#define AUTO_SUBJECT3_STOP_REASON_DONE      (1u)
#define AUTO_SUBJECT3_STOP_REASON_CTE       (2u)
#define AUTO_SUBJECT3_STOP_REASON_HEADING   (3u)
#define AUTO_SUBJECT3_STOP_REASON_FAULT     (4u)
#define AUTO_SUBJECT3_STOP_REASON_STOP      (5u)

typedef enum {
    AUTO_MENU_REC = 0,
    AUTO_MENU_AUTO
} auto_menu_item_t;

typedef enum {
    AUTO_SUBJECT_SELECT = 0,
    AUTO_SUBJECT_1,
    AUTO_SUBJECT_2,
    AUTO_SUBJECT_3
} auto_subject_t;

typedef enum {
    AUTO_SUBJECT3_IDLE = 0,
    AUTO_SUBJECT3_RECORDING,
    AUTO_SUBJECT3_WAIT_TURN,
    AUTO_SUBJECT3_RETURNING,
    AUTO_SUBJECT3_DONE,
    AUTO_SUBJECT3_FAULT
} auto_subject3_mode_t;

static auto_menu_item_t s_menu_cursor = AUTO_MENU_REC;
static auto_subject_t s_active_subject = AUTO_SUBJECT_SELECT;
static auto_subject3_mode_t s_subject3_mode = AUTO_SUBJECT3_IDLE;
static auto_status_t s_subject3_status = AUTO_OK;
static int16_t s_subject3_drive_joy = 0;
static int16_t s_subject3_steer_joy = 0;
static int32_t s_subject3_drive_cmd = 0;
static int32_t s_subject3_steer_cmd = 0;
static int32_t s_subject3_steer_percent_cmd = 0;
static int32_t s_subject3_left_drive_cmd = 0;
static int32_t s_subject3_right_drive_cmd = 0;
static float s_subject3_drive_i = 0.0f;
static int8_t s_subject3_drive_dir = 0;
static uint8_t s_subject3_abs_hold_ticks = 0u;
static uint8_t s_subject3_stop_reason = AUTO_SUBJECT3_STOP_REASON_NONE;
static bool s_subject3_steer_test_active = false;
static uint32_t s_subject3_steer_test_start_ms = 0u;
static int16_t s_subject3_steer_test_target_adc = STEER_ADC_CENTER;

static int32_t auto_seekfree_runtime_map_joystick_deadzone(int16_t joy_val,
                                                  int16_t pos_max,
                                                  int16_t neg_max,
                                                  int32_t out_max,
                                                  int16_t dead_zone)
{
    int32_t result;

    if (joy_val > -dead_zone && joy_val < dead_zone) {
        return 0;
    }

    if (joy_val > 0) {
        result = (int32_t)joy_val * out_max / (int32_t)pos_max;
        if (result > out_max) {
            result = out_max;
        }
    } else {
        result = (int32_t)joy_val * out_max / (int32_t)neg_max;
        if (result < -out_max) {
            result = -out_max;
        }
    }

    return result;
}

static int32_t auto_seekfree_runtime_map_joystick_expo4(int16_t joy_val,
                                                        int16_t pos_max,
                                                        int16_t neg_max,
                                                        int32_t out_max,
                                                        int16_t dead_zone)
{
    int32_t abs_joy;
    int32_t span;
    int32_t effective;
    int64_t scaled;

    if (joy_val > -dead_zone && joy_val < dead_zone) {
        return 0;
    }

    abs_joy = joy_val >= 0 ? (int32_t)joy_val : -(int32_t)joy_val;
    span = ((joy_val >= 0) ? (int32_t)pos_max : (int32_t)neg_max) -
           (int32_t)dead_zone;
    if (span <= 0) {
        return 0;
    }

    effective = abs_joy - (int32_t)dead_zone;
    if (effective < 0) {
        effective = 0;
    } else if (effective > span) {
        effective = span;
    }

    scaled = (int64_t)effective * (int64_t)effective *
             (int64_t)effective * (int64_t)effective *
             (int64_t)out_max;
    scaled /= (int64_t)span * (int64_t)span *
              (int64_t)span * (int64_t)span;
    if (scaled > out_max) {
        scaled = out_max;
    }

    return joy_val >= 0 ? (int32_t)scaled : -(int32_t)scaled;
}

static int32_t auto_seekfree_runtime_slew_i32(int32_t current,
                                              int32_t target,
                                              int32_t max_step)
{
    if ((target - current) > max_step) {
        return current + max_step;
    }
    if ((current - target) > max_step) {
        return current - max_step;
    }
    return target;
}

static int32_t auto_seekfree_runtime_clamp_i32(int32_t value,
                                               int32_t min_value,
                                               int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float auto_seekfree_runtime_subject3_recorded_return_heading(void)
{
    uint32_t count = s_route_recorder.point_count;

    if (count > AUTO_ROUTE_RECORDER_MAX_POINTS) {
        count = AUTO_ROUTE_RECORDER_MAX_POINTS;
    }
    if (count == 0u) {
        return 0.0f;
    }

    return s_subject3_finish_heading_deg;
}

static float auto_seekfree_runtime_steer_angle_to_percent(float steer_angle_deg)
{
    float half_range_deg = AUTO_SEEKFREE_STEER_TOTAL_DEG * 0.5f;

    if (half_range_deg < 1.0f) {
        half_range_deg = 1.0f;
    }

    return auto_clampf(steer_angle_deg * 100.0f / half_range_deg,
                       -100.0f,
                       100.0f);
}

static const char *auto_seekfree_runtime_subject3_stop_reason_text(void)
{
    switch (s_subject3_stop_reason) {
    case AUTO_SUBJECT3_STOP_REASON_DONE:
        return "DONE";
    case AUTO_SUBJECT3_STOP_REASON_CTE:
        return "CTE";
    case AUTO_SUBJECT3_STOP_REASON_HEADING:
        return "HEAD";
    case AUTO_SUBJECT3_STOP_REASON_FAULT:
        return "FAULT";
    case AUTO_SUBJECT3_STOP_REASON_STOP:
        return "STOP";
    default:
        return "--";
    }
}

static void auto_seekfree_runtime_subject3_update_stop_reason(
    const auto_diag_t *diag)
{
    float cte_abort_limit;

    if (diag == 0) {
        s_subject3_stop_reason = AUTO_SUBJECT3_STOP_REASON_STOP;
        return;
    }

    cte_abort_limit = AUTO_DYNAMIC_ROUTE_ABORT_CTE_CM;
    if (auto_absf(diag->heading_error_deg) >=
        AUTO_DYNAMIC_TURN_ABORT_START_DEG) {
        cte_abort_limit = AUTO_DYNAMIC_TURN_ABORT_CTE_CM;
    }

    if (diag->fault != AUTO_OK) {
        s_subject3_stop_reason = AUTO_SUBJECT3_STOP_REASON_FAULT;
    } else if (diag->cmd.brake) {
        if ((s_converted_count > 0u &&
             (diag->nearest_index + 1u >= s_converted_count ||
              diag->target_index + 1u >= s_converted_count) &&
             auto_absf(diag->cross_track_error_cm) >
             AUTO_DYNAMIC_ROUTE_DONE_CTE_CM) ||
            auto_absf(diag->cross_track_error_cm) >
            cte_abort_limit) {
            s_subject3_stop_reason = AUTO_SUBJECT3_STOP_REASON_CTE;
        } else if (auto_absf(diag->heading_error_deg) >=
                   AUTO_NAV_MAX_HEADING_ERROR_DEG &&
                   auto_absf(diag->cross_track_error_cm) >
                   AUTO_DYNAMIC_ROUTE_ABORT_HEADING_MIN_CTE_CM) {
            s_subject3_stop_reason = AUTO_SUBJECT3_STOP_REASON_HEADING;
        } else {
            s_subject3_stop_reason = AUTO_SUBJECT3_STOP_REASON_STOP;
        }
    } else if (s_converted_count > 0u &&
               (diag->nearest_index + 1u >= s_converted_count ||
                diag->target_index + 1u >= s_converted_count)) {
        s_subject3_stop_reason = AUTO_SUBJECT3_STOP_REASON_DONE;
    } else {
        s_subject3_stop_reason = AUTO_SUBJECT3_STOP_REASON_STOP;
    }
}

static void auto_seekfree_runtime_subject3_trace_reset(void)
{
    s_subject3_actual_trace_count = 0u;
    s_subject3_trace_last_x_cm = 0.0f;
    s_subject3_trace_last_y_cm = 0.0f;
}

static void auto_seekfree_runtime_subject3_trace_capture(const auto_pose_t *pose,
                                                         float min_delta_cm)
{
    auto_waypoint_t *point;

    if (pose == 0 ||
        s_subject3_actual_trace_count >= AUTO_ROUTE_RECORDER_MAX_POINTS) {
        return;
    }

    if (s_subject3_actual_trace_count > 0u &&
        auto_distance_cm(s_subject3_trace_last_x_cm,
                         s_subject3_trace_last_y_cm,
                         pose->x_cm,
                         pose->y_cm) < min_delta_cm) {
        return;
    }

    point = &s_subject3_actual_trace[s_subject3_actual_trace_count++];
    point->x_cm = pose->x_cm;
    point->y_cm = pose->y_cm;
    point->speed_percent = 0.0f;
    point->flags = AUTO_WAYPOINT_FLAG_NONE;
    s_subject3_trace_last_x_cm = pose->x_cm;
    s_subject3_trace_last_y_cm = pose->y_cm;
}

static void auto_seekfree_runtime_subject3_update_record_quality(void)
{
    uint32_t i;
    uint32_t count = s_route_recorder.point_count;

    s_subject3_record_max_segment_cm = 0.0f;
    s_subject3_record_bad_segment_index = 0u;

    if (count > AUTO_ROUTE_RECORDER_MAX_POINTS) {
        count = AUTO_ROUTE_RECORDER_MAX_POINTS;
    }

    for (i = 1u; i < count; ++i) {
        const auto_pose_t *prev = &s_route_recorder.points[i - 1u].pose;
        const auto_pose_t *curr = &s_route_recorder.points[i].pose;
        float segment_cm = auto_distance_cm(prev->x_cm,
                                            prev->y_cm,
                                            curr->x_cm,
                                            curr->y_cm);

        if (segment_cm > s_subject3_record_max_segment_cm) {
            s_subject3_record_max_segment_cm = segment_cm;
        }
        if (s_subject3_record_bad_segment_index == 0u &&
            segment_cm > AUTO_RECORD_MAX_SEGMENT_CM) {
            s_subject3_record_bad_segment_index = i;
        }
    }
}

static void auto_seekfree_runtime_subject3_reset_drive_loop(void)
{
    s_subject3_drive_cmd = 0;
    s_subject3_left_drive_cmd = 0;
    s_subject3_right_drive_cmd = 0;
    s_subject3_drive_i = 0.0f;
    s_subject3_drive_dir = 0;
    s_subject3_abs_hold_ticks = 0u;
}

static int32_t auto_seekfree_runtime_subject3_drive_pi(int32_t feedforward_pwm,
                                                       float target_count,
                                                       float actual_count,
                                                       float *integral)
{
    float error;
    float output;
    int32_t pwm;
    int32_t max_pwm = (int32_t)AUTO_SEEKFREE_DRIVE_PWM_MAX;

    if (integral == 0 || feedforward_pwm == 0) {
        return 0;
    }

    error = target_count - actual_count;
    *integral += error * AUTO_SUBJECT3_DRIVE_KI;
    *integral = auto_clampf(*integral,
                            -AUTO_SUBJECT3_DRIVE_I_LIMIT,
                            AUTO_SUBJECT3_DRIVE_I_LIMIT);

    output = (float)feedforward_pwm +
             error * AUTO_SUBJECT3_DRIVE_KP +
             *integral;
    output = auto_clampf(output, -(float)max_pwm, (float)max_pwm);
    pwm = (int32_t)output;

    if (feedforward_pwm > 0 && pwm < 0) {
        pwm = 0;
    } else if (feedforward_pwm < 0 && pwm > 0) {
        pwm = 0;
    }

    if (pwm > 0 && pwm < AUTO_SUBJECT3_DRIVE_PWM_MIN) {
        pwm = AUTO_SUBJECT3_DRIVE_PWM_MIN;
    } else if (pwm < 0 && pwm > -AUTO_SUBJECT3_DRIVE_PWM_MIN) {
        pwm = -AUTO_SUBJECT3_DRIVE_PWM_MIN;
    }

    return auto_seekfree_runtime_clamp_i32(pwm, -max_pwm, max_pwm);
}

static uint16_t auto_seekfree_runtime_read_steer_adc(void)
{
    return (uint16_t)Tern_Motor_Read_ADC();
}

static void auto_seekfree_runtime_subject3_center_steering_update(void)
{
    Set_Left_Pwm(0);
    Set_Right_Pwm(0);
    if (s_auto_app.port.set_steer_percent != 0) {
        s_auto_app.port.set_steer_percent(s_auto_app.port.ctx, 0.0f);
    } else {
        Steering_Set_Target_Angle(0.0f);
        Set_Steering_Pwm(Steering_PID_Calc());
    }
}

static void auto_seekfree_runtime_subject3_start_steer_test(int16_t target_adc)
{
    if (target_adc < STEER_ADC_LEFT_MAX) {
        target_adc = STEER_ADC_LEFT_MAX;
    } else if (target_adc > STEER_ADC_RIGHT_MAX) {
        target_adc = STEER_ADC_RIGHT_MAX;
    }

    s_subject3_steer_test_active = true;
    s_subject3_steer_test_start_ms = system_getval_ms();
    s_subject3_steer_test_target_adc = target_adc;
    Steering_Set_Target(target_adc);
}

static void auto_seekfree_runtime_subject3_steer_test_update(void)
{
    int16_t steer_pwm;
    uint32_t now_ms;

    if (!s_subject3_steer_test_active) {
        return;
    }

    Set_Left_Pwm(0);
    Set_Right_Pwm(0);
    now_ms = system_getval_ms();
    if ((now_ms - s_subject3_steer_test_start_ms) >
        AUTO_SUBJECT3_STEER_TEST_MS) {
        s_subject3_steer_test_active = false;
        Set_Steering_Pwm(0);
        Steering_Set_Target_Angle(0.0f);
        return;
    }

    Steering_Set_Target(s_subject3_steer_test_target_adc);
    steer_pwm = Steering_PID_Calc();
    if (steer_pwm > AUTO_SUBJECT3_STEER_TEST_PWM_LIMIT) {
        steer_pwm = AUTO_SUBJECT3_STEER_TEST_PWM_LIMIT;
    } else if (steer_pwm < -AUTO_SUBJECT3_STEER_TEST_PWM_LIMIT) {
        steer_pwm = -AUTO_SUBJECT3_STEER_TEST_PWM_LIMIT;
    }
    Set_Steering_Pwm(steer_pwm);
}

#if AUTO_ENABLE_UART_TELEMETRY
static const char *auto_seekfree_runtime_telemetry_mode(const auto_diag_t *diag)
{
    if (s_active_subject == AUTO_SUBJECT_SELECT) {
        return "MENU";
    }
    if (s_active_subject == AUTO_SUBJECT_3) {
        if (s_subject3_mode == AUTO_SUBJECT3_RECORDING) {
            return "REC";
        }
        if (s_subject3_mode == AUTO_SUBJECT3_WAIT_TURN) {
            return "WAIT";
        }
        if (s_subject3_mode == AUTO_SUBJECT3_RETURNING) {
            return "RUN";
        }
        if (s_subject3_mode == AUTO_SUBJECT3_DONE) {
            return "DONE";
        }
        if (s_subject3_mode == AUTO_SUBJECT3_FAULT) {
            return "FAULT";
        }
        return "IDLE";
    }
    if (auto_route_recorder_is_active(&s_route_recorder)) {
        return "REC";
    }
    if (diag != 0 && diag->running) {
        return "RUN";
    }
    return "IDLE";
}

static void auto_seekfree_runtime_emit_telemetry(uint32_t now_ms)
{
    auto_diag_t diag;
    const auto_pose_t *pose;
    const auto_pose_t zero_pose = {0};
    const char *mode;
    uint32_t point_index;
    uint32_t point_count;
    int16_t left_count = 0;
    int16_t right_count = 0;
    int left_display_count;
    int right_display_count;

    if ((now_ms - s_last_telemetry_ms) < 100u) {
        return;
    }
    s_last_telemetry_ms = now_ms;

    (void)Auto_GetDiag(&s_auto_app, &diag);
    pose = Auto_GetPose(&s_auto_app);
    if (pose == 0) {
        pose = &zero_pose;
    }

    mode = auto_seekfree_runtime_telemetry_mode(&diag);
    point_index = diag.target_index;
    point_count = (uint32_t)s_converted_count;
    if (auto_route_recorder_is_active(&s_route_recorder) ||
        (s_active_subject == AUTO_SUBJECT_3 &&
         s_subject3_mode == AUTO_SUBJECT3_RECORDING)) {
        point_index = s_route_recorder.point_count;
        point_count = AUTO_ROUTE_RECORDER_MAX_POINTS;
    }

    auto_seekfree_encoder_get_debug_counts(&left_count, &right_count);
    left_display_count = (int)((float)left_count * AUTO_SEEKFREE_LEFT_ENCODER_SIGN);
    right_display_count = (int)((float)right_count * AUTO_SEEKFREE_RIGHT_ENCODER_SIGN);

    printf("TEL t=%lu mode=%s x=%.1f y=%.1f h=%.1f d=%.1f "
           "v=%.1f st=%.1f p=%lu n=%lu e=%.1f nh=%.1f adc=%u ta=%d "
           "se=%d so=%d l=%d r=%d "
           "wh=%.1f sa=%.1f th=%.1f hc=%.2f co=%u\r\n",
           (unsigned long)now_ms,
           mode,
           pose->x_cm,
           pose->y_cm,
           pose->heading_deg,
           pose->distance_cm,
           diag.cmd.speed_percent,
           diag.cmd.steer_percent,
           (unsigned long)point_index,
           (unsigned long)point_count,
           diag.heading_error_deg,
           diag.nav_target_heading_deg,
           (unsigned)auto_seekfree_runtime_read_steer_adc(),
           (int)s_subject3_steer_test_target_adc,
           (int)Steering_Get_Error(),
           (int)Steering_Get_Output(),
           left_display_count,
           right_display_count,
           diag.wheel_heading_delta_deg,
           diag.steer_angle_deg,
           diag.steer_heading_delta_deg,
           diag.heading_correction_deg,
            (unsigned)diag.heading_consistency);
}
#endif

static void auto_seekfree_runtime_subject3_drive_closed_loop(int32_t drive_pwm)
{
    int16_t left_count = 0;
    int16_t right_count = 0;
    float target_count;
    float period_s;
    float left_actual_count;
    float right_actual_count;
    float avg_actual_count;
    int32_t drive_output_pwm;
    int8_t drive_dir;
    int8_t wheel_dir = 0;

    auto_seekfree_encoder_get_debug_counts(&left_count, &right_count);
    left_actual_count = (float)left_count * AUTO_SEEKFREE_LEFT_ENCODER_SIGN;
    right_actual_count = (float)right_count * AUTO_SEEKFREE_RIGHT_ENCODER_SIGN;
    avg_actual_count = (left_actual_count + right_actual_count) * 0.5f;
    if (avg_actual_count > AUTO_SUBJECT3_ABS_COUNT_THRESHOLD) {
        wheel_dir = 1;
    } else if (avg_actual_count < -AUTO_SUBJECT3_ABS_COUNT_THRESHOLD) {
        wheel_dir = -1;
    }

    if (drive_pwm == 0) {
        auto_seekfree_runtime_subject3_reset_drive_loop();
        Set_Left_Pwm(0);
        Set_Right_Pwm(0);
        return;
    }

    drive_dir = drive_pwm > 0 ? 1 : -1;
    if (wheel_dir != 0 && drive_dir != wheel_dir) {
        s_subject3_abs_hold_ticks = AUTO_SUBJECT3_ABS_HOLD_TICKS;
    }

    if (s_subject3_abs_hold_ticks > 0u) {
        s_subject3_abs_hold_ticks--;
        s_subject3_drive_i = 0.0f;
        s_subject3_drive_dir = 0;
        s_subject3_drive_cmd =
            auto_seekfree_runtime_slew_i32(s_subject3_drive_cmd,
                                           0,
                                           AUTO_SUBJECT3_ABS_RELEASE_STEP);
        s_subject3_left_drive_cmd = s_subject3_drive_cmd;
        s_subject3_right_drive_cmd = s_subject3_drive_cmd;
        Set_Left_Pwm((int16)-s_subject3_left_drive_cmd);
        Set_Right_Pwm((int16)-s_subject3_right_drive_cmd);
        return;
    }

    if (drive_dir != s_subject3_drive_dir) {
        s_subject3_drive_i = 0.0f;
        s_subject3_left_drive_cmd = 0;
        s_subject3_right_drive_cmd = 0;
        s_subject3_drive_cmd = 0;
        s_subject3_drive_dir = drive_dir;
    }

    period_s = (float)AUTO_CONTROL_PERIOD_MS / 1000.0f;
    target_count = ((float)drive_pwm / AUTO_SEEKFREE_DRIVE_PWM_MAX) *
                   AUTO_SUBJECT3_DRIVE_SPEED_MAX_CM_S *
                   period_s /
                   AUTO_SEEKFREE_ENCODER_CM_PER_COUNT;

    drive_output_pwm = auto_seekfree_runtime_subject3_drive_pi(
                           drive_pwm,
                           target_count,
                           avg_actual_count,
                           &s_subject3_drive_i);

    s_subject3_drive_cmd =
        auto_seekfree_runtime_slew_i32(s_subject3_drive_cmd,
                                       drive_output_pwm,
                                       AUTO_SUBJECT3_DRIVE_PWM_STEP);
    s_subject3_left_drive_cmd = s_subject3_drive_cmd;
    s_subject3_right_drive_cmd = s_subject3_drive_cmd;

    Set_Left_Pwm((int16)-s_subject3_left_drive_cmd);
    Set_Right_Pwm((int16)-s_subject3_right_drive_cmd);
}

static bool auto_seekfree_runtime_subject3_remote_online(void)
{
    return lora3a22_state_flag == 1 &&
           lora3a22_response_time <= LORA_TIMEOUT_TICKS;
}

static void auto_seekfree_runtime_subject3_update_lora_timeout(void)
{
    if (lora3a22_response_time <= LORA_TIMEOUT_TICKS + 10u) {
        lora3a22_response_time++;
    }
    if (lora3a22_response_time > LORA_TIMEOUT_TICKS) {
        lora3a22_state_flag = 0;
    }
}

static void auto_seekfree_runtime_stop_raw_outputs(void)
{
    auto_seekfree_runtime_subject3_reset_drive_loop();
    s_subject3_steer_cmd = 0;
    s_subject3_steer_percent_cmd = 0;
    Steering_Set_Target_Angle(0.0f);
    Set_Left_Pwm(0);
    Set_Right_Pwm(0);
    Set_Steering_Pwm(0);
}

static void auto_seekfree_runtime_select_subject(auto_subject_t subject)
{
    s_active_subject = subject;
    s_last_recorder_render_ms = 0u;
    if (subject == AUTO_SUBJECT_3) {
        s_subject3_mode = AUTO_SUBJECT3_IDLE;
        s_subject3_status = AUTO_OK;
    }
    auto_screen_seekfree_map_reset();
}

static void auto_seekfree_runtime_subject3_stop_outputs(void)
{
    (void)Auto_Stop(&s_auto_app);
    auto_seekfree_runtime_stop_raw_outputs();
}

static auto_status_t auto_seekfree_runtime_subject3_update_pose_only(void)
{
    uint32_t now_ms;
    auto_status_t status;

    now_ms = s_auto_app.port.now_ms != 0 ?
             s_auto_app.port.now_ms(s_auto_app.port.ctx) :
             system_getval_ms();
    status = auto_pose_update(&s_auto_app.pose_estimator,
                              &s_auto_app.port,
                              now_ms);
    s_auto_app.last_cmd.speed_percent = 0.0f;
    s_auto_app.last_cmd.steer_percent = 0.0f;
    s_auto_app.last_cmd.brake = true;
    s_auto_app.running = false;

    return status;
}

static void auto_seekfree_runtime_subject3_remote_update(void)
{
    int32_t drive_pwm;
    int32_t steer_percent;
    int16_t steer_pwm;

    auto_seekfree_runtime_subject3_update_lora_timeout();

    s_subject3_drive_joy = lora3a22_uart_transfer.joystick[1];
    s_subject3_steer_joy = lora3a22_uart_transfer.joystick[2];

    if (!auto_seekfree_runtime_subject3_remote_online()) {
        auto_seekfree_runtime_stop_raw_outputs();
        return;
    }

    drive_pwm = auto_seekfree_runtime_map_joystick_expo4(
                    s_subject3_drive_joy,
                    JOY_DRIVE_POS_MAX,
                    JOY_DRIVE_NEG_MAX,
                    AUTO_SUBJECT3_REMOTE_DRIVE_PWM_MAX,
                    AUTO_SUBJECT3_DRIVE_DEAD_ZONE);
    auto_seekfree_runtime_subject3_drive_closed_loop(drive_pwm);

    steer_percent = auto_seekfree_runtime_map_joystick_deadzone(
                        s_subject3_steer_joy,
                        JOY_STEER_POS_MAX,
                        JOY_STEER_NEG_MAX,
                        100,
                        AUTO_SUBJECT3_STEER_DEAD_ZONE);
    s_subject3_steer_percent_cmd =
        auto_seekfree_runtime_slew_i32(s_subject3_steer_percent_cmd,
                                       steer_percent,
                                       AUTO_SUBJECT3_STEER_PERCENT_STEP);
    Steering_Set_Target_Angle((float)-s_subject3_steer_percent_cmd);
    steer_pwm = Steering_PID_Calc();
    if (steer_pwm > 0 && steer_pwm < AUTO_SUBJECT3_STEER_PWM_MIN) {
        steer_pwm = AUTO_SUBJECT3_STEER_PWM_MIN;
    } else if (steer_pwm < 0 && steer_pwm > -AUTO_SUBJECT3_STEER_PWM_MIN) {
        steer_pwm = -AUTO_SUBJECT3_STEER_PWM_MIN;
    }
    Set_Steering_Pwm(steer_pwm);
    s_subject3_steer_cmd = steer_pwm;
}

static void auto_seekfree_runtime_update_capture_reference(const auto_pose_t *pose)
{
    if (pose != 0) {
        s_last_capture_x_cm    = pose->x_cm;
        s_last_capture_y_cm    = pose->y_cm;
        s_last_capture_dist_cm = pose->distance_cm;
        s_last_capture_heading_deg = pose->heading_deg;
    } else {
        s_last_capture_x_cm    = 0.0f;
        s_last_capture_y_cm    = 0.0f;
        s_last_capture_dist_cm = 0.0f;
        s_last_capture_heading_deg = 0.0f;
    }
    s_capture_initialized = true;
}

static bool auto_seekfree_runtime_should_capture_pose(const auto_pose_t *pose,
                                                      float min_delta_cm)
{
    float dist_delta_cm;
    float heading_delta_deg;

    if (pose == 0) {
        return false;
    }

    if (!s_capture_initialized) {
        return true;
    }

    dist_delta_cm = auto_absf(pose->distance_cm - s_last_capture_dist_cm);
    if (dist_delta_cm >= min_delta_cm) {
        return true;
    }

    heading_delta_deg =
        auto_absf(auto_normalize_angle_deg(pose->heading_deg -
                                           s_last_capture_heading_deg));
    return dist_delta_cm >= AUTO_RECORD_TURN_MIN_DELTA_CM &&
           heading_delta_deg >= AUTO_RECORD_TURN_HEADING_DEG;
}

static void auto_seekfree_runtime_capture_current_pose(float min_delta_cm)
{
    const auto_pose_t *pose = Auto_GetPose(&s_auto_app);

    if (!auto_seekfree_runtime_should_capture_pose(pose, min_delta_cm)) {
        return;
    }

    auto_route_recorder_capture(&s_route_recorder,
                                pose,
                                AUTO_CRUISE_SPEED_PERCENT,
                                AUTO_WAYPOINT_FLAG_NONE);
    auto_seekfree_runtime_update_capture_reference(pose);
}

static void auto_seekfree_runtime_subject3_build_return_route(float frame_heading_deg)
{
    uint32_t i;
    uint32_t count = s_route_recorder.point_count;
    const auto_pose_t *end_pose;
    float heading_rad;
    float cos_h;
    float sin_h;

    if (count > AUTO_ROUTE_RECORDER_MAX_POINTS) {
        count = AUTO_ROUTE_RECORDER_MAX_POINTS;
    }

    if (count == 0u) {
        s_converted_count = 0u;
        s_converted_heading_count = 0u;
        s_converted_steer_count = 0u;
        return;
    }

    end_pose = &s_route_recorder.points[count - 1u].pose;
    s_converted_count = count;
    heading_rad = -frame_heading_deg * AUTO_SUBJECT3_DEG_TO_RAD;
    cos_h = cosf(heading_rad);
    sin_h = sinf(heading_rad);

    for (i = 0u; i < count; ++i) {
        uint32_t src_index = (count - 1u) - i;
        const auto_recorded_waypoint_t *src = &s_route_recorder.points[src_index];
        float dx = src->pose.x_cm - end_pose->x_cm;
        float dy = src->pose.y_cm - end_pose->y_cm;

        s_converted_waypoints[i].x_cm = dx * cos_h - dy * sin_h;
        s_converted_waypoints[i].y_cm = dx * sin_h + dy * cos_h;
        s_converted_waypoints[i].speed_percent =
            AUTO_SUBJECT3_RETURN_REVERSE_SPEED_PERCENT;
        s_converted_waypoints[i].flags =
            (uint8_t)(src->flags | AUTO_WAYPOINT_FLAG_REVERSE);
        s_converted_headings_deg[i] = auto_normalize_angle_deg(
            src->pose.heading_deg - frame_heading_deg);
        s_converted_steers_percent[i] =
            auto_seekfree_runtime_steer_angle_to_percent(
                src->pose.steer_angle_deg);
    }
    s_converted_heading_count = count;
    s_converted_steer_count = count;

}

static void auto_seekfree_runtime_subject3_begin_recording(void)
{
    const auto_pose_t *start_pose;

    auto_seekfree_runtime_subject3_stop_outputs();
    auto_route_recorder_clear(&s_route_recorder);
    auto_route_set_dynamic(0, 0u);
    s_converted_count = 0u;
    s_converted_heading_count = 0u;
    s_converted_steer_count = 0u;
    auto_seekfree_runtime_subject3_trace_reset();
    s_show_end_diag = false;
    s_subject3_status = AUTO_OK;
    s_subject3_stop_reason = AUTO_SUBJECT3_STOP_REASON_NONE;
    s_subject3_record_max_segment_cm = 0.0f;
    s_subject3_record_bad_segment_index = 0u;

    auto_seekfree_encoder_reset_distance();
    auto_seekfree_imu_set_heading(0.0f);
    Auto_ResetPose(&s_auto_app);
    s_auto_app.pose_estimator.pose.heading_deg = 0.0f;
    lora3a22_state_flag = 0;
    lora3a22_response_time = LORA_TIMEOUT_TICKS + 1u;
    s_subject3_drive_joy = 0;
    s_subject3_steer_joy = 0;

    auto_route_recorder_begin(&s_route_recorder);
    start_pose = Auto_GetPose(&s_auto_app);
    if (start_pose != 0) {
        auto_route_recorder_capture(&s_route_recorder,
                                    start_pose,
                                    AUTO_CRUISE_SPEED_PERCENT,
                                    AUTO_WAYPOINT_FLAG_NONE);
    }
    auto_seekfree_runtime_update_capture_reference(start_pose);

    s_subject3_mode = AUTO_SUBJECT3_RECORDING;
    s_last_recorder_render_ms = 0u;
}

static void auto_seekfree_runtime_subject3_finish_and_wait_start(void)
{
    uint32_t i;
    uint32_t count;

    auto_seekfree_runtime_capture_current_pose(1.0f);
    auto_route_recorder_end(&s_route_recorder);
    auto_seekfree_runtime_subject3_stop_outputs();
    auto_seekfree_runtime_subject3_update_record_quality();

    if (s_route_recorder.point_count < 2u) {
        s_subject3_status = AUTO_ERR_INVALID_STATE;
        s_subject3_mode = AUTO_SUBJECT3_FAULT;
        return;
    }

    s_subject3_finish_heading_deg =
        s_route_recorder.points[s_route_recorder.point_count - 1u].pose.heading_deg;
    s_subject3_return_frame_heading_deg =
        auto_seekfree_runtime_subject3_recorded_return_heading();
    count = s_route_recorder.point_count;
    if (count > AUTO_ROUTE_RECORDER_MAX_POINTS) {
        count = AUTO_ROUTE_RECORDER_MAX_POINTS;
    }
    s_converted_count = count;
    s_converted_heading_count = 0u;
    s_converted_steer_count = 0u;
    for (i = 0u; i < count; ++i) {
        s_converted_waypoints[i].x_cm = s_route_recorder.points[i].pose.x_cm;
        s_converted_waypoints[i].y_cm = s_route_recorder.points[i].pose.y_cm;
        s_converted_waypoints[i].speed_percent = AUTO_CRUISE_SPEED_PERCENT;
        s_converted_waypoints[i].flags = s_route_recorder.points[i].flags;
    }
    auto_route_set_dynamic(s_converted_waypoints, s_converted_count);
    auto_route_set_dynamic_headings(0, 0u);
    auto_route_set_dynamic_steers(0, 0u);
    auto_screen_seekfree_map_reset();
    s_subject3_status = AUTO_OK;
    s_subject3_mode = AUTO_SUBJECT3_WAIT_TURN;
    s_show_end_diag = false;
}

static void auto_seekfree_runtime_subject3_start_return(void)
{
    auto_status_t status;
    const auto_pose_t *pose;
    float current_heading_deg;
    float relative_heading_deg;

    if (s_converted_count < 2u) {
        s_subject3_status = AUTO_ERR_INVALID_STATE;
        s_subject3_mode = AUTO_SUBJECT3_FAULT;
        return;
    }
    if (s_subject3_record_bad_segment_index != 0u) {
        s_subject3_status = AUTO_ERR_INVALID_STATE;
        s_subject3_mode = AUTO_SUBJECT3_FAULT;
        return;
    }

    (void)auto_seekfree_runtime_subject3_update_pose_only();
    pose = Auto_GetPose(&s_auto_app);
    current_heading_deg = pose != 0 ?
                          pose->heading_deg :
                          auto_seekfree_imu_get_heading();
    relative_heading_deg =
        auto_normalize_angle_deg(current_heading_deg -
                                 s_subject3_return_frame_heading_deg);
    auto_seekfree_runtime_subject3_stop_outputs();
    auto_seekfree_runtime_subject3_build_return_route(
        s_subject3_return_frame_heading_deg);
    auto_route_set_dynamic(s_converted_waypoints, s_converted_count);
    auto_route_set_dynamic_headings(s_converted_headings_deg,
                                    s_converted_heading_count);
    auto_route_set_dynamic_steers(s_converted_steers_percent,
                                  s_converted_steer_count);
    auto_screen_seekfree_map_reset();
    auto_seekfree_runtime_subject3_trace_reset();
    s_subject3_stop_reason = AUTO_SUBJECT3_STOP_REASON_NONE;
    auto_seekfree_encoder_reset_distance();
    auto_seekfree_imu_set_heading(relative_heading_deg);
    s_auto_app.pose_estimator.pose.heading_deg = relative_heading_deg;

    status = Auto_Start(&s_auto_app);
    s_subject3_status = status;
    if (status == AUTO_OK) {
        s_auto_app.pose_estimator.pose.heading_deg = relative_heading_deg;
        auto_seekfree_runtime_subject3_trace_capture(
            Auto_GetPose(&s_auto_app),
            0.0f);
        s_subject3_mode = AUTO_SUBJECT3_RETURNING;
        s_show_end_diag = false;
    } else {
        s_subject3_mode = AUTO_SUBJECT3_FAULT;
    }
}

static void auto_seekfree_runtime_subject3_abort(void)
{
    auto_seekfree_runtime_subject3_stop_outputs();
    if (auto_route_recorder_is_active(&s_route_recorder)) {
        auto_route_recorder_end(&s_route_recorder);
    }
    auto_route_recorder_clear(&s_route_recorder);
    auto_route_set_dynamic(0, 0u);
    s_converted_count = 0u;
    s_converted_heading_count = 0u;
    s_converted_steer_count = 0u;
    auto_seekfree_runtime_subject3_trace_reset();
    s_subject3_stop_reason = AUTO_SUBJECT3_STOP_REASON_NONE;
    s_show_end_diag = false;
    s_subject3_status = AUTO_OK;
    s_subject3_mode = AUTO_SUBJECT3_IDLE;
    auto_screen_seekfree_map_reset();
}

static void auto_seekfree_runtime_handle_subject3_keys(void)
{
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
        key_clear_state(KEY_1);
        if (s_subject3_mode == AUTO_SUBJECT3_IDLE ||
            s_subject3_mode == AUTO_SUBJECT3_FAULT) {
            auto_seekfree_runtime_subject3_begin_recording();
        } else if (s_subject3_mode == AUTO_SUBJECT3_DONE) {
            auto_seekfree_runtime_subject3_begin_recording();
        } else if (s_subject3_mode == AUTO_SUBJECT3_RECORDING) {
            auto_seekfree_runtime_subject3_finish_and_wait_start();
        } else if (s_subject3_mode == AUTO_SUBJECT3_WAIT_TURN) {
            auto_seekfree_runtime_subject3_start_return();
        }
    }

    if (key_get_state(KEY_2) == KEY_SHORT_PRESS) {
        key_clear_state(KEY_2);
        if (s_subject3_mode == AUTO_SUBJECT3_RECORDING) {
            auto_seekfree_runtime_capture_current_pose(AUTO_RECORD_INTERVAL_CM * 0.5f);
        } else if (s_subject3_mode == AUTO_SUBJECT3_WAIT_TURN) {
            auto_seekfree_runtime_subject3_start_steer_test(
                (int16_t)(STEER_ADC_CENTER -
                          AUTO_SUBJECT3_STEER_TEST_DELTA_ADC));
        }
    }

    if (key_get_state(KEY_3) == KEY_SHORT_PRESS) {
        key_clear_state(KEY_3);
        if (s_subject3_mode == AUTO_SUBJECT3_RECORDING) {
            auto_route_recorder_undo(&s_route_recorder);
        } else if (s_subject3_mode == AUTO_SUBJECT3_WAIT_TURN) {
            auto_seekfree_runtime_subject3_start_steer_test(
                (int16_t)(STEER_ADC_CENTER +
                          AUTO_SUBJECT3_STEER_TEST_DELTA_ADC));
        }
    }

    if (key_get_state(KEY_4) == KEY_SHORT_PRESS) {
        key_clear_state(KEY_4);
        auto_seekfree_runtime_subject3_abort();
        auto_seekfree_runtime_select_subject(AUTO_SUBJECT_SELECT);
    }
}

static void auto_seekfree_runtime_handle_keys(void)
{
    const auto_pose_t *pose;

    if (s_active_subject == AUTO_SUBJECT_SELECT) {
        if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_1);
            auto_seekfree_runtime_select_subject(AUTO_SUBJECT_1);
        }
        if (key_get_state(KEY_2) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_2);
            auto_seekfree_runtime_select_subject(AUTO_SUBJECT_2);
        }
        if (key_get_state(KEY_3) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_3);
            auto_seekfree_runtime_select_subject(AUTO_SUBJECT_3);
        }
        if (key_get_state(KEY_4) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_4);
        }
        return;
    }

    if (s_active_subject == AUTO_SUBJECT_2) {
        if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_1);
        }
        if (key_get_state(KEY_2) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_2);
        }
        if (key_get_state(KEY_3) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_3);
        }
        if (key_get_state(KEY_4) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_4);
            auto_seekfree_runtime_select_subject(AUTO_SUBJECT_SELECT);
        }
        return;
    }

    if (s_active_subject == AUTO_SUBJECT_3) {
        auto_seekfree_runtime_handle_subject3_keys();
        return;
    }

    if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
        key_clear_state(KEY_1);
        if (auto_route_recorder_is_active(&s_route_recorder)) {
            auto_route_recorder_end(&s_route_recorder);
            s_menu_cursor = AUTO_MENU_AUTO;
            s_show_end_diag = false;

            /* 录点结束：立即转换路线点到地图显示缓存，
             * 保证地图显示的是本次录制的路线而非上次自动驾驶的旧路线 */
            s_converted_count = s_route_recorder.point_count;
            if (s_converted_count > AUTO_ROUTE_RECORDER_MAX_POINTS) {
                s_converted_count = AUTO_ROUTE_RECORDER_MAX_POINTS;
            }
            s_converted_heading_count = 0u;
            s_converted_steer_count = 0u;
            {
                uint32_t _i;
                for (_i = 0u; _i < s_converted_count; ++_i) {
                    s_converted_waypoints[_i].x_cm          = s_route_recorder.points[_i].pose.x_cm;
                    s_converted_waypoints[_i].y_cm          = s_route_recorder.points[_i].pose.y_cm;
                    s_converted_waypoints[_i].speed_percent = AUTO_CRUISE_SPEED_PERCENT;
                    s_converted_waypoints[_i].flags         = s_route_recorder.points[_i].flags;
                }
            }
            auto_screen_seekfree_map_reset();  /* 强制重绘地图 */
        } else if (s_route_recorder.point_count >= 2u ||
                   s_menu_cursor == AUTO_MENU_AUTO) {
            (void)auto_seekfree_runtime_start();
        } else {
            const auto_pose_t *start_pose;
            (void)Auto_Stop(&s_auto_app);
            auto_route_recorder_clear(&s_route_recorder);
            s_show_end_diag = false;
            auto_seekfree_encoder_reset_distance();
            auto_seekfree_imu_set_heading(0.0f);
            Auto_ResetPose(&s_auto_app);
            s_auto_app.pose_estimator.pose.heading_deg = 0.0f;
            auto_route_recorder_begin(&s_route_recorder);
            /* 初始化自动采点参考点 */
            start_pose = Auto_GetPose(&s_auto_app);
            if (start_pose != 0) {
                auto_route_recorder_capture(&s_route_recorder,
                                            start_pose,
                                            AUTO_CRUISE_SPEED_PERCENT,
                                            AUTO_WAYPOINT_FLAG_NONE);
                s_last_capture_x_cm    = start_pose->x_cm;
                s_last_capture_y_cm    = start_pose->y_cm;
                s_last_capture_dist_cm = start_pose->distance_cm;
            } else {
                s_last_capture_x_cm    = 0.0f;
                s_last_capture_y_cm    = 0.0f;
                s_last_capture_dist_cm = 0.0f;
            }
            s_capture_initialized = true;
        }
    }

    if (key_get_state(KEY_2) == KEY_SHORT_PRESS) {
        key_clear_state(KEY_2);
        if (auto_route_recorder_is_active(&s_route_recorder)) {
            pose = Auto_GetPose(&s_auto_app);
            if (pose != 0 &&
                auto_absf(pose->distance_cm - s_last_capture_dist_cm) >=
                (AUTO_RECORD_INTERVAL_CM * 0.5f)) {
                auto_route_recorder_capture(&s_route_recorder,
                                            pose,
                                            AUTO_CRUISE_SPEED_PERCENT,
                                            AUTO_WAYPOINT_FLAG_NONE);
                s_last_capture_x_cm    = pose->x_cm;
                s_last_capture_y_cm    = pose->y_cm;
                s_last_capture_dist_cm = pose->distance_cm;
                s_capture_initialized  = true;
            }
        } else {
            s_menu_cursor = AUTO_MENU_REC;
        }
    }

    if (key_get_state(KEY_3) == KEY_SHORT_PRESS) {
        key_clear_state(KEY_3);
        if (auto_route_recorder_is_active(&s_route_recorder)) {
            auto_route_recorder_undo(&s_route_recorder);
        } else {
            s_menu_cursor = AUTO_MENU_AUTO;
        }
    }

    if (key_get_state(KEY_4) == KEY_SHORT_PRESS) {
        key_clear_state(KEY_4);
        auto_route_recorder_clear(&s_route_recorder);
        auto_route_set_dynamic(0, 0u);
        s_converted_count = 0u;
        s_converted_heading_count = 0u;
        s_converted_steer_count = 0u;
        s_show_end_diag = false;
        auto_screen_seekfree_map_reset();
    }
}

static void auto_seekfree_runtime_render_subject_menu(void)
{
    char lines[6][32];
    uint16_t steer_adc = auto_seekfree_runtime_read_steer_adc();

    snprintf(lines[0], sizeof(lines[0]), "== KEMU XUANZE ==");
    snprintf(lines[1], sizeof(lines[1]), "[K1] KEMU 1");
    snprintf(lines[2], sizeof(lines[2]), "[K2] KEMU 2");
    snprintf(lines[3], sizeof(lines[3]), "[K3] KEMU 3");
    snprintf(lines[4], sizeof(lines[4]), "ADC:%u C:%d",
             (unsigned)steer_adc,
             STEER_ADC_CENTER);
    snprintf(lines[5], sizeof(lines[5]), "L:%d R:%d",
             STEER_ADC_LEFT_MAX,
             STEER_ADC_RIGHT_MAX);

    auto_screen_seekfree_render_lines(lines, 6u);
}

static void auto_seekfree_runtime_render_subject_hold(void)
{
    char lines[6][32];

    snprintf(lines[0], sizeof(lines[0]), "== KEMU %u ==",
             (unsigned)s_active_subject);
    snprintf(lines[1], sizeof(lines[1]), "WEI PEIZHI");
    snprintf(lines[2], sizeof(lines[2]), "K4 FANHUI");
    snprintf(lines[3], sizeof(lines[3]), "");
    snprintf(lines[4], sizeof(lines[4]), "");
    snprintf(lines[5], sizeof(lines[5]), "");

    auto_screen_seekfree_render_lines(lines, 6u);
}

static void auto_seekfree_runtime_render_subject3_menu(void)
{
    char lines[6][32];
    const auto_pose_t *pose;
    uint16_t steer_adc = auto_seekfree_runtime_read_steer_adc();

    snprintf(lines[0], sizeof(lines[0]), "== KEMU 3 ==");
    if (s_subject3_mode == AUTO_SUBJECT3_WAIT_TURN) {
        pose = Auto_GetPose(&s_auto_app);
        snprintf(lines[1], sizeof(lines[1]), "READY K1 BACK");
        snprintf(lines[2], sizeof(lines[2]), "NO TURN 180");
        snprintf(lines[3], sizeof(lines[3]), "T:%d A:%u O:%d",
                 (int)s_subject3_steer_test_target_adc,
                 (unsigned)auto_seekfree_runtime_read_steer_adc(),
                 (int)Steering_Get_Output());
        snprintf(lines[4], sizeof(lines[4]), "H:%.0f P:%lu",
                 pose ? pose->heading_deg : 0.0f,
                 (unsigned long)s_converted_count);
    } else if (s_subject3_mode == AUTO_SUBJECT3_DONE) {
        snprintf(lines[1], sizeof(lines[1]), "FANHUI WANCHENG");
        snprintf(lines[2], sizeof(lines[2]), "K1 CHONGXIN");
        snprintf(lines[3], sizeof(lines[3]), "K4 FANHUI");
        snprintf(lines[4], sizeof(lines[4]), "LORA:%u J1:%d",
                 (unsigned)lora3a22_state_flag,
                 (int)s_subject3_drive_joy);
    } else if (s_subject3_mode == AUTO_SUBJECT3_FAULT) {
        snprintf(lines[1], sizeof(lines[1]), "LUXIAN DIAN BUZU");
        snprintf(lines[2], sizeof(lines[2]), "ERR:%u",
                 (unsigned)s_subject3_status);
        snprintf(lines[3], sizeof(lines[3]), "K1 CHONGXIN");
        snprintf(lines[4], sizeof(lines[4]), "LORA:%u J1:%d",
                 (unsigned)lora3a22_state_flag,
                 (int)s_subject3_drive_joy);
    } else {
        snprintf(lines[1], sizeof(lines[1]), "YK LUXIAN LUZHI");
        snprintf(lines[2], sizeof(lines[2]), "K1 KAISHI LUZHI");
        snprintf(lines[3], sizeof(lines[3]), "K1 JIESHU FANHUI");
        snprintf(lines[4], sizeof(lines[4]), "LORA:%u J1:%d",
                 (unsigned)lora3a22_state_flag,
                 (int)s_subject3_drive_joy);
    }
    snprintf(lines[5], sizeof(lines[5]), "ADC:%u S:%ld",
             (unsigned)steer_adc,
             (long)s_subject3_steer_cmd);

    auto_screen_seekfree_render_lines(lines, 6u);
}

static void auto_seekfree_runtime_render_menu(const auto_diag_t *diag)
{
    char lines[6][32];
    const auto_pose_t *pose = (diag == 0) ? 0 : &diag->pose;
    bool has_points = (s_route_recorder.point_count > 0u);
    int16_t left_count = 0;
    int16_t right_count = 0;
    int left_display_count;
    int right_display_count;

    auto_seekfree_encoder_get_debug_counts(&left_count, &right_count);
    left_display_count = (int)((float)left_count * AUTO_SEEKFREE_LEFT_ENCODER_SIGN);
    right_display_count = (int)((float)right_count * AUTO_SEEKFREE_RIGHT_ENCODER_SIGN);
    /* 行1: 标题 + 点数 */
    snprintf(lines[0], sizeof(lines[0]), "== DIAN:%lu ==",
             (unsigned long)s_route_recorder.point_count);

    /* 行2-3: 菜单选项，ZIDONG无采点时警告 */
    snprintf(lines[1], sizeof(lines[1]), "%s CAIDIAN",
             s_menu_cursor == AUTO_MENU_REC ? ">" : " ");
    if (s_menu_cursor == AUTO_MENU_AUTO && !has_points) {
        snprintf(lines[2], sizeof(lines[2]), "> ZIDONG !MEIYOU DIAN");
    } else {
        snprintf(lines[2], sizeof(lines[2]), "%s ZIDONG %s",
                 s_menu_cursor == AUTO_MENU_AUTO ? ">" : " ",
                 has_points ? "OK" : "--");
    }

    /* 行4: 按键提示 */
    snprintf(lines[3], sizeof(lines[3]), "[K1]OK [K2]CAI [K3]ZI");

    /* 行5: 航向 */
    snprintf(lines[4], sizeof(lines[4]), "H:%.0f D:%.0f",
             pose ? pose->heading_deg : 0.0f,
             pose ? pose->distance_cm : 0.0f);

    /* 行6: 转向ADC实时值（直接读硬件）+ 航向IMU状态 */
    snprintf(lines[5], sizeof(lines[5]), "L:%d R:%d",
             left_display_count,
             right_display_count);

    auto_screen_seekfree_render_lines(lines, 6u);
}

auto_status_t auto_seekfree_runtime_init(void)
{
    auto_status_t status;

    if (s_runtime_initialized) {
        return AUTO_OK;
    }

    auto_seekfree_platform_init();
    auto_screen_seekfree_init();
    Button_Init();
    lora3a22_init();
    key_init(AUTO_CONTROL_PERIOD_MS);

    s_auto_platform = auto_seekfree_platform_create();
    status = Auto_Init(&s_auto_app, &s_auto_platform);
    if (status != AUTO_OK) {
        return status;
    }

    auto_route_recorder_init(&s_route_recorder);
    s_active_subject = AUTO_SUBJECT_SELECT;
    s_menu_cursor = AUTO_MENU_REC;
    s_runtime_initialized = true;
    return AUTO_OK;
}

auto_status_t auto_seekfree_runtime_start(void)
{
    auto_status_t status;
    uint32_t i;

    status = auto_seekfree_runtime_init();
    if (status != AUTO_OK) {
        return status;
    }

    /* Refuse to start without enough recorded waypoints. The first point is
     * the origin captured automatically, so at least two points are needed.
     */
    if (s_route_recorder.point_count < 2u) {
        printf("AUTO: need at least 2 waypoints, cannot start\r\n");
        return AUTO_ERR_INVALID_STATE;
    }

    /* Convert recorded waypoints and inject into route */
    s_converted_count = s_route_recorder.point_count;
    if (s_converted_count > AUTO_ROUTE_RECORDER_MAX_POINTS) {
        s_converted_count = AUTO_ROUTE_RECORDER_MAX_POINTS;
    }
    s_converted_heading_count = 0u;
    s_converted_steer_count = 0u;
    for (i = 0u; i < s_converted_count; ++i) {
        s_converted_waypoints[i].x_cm          = s_route_recorder.points[i].pose.x_cm;
        s_converted_waypoints[i].y_cm          = s_route_recorder.points[i].pose.y_cm;
        s_converted_waypoints[i].speed_percent = AUTO_CRUISE_SPEED_PERCENT;
        s_converted_waypoints[i].flags         = s_route_recorder.points[i].flags;
    }
    auto_route_set_dynamic(s_converted_waypoints, s_converted_count);
    auto_route_set_dynamic_headings(0, 0u);
    auto_route_set_dynamic_steers(0, 0u);

    /* 重置地图显示，确保新路线从头绘制 */
    auto_screen_seekfree_map_reset();

    auto_seekfree_encoder_reset_distance();
    auto_seekfree_imu_set_heading(0.0f);

    status = Auto_Start(&s_auto_app);
    if (status == AUTO_OK) {
        s_auto_app.pose_estimator.pose.heading_deg = 0.0f;
        s_show_end_diag = true;
    }
    return status;
}

auto_status_t auto_seekfree_runtime_stop(void)
{
    if (!s_runtime_initialized) {
        return AUTO_ERR_INVALID_STATE;
    }

    return Auto_Stop(&s_auto_app);
}

auto_status_t auto_seekfree_runtime_update10ms(void)
{
    auto_status_t status;

    if (!s_runtime_initialized) {
        return AUTO_ERR_INVALID_STATE;
    }

    key_scanner();

    if (s_active_subject == AUTO_SUBJECT_3 &&
        s_subject3_mode == AUTO_SUBJECT3_RECORDING) {
        status = auto_seekfree_runtime_subject3_update_pose_only();
        auto_seekfree_runtime_capture_current_pose(AUTO_RECORD_INTERVAL_CM);

        auto_seekfree_runtime_handle_keys();
        if (s_subject3_mode == AUTO_SUBJECT3_RECORDING) {
            auto_seekfree_runtime_subject3_remote_update();
        }
        return status;
    }

    if (auto_route_recorder_is_active(&s_route_recorder)) {
        status = Auto_UpdatePoseOnly(&s_auto_app);

        /* 自动采点：车移动超过 AUTO_RECORD_INTERVAL_CM 就自动记录一个点 */
        auto_seekfree_runtime_capture_current_pose(AUTO_RECORD_INTERVAL_CM);

        auto_seekfree_runtime_handle_keys();
        /* Recording-only mode updates pose and does not drive steering. */
        return status;
    }

    auto_seekfree_runtime_handle_keys();
    if (s_active_subject == AUTO_SUBJECT_3 &&
        s_subject3_mode == AUTO_SUBJECT3_WAIT_TURN) {
        status = auto_seekfree_runtime_subject3_update_pose_only();
        if (s_subject3_steer_test_active) {
            auto_seekfree_runtime_subject3_steer_test_update();
        } else {
            auto_seekfree_runtime_subject3_center_steering_update();
        }
        return status;
    }

    if (s_active_subject == AUTO_SUBJECT_3 &&
        s_subject3_mode == AUTO_SUBJECT3_RECORDING) {
        status = auto_seekfree_runtime_subject3_update_pose_only();
        auto_seekfree_runtime_subject3_remote_update();
        return status;
    }

    if (s_active_subject == AUTO_SUBJECT_SELECT ||
        s_active_subject == AUTO_SUBJECT_2 ||
        (s_active_subject == AUTO_SUBJECT_3 &&
         s_subject3_mode != AUTO_SUBJECT3_RETURNING)) {
        auto_seekfree_runtime_stop_raw_outputs();
        return AUTO_OK;
    }

    if (auto_route_recorder_is_active(&s_route_recorder)) {
        status = Auto_UpdatePoseOnly(&s_auto_app);
        /* Recording-only mode updates pose and does not drive steering. */
        return status;
    }

    if (!s_auto_app.running) {
        status = Auto_UpdatePoseOnly(&s_auto_app);
        return status;
    }

    status = Auto_Update10ms(&s_auto_app);
    if (s_active_subject == AUTO_SUBJECT_3 &&
        s_subject3_mode == AUTO_SUBJECT3_RETURNING) {
        auto_seekfree_runtime_subject3_trace_capture(
            Auto_GetPose(&s_auto_app),
            AUTO_SUBJECT3_TRACE_INTERVAL_CM);
        if (!s_auto_app.running) {
            auto_diag_t end_diag;

            Auto_GetDiag(&s_auto_app, &end_diag);
            auto_seekfree_runtime_subject3_update_stop_reason(&end_diag);
            auto_seekfree_runtime_subject3_trace_capture(
                Auto_GetPose(&s_auto_app),
                0.0f);
            s_subject3_mode = AUTO_SUBJECT3_DONE;
            s_show_end_diag = true;
            auto_seekfree_runtime_stop_raw_outputs();
        }
    }
    /* Automatic modes own their steering/drive outputs inside Auto_Update10ms. */
    return status;
}

void auto_seekfree_runtime_loop(void)
{
    auto_diag_t diag;
    const auto_pose_t *pose;
    char lines[6][32];
    uint32_t now_ms;
    bool recorder_dirty;
    unsigned i;
    int16_t left_count = 0;
    int16_t right_count = 0;
    int left_display_count;
    int right_display_count;

    if (!s_runtime_initialized) {
        return;
    }

    now_ms = system_getval_ms();
#if AUTO_ENABLE_UART_TELEMETRY
    auto_seekfree_runtime_emit_telemetry(now_ms);
#endif

    if (s_active_subject == AUTO_SUBJECT_SELECT) {
        auto_seekfree_runtime_render_subject_menu();
        return;
    }
    if (s_active_subject == AUTO_SUBJECT_2) {
        auto_seekfree_runtime_render_subject_hold();
        return;
    }
    if (s_active_subject == AUTO_SUBJECT_3 &&
        (s_subject3_mode == AUTO_SUBJECT3_IDLE ||
         (s_subject3_mode == AUTO_SUBJECT3_DONE &&
          s_subject3_actual_trace_count == 0u) ||
         s_subject3_mode == AUTO_SUBJECT3_FAULT)) {
        auto_seekfree_runtime_render_subject3_menu();
        return;
    }

    auto_seekfree_encoder_get_debug_counts(&left_count, &right_count);
    left_display_count = (int)((float)left_count * AUTO_SEEKFREE_LEFT_ENCODER_SIGN);
    right_display_count = (int)((float)right_count * AUTO_SEEKFREE_RIGHT_ENCODER_SIGN);
    recorder_dirty = auto_route_recorder_take_dirty(&s_route_recorder);
    if (auto_route_recorder_is_active(&s_route_recorder) || recorder_dirty) {
        if (recorder_dirty ||
            (now_ms - s_last_recorder_render_ms) >= AUTO_SCREEN_REFRESH_MS) {
            pose = Auto_GetPose(&s_auto_app);
            if (s_active_subject == AUTO_SUBJECT_3 &&
                s_subject3_mode == AUTO_SUBJECT3_RECORDING) {
                snprintf(lines[0], sizeof(lines[0]), "K3 REC DIAN:%lu",
                         (unsigned long)s_route_recorder.point_count);
                snprintf(lines[1], sizeof(lines[1]), "K1 JIESHU->FANHUI");
                snprintf(lines[2], sizeof(lines[2]), "J1:%d J2:%d",
                         (int)s_subject3_drive_joy,
                         (int)s_subject3_steer_joy);
                if (pose != 0) {
                    snprintf(lines[3], sizeof(lines[3]), "X:%.0f Y:%.0f",
                             pose->x_cm,
                             pose->y_cm);
                    snprintf(lines[4], sizeof(lines[4]), "H:%.0f D:%.0f",
                             pose->heading_deg,
                             pose->distance_cm);
                } else {
                    snprintf(lines[3], sizeof(lines[3]), "X:-- Y:--");
                    snprintf(lines[4], sizeof(lines[4]), "H:-- D:--");
                }
            } else {
                for (i = 0u; i < 6u; ++i) {
                    auto_route_recorder_format_line(&s_route_recorder,
                                                    pose,
                                                    i,
                                                    lines[i],
                                                    sizeof(lines[i]));
                }
            }
            snprintf(lines[5], sizeof(lines[5]), "L:%d R:%d",
                     left_display_count,
                     right_display_count);
            if (s_active_subject == AUTO_SUBJECT_3 &&
                s_subject3_mode == AUTO_SUBJECT3_RECORDING) {
                snprintf(lines[5], sizeof(lines[5]), "ADC:%u S:%ld",
                         (unsigned)auto_seekfree_runtime_read_steer_adc(),
                         (long)s_subject3_steer_cmd);
            }
            auto_screen_seekfree_render_lines(lines, 6u);
            s_last_recorder_render_ms = now_ms;
        }
        return;
    }

    Auto_GetDiag(&s_auto_app, &diag);

    if (diag.running) {
        if ((now_ms - s_last_recorder_render_ms) >= AUTO_SCREEN_REFRESH_MS) {
            if (s_converted_count > 0u) {
                auto_screen_seekfree_render_map(s_converted_waypoints,
                                                s_converted_count,
                                                diag.pose.x_cm,
                                                diag.pose.y_cm);
                if (s_active_subject == AUTO_SUBJECT_3 &&
                    s_subject3_actual_trace_count > 0u) {
                    auto_screen_seekfree_render_trace(
                        s_subject3_actual_trace,
                        s_subject3_actual_trace_count);
                }
                snprintf(lines[0], sizeof(lines[0]), "RUN P:%lu>%lu/%lu",
                         (unsigned long)diag.nearest_index,
                         (unsigned long)diag.target_index,
                         (unsigned long)s_converted_count);
                snprintf(lines[1], sizeof(lines[1]), "D%.0f A%u O%d",
                         diag.pose.distance_cm,
                         (unsigned)auto_seekfree_runtime_read_steer_adc(),
                         (int)Steering_Get_Output());
                snprintf(lines[2], sizeof(lines[2]), "CT%.0f E%.0f S%.0f",
                         diag.cross_track_error_cm,
                         diag.heading_error_deg,
                         diag.cmd.steer_percent);
                snprintf(lines[3], sizeof(lines[3]), "H%.0f N%.0f T%.2f",
                         diag.pose.heading_deg,
                         diag.nav_target_heading_deg,
                         diag.nav_segment_t);
                auto_screen_seekfree_debug_line(0u, lines[0]);
                auto_screen_seekfree_debug_line(1u, lines[1]);
                auto_screen_seekfree_debug_line(2u, lines[2]);
                auto_screen_seekfree_debug_line(3u, lines[3]);
            } else {
                snprintf(lines[0], sizeof(lines[0]), "RUN P:%lu/%lu",
                         (unsigned long)diag.target_index,
                         (unsigned long)s_converted_count);
                snprintf(lines[1], sizeof(lines[1]), "S:%u F:%u R:%u",
                         (unsigned)diag.state,
                         (unsigned)diag.fault,
                         diag.running ? 1u : 0u);
                snprintf(lines[2], sizeof(lines[2]), "H:%.0f D:%.0f",
                         diag.pose.heading_deg,
                         diag.pose.distance_cm);
                snprintf(lines[3], sizeof(lines[3]), "X:%.0f Y:%.0f",
                         diag.pose.x_cm,
                         diag.pose.y_cm);
                snprintf(lines[4], sizeof(lines[4]), "V:%.0f ST:%.0f",
                         diag.cmd.speed_percent,
                         diag.cmd.steer_percent);
                snprintf(lines[5], sizeof(lines[5]), "L:%d R:%d",
                         left_display_count,
                         right_display_count);
                auto_screen_seekfree_render_lines(lines, 6u);
            }
            s_last_recorder_render_ms = now_ms;
        }
        return;
    }

    /* 有路径点时始终显示地图（录点前/自动中/自动后都可见）
     * 自动驾驶时：红点跟随车辆位置移动
     * 非驾驶时：红点停在最后位置 */
    if (s_show_end_diag &&
        (diag.state == AUTO_STATE_STOP || diag.state == AUTO_STATE_FAULT) &&
        s_converted_count > 0u) {
        if ((now_ms - s_last_recorder_render_ms) >= AUTO_SCREEN_REFRESH_MS) {
            if (s_active_subject == AUTO_SUBJECT_3 &&
                s_subject3_actual_trace_count > 0u) {
                auto_screen_seekfree_render_map(s_converted_waypoints,
                                                s_converted_count,
                                                diag.pose.x_cm,
                                                diag.pose.y_cm);
                auto_screen_seekfree_render_trace(
                    s_subject3_actual_trace,
                    s_subject3_actual_trace_count);
                snprintf(lines[0], sizeof(lines[0]), "END %s %lu>%lu/%lu",
                         auto_seekfree_runtime_subject3_stop_reason_text(),
                         (unsigned long)diag.nearest_index,
                         (unsigned long)diag.target_index,
                         (unsigned long)s_converted_count);
                snprintf(lines[1], sizeof(lines[1]), "D%.0f A%u O%d",
                         diag.pose.distance_cm,
                         (unsigned)auto_seekfree_runtime_read_steer_adc(),
                         (int)Steering_Get_Output());
                snprintf(lines[2], sizeof(lines[2]), "CT%.0f E%.0f S%.0f",
                         diag.cross_track_error_cm,
                         diag.heading_error_deg,
                         diag.cmd.steer_percent);
                snprintf(lines[3], sizeof(lines[3]), "H%.0f N%.0f T%.2f",
                         diag.pose.heading_deg,
                         diag.nav_target_heading_deg,
                         diag.nav_segment_t);
                auto_screen_seekfree_debug_line(0u, lines[0]);
                auto_screen_seekfree_debug_line(1u, lines[1]);
                auto_screen_seekfree_debug_line(2u, lines[2]);
                auto_screen_seekfree_debug_line(3u, lines[3]);
            } else {
                snprintf(lines[0], sizeof(lines[0]), "END P:%lu/%lu",
                         (unsigned long)diag.target_index,
                         (unsigned long)s_converted_count);
                snprintf(lines[1], sizeof(lines[1]), "S:%u F:%u R:%u",
                         (unsigned)diag.state,
                         (unsigned)diag.fault,
                         diag.running ? 1u : 0u);
                snprintf(lines[2], sizeof(lines[2]), "H:%.0f D:%.0f",
                         diag.pose.heading_deg,
                         diag.pose.distance_cm);
                snprintf(lines[3], sizeof(lines[3]), "X:%.0f Y:%.0f",
                         diag.pose.x_cm,
                         diag.pose.y_cm);
                snprintf(lines[4], sizeof(lines[4]), "V:%.0f ST:%.0f",
                         diag.cmd.speed_percent,
                         diag.cmd.steer_percent);
                snprintf(lines[5], sizeof(lines[5]), "L:%d R:%d",
                         left_display_count,
                         right_display_count);
                auto_screen_seekfree_render_lines(lines, 6u);
            }
            s_last_recorder_render_ms = now_ms;
        }
        return;
    }

    if (s_converted_count > 0u) {
        const auto_pose_t *map_pose = Auto_GetPose(&s_auto_app);
        float cx = (map_pose != 0) ? map_pose->x_cm : 0.0f;
        float cy = (map_pose != 0) ? map_pose->y_cm : 0.0f;
        auto_screen_seekfree_render_map(s_converted_waypoints, s_converted_count,
                                        cx, cy);
        if (s_active_subject == AUTO_SUBJECT_3 &&
            s_subject3_actual_trace_count > 0u) {
            auto_screen_seekfree_render_trace(s_subject3_actual_trace,
                                              s_subject3_actual_trace_count);
        }
        if (s_active_subject == AUTO_SUBJECT_3 &&
            s_subject3_mode == AUTO_SUBJECT3_WAIT_TURN) {
            if (s_subject3_record_bad_segment_index != 0u) {
                snprintf(lines[0], sizeof(lines[0]), "BAD SEG:%lu",
                         (unsigned long)s_subject3_record_bad_segment_index);
            } else {
                snprintf(lines[0], sizeof(lines[0]), "READY K1 BACK");
            }
            snprintf(lines[1], sizeof(lines[1]), "T:%d A:%u O:%d",
                     (int)s_subject3_steer_test_target_adc,
                     (unsigned)auto_seekfree_runtime_read_steer_adc(),
                     (int)Steering_Get_Output());
            snprintf(lines[2], sizeof(lines[2]), "P:%lu G:%.0f",
                     (unsigned long)s_converted_count,
                     s_subject3_record_max_segment_cm);
            snprintf(lines[3], sizeof(lines[3]), "H:%.0f",
                     map_pose ? map_pose->heading_deg : 0.0f);
            auto_screen_seekfree_debug_line(0u, lines[0]);
            auto_screen_seekfree_debug_line(1u, lines[1]);
            auto_screen_seekfree_debug_line(2u, lines[2]);
            auto_screen_seekfree_debug_line(3u, lines[3]);
        }
        return;
    }

    /* 无路径点：显示普通菜单 */
    if (!diag.running) {
        auto_seekfree_runtime_render_menu(&diag);
        return;
    }

    auto_screen_seekfree_render_periodic(&diag, now_ms);
}

auto_app_t *auto_seekfree_runtime_app(void)
{
    return &s_auto_app;
}
