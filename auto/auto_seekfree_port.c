#include "auto_seekfree_port.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "auto_config.h"
#include "auto_math.h"
#include "zf_common_headfile.h"
#include "PID.h"

static float s_heading_deg = 0.0f;
static float s_last_gyro_time_ms = 0.0f;
static float s_encoder_total_cm = 0.0f;
static float s_gyro_z_bias_dps = 0.0f;
static int16_t s_encoder_left_count = 0;
static int16_t s_encoder_right_count = 0;

#define SEEKFREE_RAD_TO_DEG (57.29577951308232f)
#define SEEKFREE_DEG_TO_RAD (0.017453292519943295f)

static float normalize_heading_deg(float heading_deg)
{
    while (heading_deg > 180.0f) {
        heading_deg -= 360.0f;
    }
    while (heading_deg < -180.0f) {
        heading_deg += 360.0f;
    }
    return heading_deg;
}

void auto_seekfree_imu_set_heading(float heading_deg)
{
    s_heading_deg = heading_deg;
    s_last_gyro_time_ms = 0.0f;
}

float auto_seekfree_imu_get_heading(void)
{
    return s_heading_deg;
}

void auto_seekfree_encoder_reset_distance(void)
{
    s_encoder_total_cm = 0.0f;
    s_encoder_left_count = 0;
    s_encoder_right_count = 0;

#if AUTO_SEEKFREE_USE_ENCODER
    encoder_clear_count(AUTO_SEEKFREE_LEFT_ENCODER_INDEX);
    encoder_clear_count(AUTO_SEEKFREE_RIGHT_ENCODER_INDEX);
#endif
}

void auto_seekfree_encoder_get_debug_counts(int16_t *left_count, int16_t *right_count)
{
    if (left_count != 0) {
        *left_count = s_encoder_left_count;
    }
    if (right_count != 0) {
        *right_count = s_encoder_right_count;
    }
}

void auto_seekfree_imu_reset_heading_from_mag(void)
{
#if AUTO_SEEKFREE_USE_IMU963RA
    float mag_x_g, mag_y_g, mag_hdg;
    imu963ra_get_mag();
    mag_x_g = imu963ra_mag_transition(imu963ra_mag_x);
    mag_y_g = imu963ra_mag_transition(imu963ra_mag_y);
    mag_hdg = normalize_heading_deg(
        atan2f(mag_y_g, mag_x_g) * (180.0f / 3.14159265f));
    s_heading_deg       = mag_hdg;
    s_last_gyro_time_ms = 0.0f;
#endif
}

static uint32_t seekfree_now_ms(void *ctx)
{
    (void)ctx;
    return system_getval_ms();
}

static bool seekfree_read_gyro_z_dps(float *gyro_z_dps)
{
    if (gyro_z_dps == 0) {
        return false;
    }

#if AUTO_SEEKFREE_USE_IMU660RA
    imu660ra_get_gyro();
    *gyro_z_dps = imu660ra_gyro_transition(imu660ra_gyro_z);
#elif AUTO_SEEKFREE_USE_IMU660RB
    imu660rb_get_gyro();
    *gyro_z_dps = imu660rb_gyro_transition(imu660rb_gyro_z);
#elif AUTO_SEEKFREE_USE_IMU660RX
    imu660rx_get_gyro();
    *gyro_z_dps = imu660rx_gyro_transition(imu660rx_gyro_z);
#elif AUTO_SEEKFREE_USE_IMU963RA
    imu963ra_get_gyro();
    *gyro_z_dps = imu963ra_gyro_transition(imu963ra_gyro_z)
                  * AUTO_SEEKFREE_GYRO_SCALE;
#else
    *gyro_z_dps = 0.0f;
    return false;
#endif

    *gyro_z_dps *= AUTO_SEEKFREE_GYRO_SIGN;
    return true;
}

static void seekfree_calibrate_gyro_bias(void)
{
    uint32_t i;
    float sum_dps = 0.0f;
    float gyro_z_dps;

    s_gyro_z_bias_dps = 0.0f;
    for (i = 0u; i < AUTO_SEEKFREE_GYRO_BIAS_SAMPLES; ++i) {
        if (seekfree_read_gyro_z_dps(&gyro_z_dps)) {
            sum_dps += gyro_z_dps;
        }
        system_delay_ms(AUTO_SEEKFREE_GYRO_BIAS_DELAY_MS);
    }

    if (AUTO_SEEKFREE_GYRO_BIAS_SAMPLES > 0u) {
        s_gyro_z_bias_dps = sum_dps / (float)AUTO_SEEKFREE_GYRO_BIAS_SAMPLES;
    }
    s_last_gyro_time_ms = 0.0f;
}

static bool seekfree_read_imu_heading_deg(void *ctx, float *heading_deg)
{
    uint32_t now_ms;
    float dt_s;
    float gyro_z_dps;

    (void)ctx;

    if (heading_deg == 0) {
        return false;
    }

    if (!seekfree_read_gyro_z_dps(&gyro_z_dps)) {
        *heading_deg = s_heading_deg;
        return true;
    }
    gyro_z_dps -= s_gyro_z_bias_dps;

    now_ms = system_getval_ms();
    if (s_last_gyro_time_ms <= 0.0f) {
        s_last_gyro_time_ms = (float)now_ms;
        *heading_deg = s_heading_deg;
        return true;
    }

    dt_s = ((float)now_ms - s_last_gyro_time_ms) / 1000.0f;
    s_last_gyro_time_ms = (float)now_ms;

    s_heading_deg = normalize_heading_deg(s_heading_deg + gyro_z_dps * dt_s);
    *heading_deg = s_heading_deg;
    return true;
}

static bool seekfree_read_gps_lat_lon(void *ctx, double *lat_deg, double *lon_deg)
{
    (void)ctx;

    if (lat_deg == 0 || lon_deg == 0) {
        return false;
    }

#if AUTO_SEEKFREE_USE_GNSS
    if (gnss_data_parse()) {
        *lat_deg = gnss.latitude;
        *lon_deg = gnss.longitude;
        return true;
    }
#endif

    return false;
}

static float seekfree_steer_adc_to_angle_deg(int16 steer_adc)
{
    float percent;
    float span;

    if (steer_adc < STEER_ADC_CENTER) {
        span = (float)(STEER_ADC_CENTER - STEER_ADC_LEFT_MAX);
        if (span < 1.0f) {
            span = 1.0f;
        }
        percent = ((float)steer_adc - (float)STEER_ADC_CENTER) / span;
    } else {
        span = (float)(STEER_ADC_RIGHT_MAX - STEER_ADC_CENTER);
        if (span < 1.0f) {
            span = 1.0f;
        }
        percent = ((float)steer_adc - (float)STEER_ADC_CENTER) / span;
    }

    if (percent < -1.0f) {
        percent = -1.0f;
    } else if (percent > 1.0f) {
        percent = 1.0f;
    }

    return -percent * (AUTO_SEEKFREE_STEER_TOTAL_DEG * 0.5f);
}

static void seekfree_fill_motion_sample(auto_motion_sample_t *sample,
                                        int16 left_count,
                                        int16 right_count)
{
    float left_delta_cm;
    float right_delta_cm;
    float center_delta_cm;
    float wheel_heading_rad;
    float steer_heading_rad;
    float rear_track_cm = AUTO_SEEKFREE_REAR_TRACK_CM;
    float wheelbase_cm = AUTO_SEEKFREE_WHEELBASE_CM;
    int16 steer_adc;

    left_delta_cm = (float)left_count *
                    AUTO_SEEKFREE_LEFT_ENCODER_SIGN *
                    AUTO_SEEKFREE_ENCODER_CM_PER_COUNT;
    right_delta_cm = (float)right_count *
                     AUTO_SEEKFREE_RIGHT_ENCODER_SIGN *
                     AUTO_SEEKFREE_ENCODER_CM_PER_COUNT;

#if AUTO_SEEKFREE_ENCODER_USE_AVERAGE
    center_delta_cm = (left_delta_cm + right_delta_cm) * 0.5f;
#else
    center_delta_cm = right_delta_cm;
#endif

    s_encoder_total_cm += center_delta_cm;

    memset(sample, 0, sizeof(*sample));
    sample->delta_left_cm = left_delta_cm;
    sample->delta_right_cm = right_delta_cm;
    sample->delta_center_cm = center_delta_cm;
    sample->distance_cm = s_encoder_total_cm;

    if (rear_track_cm < 1.0f) {
        rear_track_cm = 1.0f;
    }
    if (wheelbase_cm < 1.0f) {
        wheelbase_cm = 1.0f;
    }

    wheel_heading_rad =
        (right_delta_cm - left_delta_cm) / rear_track_cm;
    sample->wheel_heading_delta_deg =
        wheel_heading_rad * SEEKFREE_RAD_TO_DEG;

    steer_adc = (int16)Tern_Motor_Read_ADC();
    sample->steer_angle_deg = seekfree_steer_adc_to_angle_deg(steer_adc);
    steer_heading_rad =
        tanf(sample->steer_angle_deg * SEEKFREE_DEG_TO_RAD) *
        center_delta_cm / wheelbase_cm;
    sample->steer_heading_delta_deg =
        steer_heading_rad * SEEKFREE_RAD_TO_DEG;
}

static bool seekfree_read_motion_sample(void *ctx, auto_motion_sample_t *sample)
{
    int16 left_count;
    int16 right_count;

    (void)ctx;

    if (sample == 0) {
        return false;
    }

#if AUTO_SEEKFREE_USE_ENCODER
    left_count = encoder_get_count(AUTO_SEEKFREE_LEFT_ENCODER_INDEX);
    right_count = encoder_get_count(AUTO_SEEKFREE_RIGHT_ENCODER_INDEX);
    encoder_clear_count(AUTO_SEEKFREE_LEFT_ENCODER_INDEX);
    encoder_clear_count(AUTO_SEEKFREE_RIGHT_ENCODER_INDEX);

    s_encoder_left_count = left_count;
    s_encoder_right_count = right_count;

    seekfree_fill_motion_sample(sample, left_count, right_count);
    return true;
#else
    (void)left_count;
    (void)right_count;
    return false;
#endif
}

static bool seekfree_read_encoder_distance_cm(void *ctx, float *distance_cm)
{
    auto_motion_sample_t sample;

    (void)ctx;

    if (distance_cm == 0) {
        return false;
    }

    if (seekfree_read_motion_sample(ctx, &sample)) {
        *distance_cm = sample.distance_cm;
        return true;
    }

    return false;
}

static bool seekfree_is_emergency_stop(void *ctx)
{
    (void)ctx;
    return false;
}

static void seekfree_set_drive_percent(void *ctx, float speed_percent)
{
    int16 pwm;

    (void)ctx;

    if (speed_percent > 100.0f) {
        speed_percent = 100.0f;
    } else if (speed_percent < -100.0f) {
        speed_percent = -100.0f;
    }

    /* 最低速度门限：低于此绝对值直接停止，消除死区附近电机抖振 */
    if (speed_percent > -AUTO_SPEED_MIN_PERCENT &&
        speed_percent < AUTO_SPEED_MIN_PERCENT) {
        Set_Left_Pwm(0);
        Set_Right_Pwm(0);
        return;
    }

    pwm = (int16)(speed_percent * AUTO_SEEKFREE_DRIVE_PWM_MAX / 100.0f);
    /* Negate: positive speed_percent = forward; hardware needs negative PWM for forward */
    Set_Left_Pwm(-pwm);
    Set_Right_Pwm(-pwm);
}

static void seekfree_set_drive_steer_percent(void *ctx,
                                             float speed_percent,
                                             float steer_percent)
{
    float abs_steer;
    float ratio;
    float left_percent;
    float right_percent;
    int16 left_pwm;
    int16 right_pwm;

    (void)ctx;

    if (speed_percent > 100.0f) {
        speed_percent = 100.0f;
    } else if (speed_percent < -100.0f) {
        speed_percent = -100.0f;
    }

    if (steer_percent > 100.0f) {
        steer_percent = 100.0f;
    } else if (steer_percent < -100.0f) {
        steer_percent = -100.0f;
    }

    if (speed_percent > -AUTO_SPEED_MIN_PERCENT &&
        speed_percent < AUTO_SPEED_MIN_PERCENT) {
        Set_Left_Pwm(0);
        Set_Right_Pwm(0);
        return;
    }

    left_percent = speed_percent;
    right_percent = speed_percent;

#if AUTO_SEEKFREE_DIFF_STEER_ENABLE
    abs_steer = steer_percent < 0.0f ? -steer_percent : steer_percent;
    if (abs_steer > AUTO_SEEKFREE_DIFF_STEER_START_PERCENT) {
        ratio = (abs_steer - AUTO_SEEKFREE_DIFF_STEER_START_PERCENT) /
                (100.0f - AUTO_SEEKFREE_DIFF_STEER_START_PERCENT);
        ratio = auto_clampf(ratio, 0.0f, 1.0f) *
                AUTO_SEEKFREE_DIFF_STEER_MAX_RATIO;

        if (speed_percent > 0.0f) {
            if (steer_percent < 0.0f) {
                right_percent *= (1.0f - ratio);
            } else {
                left_percent *= (1.0f - ratio);
            }
        } else if (speed_percent < 0.0f) {
            if (steer_percent < 0.0f) {
                left_percent *= (1.0f - ratio);
            } else {
                right_percent *= (1.0f - ratio);
            }
        }
    }
#endif

    left_pwm = (int16)(left_percent * AUTO_SEEKFREE_DRIVE_PWM_MAX / 100.0f);
    right_pwm = (int16)(right_percent * AUTO_SEEKFREE_DRIVE_PWM_MAX / 100.0f);
    Set_Left_Pwm(-left_pwm);
    Set_Right_Pwm(-right_pwm);
}

static void seekfree_set_steer_percent(void *ctx, float steer_percent)
{
    float abs_steer;

    (void)ctx;

    if (steer_percent > 100.0f) {
        steer_percent = 100.0f;
    } else if (steer_percent < -100.0f) {
        steer_percent = -100.0f;
    }

    /* 死区门槛：低于最小转向量直接停（避免死区内振荡） */
    abs_steer = steer_percent < 0.0f ? -steer_percent : steer_percent;
    if (abs_steer < AUTO_STEER_MIN_PERCENT) {
        Set_Steering_Pwm(0);
        return;
    }

#if AUTO_SEEKFREE_STEER_CLOSED_LOOP
    int16 steer_pwm;

    Steering_Set_Target_Angle(-steer_percent);
    steer_pwm = Steering_PID_Calc();
    if (abs_steer >= AUTO_SEEKFREE_STEER_CLOSED_MIN_CMD_PERCENT) {
        if (steer_pwm > 0 &&
            steer_pwm < (int16)AUTO_SEEKFREE_STEER_CLOSED_PWM_MIN) {
            steer_pwm = (int16)AUTO_SEEKFREE_STEER_CLOSED_PWM_MIN;
        } else if (steer_pwm < 0 &&
                   steer_pwm > -(int16)AUTO_SEEKFREE_STEER_CLOSED_PWM_MIN) {
            steer_pwm = -(int16)AUTO_SEEKFREE_STEER_CLOSED_PWM_MIN;
        }
    }
    Set_Steering_Pwm(steer_pwm);
#else
    int16 pwm;
    float pwm_abs;

    pwm_abs = abs_steer * AUTO_SEEKFREE_STEER_PWM_MAX / 100.0f;
    if (pwm_abs < AUTO_SEEKFREE_STEER_PWM_MIN) {
        pwm_abs = AUTO_SEEKFREE_STEER_PWM_MIN;
    } else if (pwm_abs > AUTO_SEEKFREE_STEER_PWM_MAX) {
        pwm_abs = AUTO_SEEKFREE_STEER_PWM_MAX;
    }

    /* Positive steer command keeps the original hardware sign convention. */
    pwm = (int16)((steer_percent > 0.0f) ? pwm_abs : -pwm_abs);
    Set_Steering_Pwm(-pwm);
#endif
}

static void seekfree_set_brake(void *ctx, bool enable)
{
    (void)ctx;

    if (enable) {
        Set_Left_Pwm(0);
        Set_Right_Pwm(0);
    }
}

static void seekfree_log_text(void *ctx, const char *text)
{
    (void)ctx;

    if (text != 0) {
        printf("%s\r\n", text);
    }
}

void auto_seekfree_platform_init(void)
{
    int16 steer_adc_now;

    Motor_Init();
    Pedal_Init();
    Steering_PID_Init(AUTO_SEEKFREE_STEER_KP,
                      AUTO_SEEKFREE_STEER_KI,
                      AUTO_SEEKFREE_STEER_KD,
                      AUTO_SEEKFREE_STEER_I_LIMIT);

    /* 读取当前转向ADC，把PID目标初始化为当前位置，防止上电时打死方向盘 */
    steer_adc_now = (int16)Tern_Motor_Read_ADC();
    Steering_Set_Target(steer_adc_now);
    /* 打印ADC值：把前轮手动摆到中心位置后看串口输出，
     * 把打印出的值填到 PID.h 的 STEER_ADC_CENTER */
    printf("STEER_CALIB: ADC/10=%d  (center=%d left=%d right=%d)\r\n",
           steer_adc_now, STEER_ADC_CENTER, STEER_ADC_LEFT_MAX, STEER_ADC_RIGHT_MAX);

#if AUTO_SEEKFREE_USE_IMU660RA
    while (imu660ra_init()) {
        system_delay_ms(100);
    }
#elif AUTO_SEEKFREE_USE_IMU660RB
    while (imu660rb_init()) {
        system_delay_ms(100);
    }
#elif AUTO_SEEKFREE_USE_IMU660RX
    while (imu660rx_init()) {
        system_delay_ms(100);
    }
#elif AUTO_SEEKFREE_USE_IMU963RA
    while (imu963ra_init()) {
        system_delay_ms(100);
    }
#endif

    seekfree_calibrate_gyro_bias();

#if AUTO_SEEKFREE_USE_GNSS
    gnss_init(AUTO_SEEKFREE_GNSS_TYPE);
#endif

#if AUTO_SEEKFREE_USE_ENCODER
    encoder_dir_init(AUTO_SEEKFREE_LEFT_ENCODER_INDEX,
                     AUTO_SEEKFREE_LEFT_ENCODER_COUNT_PIN,
                     AUTO_SEEKFREE_LEFT_ENCODER_DIR_PIN);
    encoder_dir_init(AUTO_SEEKFREE_RIGHT_ENCODER_INDEX,
                     AUTO_SEEKFREE_RIGHT_ENCODER_COUNT_PIN,
                     AUTO_SEEKFREE_RIGHT_ENCODER_DIR_PIN);
    auto_seekfree_encoder_reset_distance();
#endif
}

auto_platform_t auto_seekfree_platform_create(void)
{
    auto_platform_t platform;

    platform.ctx = 0;
    platform.now_ms = seekfree_now_ms;
    platform.read_imu_heading_deg = seekfree_read_imu_heading_deg;
    platform.read_gps_lat_lon = seekfree_read_gps_lat_lon;
    platform.read_encoder_distance_cm = seekfree_read_encoder_distance_cm;
    platform.read_motion_sample = seekfree_read_motion_sample;
    platform.is_emergency_stop = seekfree_is_emergency_stop;
    platform.set_drive_percent = seekfree_set_drive_percent;
    platform.set_drive_steer_percent = seekfree_set_drive_steer_percent;
    platform.set_steer_percent = seekfree_set_steer_percent;
    platform.set_brake = seekfree_set_brake;
    platform.log_text = seekfree_log_text;

    return platform;
}
