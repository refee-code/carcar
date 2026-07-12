#include "auto_pose.h"

#include <math.h>
#include <string.h>

#include "auto_config.h"

#define AUTO_EARTH_RADIUS_CM (637100000.0)
#define AUTO_DEG_TO_RAD      (0.017453292519943295)

static float normalize_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float clampf_local(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float absf_local(float value)
{
    return value < 0.0f ? -value : value;
}

static bool yaw_delta_plausible(float delta_deg)
{
    return absf_local(delta_deg) <= AUTO_FUSION_MAX_YAW_DELTA_DEG;
}

static bool yaw_delta_agree(float a_deg, float b_deg)
{
    const float small_deg = 0.8f;

    if (absf_local(a_deg) <= small_deg && absf_local(b_deg) <= small_deg) {
        return true;
    }

    if ((a_deg > 0.0f && b_deg < 0.0f) ||
        (a_deg < 0.0f && b_deg > 0.0f)) {
        return false;
    }

    return absf_local(a_deg - b_deg) <= AUTO_FUSION_AGREE_TOL_DEG;
}

static void gps_to_local_cm(double origin_lat_deg,
                            double origin_lon_deg,
                            double lat_deg,
                            double lon_deg,
                            float *x_cm,
                            float *y_cm)
{
    const double lat0 = origin_lat_deg * AUTO_DEG_TO_RAD;
    const double dlat = (lat_deg - origin_lat_deg) * AUTO_DEG_TO_RAD;
    const double dlon = (lon_deg - origin_lon_deg) * AUTO_DEG_TO_RAD;

    *x_cm = (float)(AUTO_EARTH_RADIUS_CM * dlon * cos(lat0));
    *y_cm = (float)(AUTO_EARTH_RADIUS_CM * dlat);
}

void auto_pose_init(auto_pose_estimator_t *estimator)
{
    if (estimator == 0) {
        return;
    }
    memset(estimator, 0, sizeof(*estimator));
}

void auto_pose_reset(auto_pose_estimator_t *estimator, uint32_t now_ms)
{
    float saved_heading;

    if (estimator == 0) {
        return;
    }

    /* Preserve heading across reset so the car knows its orientation */
    saved_heading = estimator->pose.heading_deg;

    memset(estimator, 0, sizeof(*estimator));

    estimator->pose.heading_deg = saved_heading;

    /* Initialize timestamps to now so safety checks don't trigger immediately */
    estimator->pose.last_imu_ms     = now_ms;
    estimator->pose.last_gps_ms     = now_ms;
    estimator->pose.last_encoder_ms = now_ms;
}

auto_status_t auto_pose_update(auto_pose_estimator_t *estimator,
                               const auto_platform_t *port,
                               uint32_t now_ms)
{
    float heading_deg;
    float encoder_distance_cm;
    double lat_deg;
    double lon_deg;
    bool have_gps = false;
    bool have_encoder = false;
    float previous_heading_deg;
    float gyro_delta_deg = 0.0f;

    if (estimator == 0 || port == 0) {
        return AUTO_ERR_INVALID_ARG;
    }

    estimator->pose.time_ms = now_ms;
    previous_heading_deg = estimator->pose.heading_deg;

    if (port->read_imu_heading_deg != 0 &&
        port->read_imu_heading_deg(port->ctx, &heading_deg)) {
        estimator->pose.heading_deg = heading_deg;
        estimator->pose.imu_valid = true;
        estimator->pose.last_imu_ms = now_ms;
    }
    gyro_delta_deg = normalize_angle_deg(estimator->pose.heading_deg -
                                         previous_heading_deg);

    estimator->pose.wheel_heading_delta_deg = 0.0f;
    estimator->pose.steer_angle_deg = 0.0f;
    estimator->pose.steer_heading_delta_deg = 0.0f;
    estimator->pose.heading_correction_deg = 0.0f;
    estimator->pose.heading_consistency = 0u;

    if (port->read_motion_sample != 0) {
        auto_motion_sample_t sample;

        memset(&sample, 0, sizeof(sample));
        if (port->read_motion_sample(port->ctx, &sample)) {
            float delta_cm = sample.delta_center_cm;
            float correction_deg = 0.0f;
            bool enough_motion =
                absf_local(delta_cm) >= AUTO_FUSION_MIN_DELTA_CM;
            bool consistent =
                enough_motion &&
                yaw_delta_plausible(gyro_delta_deg) &&
                yaw_delta_plausible(sample.wheel_heading_delta_deg) &&
                yaw_delta_plausible(sample.steer_heading_delta_deg) &&
                yaw_delta_agree(gyro_delta_deg,
                                sample.wheel_heading_delta_deg) &&
                yaw_delta_agree(gyro_delta_deg,
                                sample.steer_heading_delta_deg);

            if (consistent) {
                float assist_delta =
                    (sample.wheel_heading_delta_deg +
                     sample.steer_heading_delta_deg) * 0.5f;
                correction_deg =
                    clampf_local((assist_delta - gyro_delta_deg) *
                                 AUTO_FUSION_CORRECTION_GAIN,
                                 -AUTO_FUSION_CORRECTION_LIMIT_DEG,
                                 AUTO_FUSION_CORRECTION_LIMIT_DEG);
                estimator->pose.heading_deg =
                    normalize_angle_deg(estimator->pose.heading_deg +
                                        correction_deg);
                gyro_delta_deg =
                    normalize_angle_deg(estimator->pose.heading_deg -
                                        previous_heading_deg);
                estimator->pose.heading_consistency = 1u;
            }

            estimator->last_encoder_distance_cm = sample.distance_cm;
            estimator->encoder_seen = true;
            estimator->pose.distance_cm = sample.distance_cm;
            estimator->pose.encoder_valid = true;
            estimator->pose.last_encoder_ms = now_ms;
            estimator->pose.wheel_heading_delta_deg =
                sample.wheel_heading_delta_deg;
            estimator->pose.steer_angle_deg = sample.steer_angle_deg;
            estimator->pose.steer_heading_delta_deg =
                sample.steer_heading_delta_deg;
            estimator->pose.heading_correction_deg = correction_deg;
            have_encoder = true;

            if (estimator->pose.imu_valid && delta_cm != 0.0f) {
                float motion_heading_deg =
                    previous_heading_deg + gyro_delta_deg * 0.5f;
                float heading_rad =
                    motion_heading_deg * (float)AUTO_DEG_TO_RAD;
                estimator->pose.x_cm += delta_cm * cosf(heading_rad);
                estimator->pose.y_cm += delta_cm * sinf(heading_rad);
            }
        }
    }

    if (!have_encoder &&
        port->read_encoder_distance_cm != 0 &&
        port->read_encoder_distance_cm(port->ctx, &encoder_distance_cm)) {
        float delta_cm = 0.0f;

        if (estimator->encoder_seen) {
            delta_cm = encoder_distance_cm - estimator->last_encoder_distance_cm;
        }

        estimator->last_encoder_distance_cm = encoder_distance_cm;
        estimator->encoder_seen = true;
        estimator->pose.distance_cm = encoder_distance_cm;
        estimator->pose.encoder_valid = true;
        estimator->pose.last_encoder_ms = now_ms;
        have_encoder = true;

        if (estimator->pose.imu_valid && delta_cm != 0.0f) {
            float motion_heading_deg;
            float heading_rad;
            motion_heading_deg = previous_heading_deg + gyro_delta_deg * 0.5f;
            heading_rad = motion_heading_deg * (float)AUTO_DEG_TO_RAD;
            estimator->pose.x_cm += delta_cm * cosf(heading_rad);
            estimator->pose.y_cm += delta_cm * sinf(heading_rad);
        }
    }

    if (port->read_gps_lat_lon != 0 &&
        port->read_gps_lat_lon(port->ctx, &lat_deg, &lon_deg)) {
        float gps_x_cm, gps_y_cm;

        if (!estimator->origin_set) {
            estimator->origin_lat_deg = lat_deg;
            estimator->origin_lon_deg = lon_deg;
            estimator->origin_set = true;
            estimator->pose.x_cm = 0.0f;
            estimator->pose.y_cm = 0.0f;
        }

        gps_to_local_cm(estimator->origin_lat_deg,
                        estimator->origin_lon_deg,
                        lat_deg,
                        lon_deg,
                        &gps_x_cm,
                        &gps_y_cm);

        /* Complementary filter: GPS corrects encoder drift slowly
         * If encoder is active, blend GPS slowly (5% weight per update)
         * Otherwise use GPS directly
         */
        if (have_encoder && estimator->encoder_seen) {
            const float GPS_BLEND_FACTOR = 0.05f;
            estimator->pose.x_cm = estimator->pose.x_cm * (1.0f - GPS_BLEND_FACTOR)
                                   + gps_x_cm * GPS_BLEND_FACTOR;
            estimator->pose.y_cm = estimator->pose.y_cm * (1.0f - GPS_BLEND_FACTOR)
                                   + gps_y_cm * GPS_BLEND_FACTOR;
        } else {
            estimator->pose.x_cm = gps_x_cm;
            estimator->pose.y_cm = gps_y_cm;
        }

        estimator->pose.gps_valid = true;
        estimator->pose.last_gps_ms = now_ms;
        have_gps = true;
    }

    (void)have_gps;
    (void)have_encoder;
    return AUTO_OK;
}

const auto_pose_t *auto_pose_get(const auto_pose_estimator_t *estimator)
{
    return estimator == 0 ? 0 : &estimator->pose;
}
