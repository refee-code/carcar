#include "auto_nav.h"

#include <string.h>

#include "auto_config.h"
#include "auto_math.h"

/* 沿路径段积累长度，找前视点 */
static size_t find_lookahead_index(const auto_route_t *route,
                                   size_t start_index,
                                   float lookahead_cm)
{
    size_t i;
    float accumulated = 0.0f;

    for (i = start_index; i + 1u < route->count; ++i) {
        accumulated += auto_distance_cm(route->points[i].x_cm,
                                        route->points[i].y_cm,
                                        route->points[i + 1u].x_cm,
                                        route->points[i + 1u].y_cm);
        if (accumulated >= lookahead_cm) {
            return i + 1u;
        }
    }
    return route->count - 1u;
}

/* 投影参数 t：车辆在路径段 AB 上的进度，<0=未到A, 0~1=在段内, >1=已过B */
static float segment_projection_t(float ax, float ay,
                                   float bx, float by,
                                   float px, float py)
{
    float dx = bx - ax;
    float dy = by - ay;
    float len_sq = dx*dx + dy*dy;
    if (len_sq < 1.0f) return 0.0f;
    return ((px - ax)*dx + (py - ay)*dy) / len_sq;
}

static float route_length_to_index(const auto_route_t *route, size_t end_index)
{
    size_t i;
    float length_cm = 0.0f;

    if (route == 0 || route->points == 0 || route->count == 0u) {
        return 0.0f;
    }

    if (end_index >= route->count) {
        end_index = route->count - 1u;
    }

    for (i = 0u; i < end_index; ++i) {
        length_cm += auto_distance_cm(route->points[i].x_cm,
                                      route->points[i].y_cm,
                                      route->points[i + 1u].x_cm,
                                      route->points[i + 1u].y_cm);
    }

    return length_cm;
}

static size_t distance_limited_index(const auto_route_t *route,
                                     float distance_cm,
                                     size_t lead_limit_points)
{
    size_t i;
    size_t index = 0u;
    float travelled_cm;
    float accumulated_cm = 0.0f;

    if (route == 0 || route->points == 0 || route->count == 0u) {
        return 0u;
    }

    travelled_cm = distance_cm;
    if (travelled_cm < 0.0f) {
        travelled_cm = -travelled_cm;
    }

    for (i = 0u; i + 1u < route->count; ++i) {
        float seg_len = auto_distance_cm(route->points[i].x_cm,
                                         route->points[i].y_cm,
                                         route->points[i + 1u].x_cm,
                                         route->points[i + 1u].y_cm);
        if ((accumulated_cm + seg_len) <= travelled_cm) {
            accumulated_cm += seg_len;
            index = i + 1u;
        } else {
            break;
        }
    }

    index += lead_limit_points;
    if (index >= route->count) {
        index = route->count - 1u;
    }
    return index;
}

static float absolute_distance_cm(float distance_cm)
{
    return distance_cm < 0.0f ? -distance_cm : distance_cm;
}

static float route_segment_heading_deg(const auto_route_t *route,
                                       size_t segment_index)
{
    if (route == 0 || route->points == 0 || route->count < 2u) {
        return 0.0f;
    }

    if (segment_index >= route->count - 1u) {
        segment_index = route->count - 2u;
    }

    return auto_heading_to_deg(route->points[segment_index].x_cm,
                               route->points[segment_index].y_cm,
                               route->points[segment_index + 1u].x_cm,
                               route->points[segment_index + 1u].y_cm);
}

static bool route_target_is_behind_segment(const auto_route_t *route,
                                           size_t segment_index,
                                           const auto_pose_t *pose,
                                           float target_x_cm,
                                           float target_y_cm)
{
    const auto_waypoint_t *a;
    const auto_waypoint_t *b;
    float seg_dx;
    float seg_dy;
    float target_dx;
    float target_dy;

    if (route == 0 || route->points == 0 || route->count < 2u ||
        pose == 0) {
        return false;
    }

    if (segment_index >= route->count - 1u) {
        segment_index = route->count - 2u;
    }

    a = &route->points[segment_index];
    b = &route->points[segment_index + 1u];
    seg_dx = b->x_cm - a->x_cm;
    seg_dy = b->y_cm - a->y_cm;
    target_dx = target_x_cm - pose->x_cm;
    target_dy = target_y_cm - pose->y_cm;

    if ((seg_dx * seg_dx + seg_dy * seg_dy) < 1.0f ||
        (target_dx * target_dx + target_dy * target_dy) < 4.0f) {
        return true;
    }

    return (seg_dx * target_dx + seg_dy * target_dy) < 0.0f;
}

static float route_distance_to_next_turn_cm(const auto_route_t *route,
                                            size_t segment_index,
                                            float segment_t,
                                            float segment_heading_deg)
{
    size_t i;
    float distance_cm = 0.0f;

    if (route == 0 || route->points == 0 || route->count < 2u) {
        return 1.0e30f;
    }

    if (segment_index >= route->count - 1u) {
        return 1.0e30f;
    }

    segment_t = auto_clampf(segment_t, 0.0f, 1.0f);

    for (i = segment_index; i + 1u < route->count; ++i) {
        const auto_waypoint_t *a = &route->points[i];
        const auto_waypoint_t *b = &route->points[i + 1u];
        float seg_len = auto_distance_cm(a->x_cm, a->y_cm,
                                         b->x_cm, b->y_cm);
        float heading_delta;

        if (seg_len < 1.0f) {
            continue;
        }

        if (i != segment_index) {
            float heading_deg = auto_heading_to_deg(a->x_cm, a->y_cm,
                                                    b->x_cm, b->y_cm);
            heading_delta = auto_absf(auto_normalize_angle_deg(
                                          heading_deg - segment_heading_deg));
            if (heading_delta >= AUTO_DYNAMIC_TURN_DETECT_DEG) {
                return distance_cm;
            }
        }

        if (i == segment_index) {
            distance_cm += seg_len * (1.0f - segment_t);
        } else {
            distance_cm += seg_len;
        }

        if (distance_cm > AUTO_DYNAMIC_PRETURN_START_CM +
                          AUTO_NAV_LOOKAHEAD_DISTANCE_CM) {
            break;
        }
    }

    return 1.0e30f;
}

static float route_dynamic_heading_deg(float segment_heading_deg,
                                       float preview_heading_deg,
                                       float turn_distance_cm)
{
    float blend;
    float preview_delta_deg;

    if (turn_distance_cm > AUTO_DYNAMIC_PRETURN_START_CM) {
        return segment_heading_deg;
    }

    if (turn_distance_cm <= AUTO_DYNAMIC_PRETURN_FULL_CM) {
        blend = 1.0f;
    } else {
        blend = (AUTO_DYNAMIC_PRETURN_START_CM - turn_distance_cm) /
                (AUTO_DYNAMIC_PRETURN_START_CM -
                 AUTO_DYNAMIC_PRETURN_FULL_CM);
    }
    blend = auto_clampf(blend, 0.0f, 1.0f);
    blend = blend * blend * (3.0f - 2.0f * blend);
    preview_delta_deg = auto_normalize_angle_deg(preview_heading_deg -
                                                 segment_heading_deg);
    preview_delta_deg = auto_clampf(preview_delta_deg,
                                    -AUTO_DYNAMIC_PREVIEW_HEADING_LIMIT_DEG,
                                    AUTO_DYNAMIC_PREVIEW_HEADING_LIMIT_DEG);

    return auto_normalize_angle_deg(segment_heading_deg +
                                    preview_delta_deg * blend);
}

static void route_segment_at_distance(const auto_route_t *route,
                                      float distance_cm,
                                      size_t *segment_index,
                                      float *segment_t)
{
    size_t i;
    float travelled_cm;
    float accumulated_cm = 0.0f;

    if (segment_index == 0 || segment_t == 0) {
        return;
    }

    *segment_index = 0u;
    *segment_t = 0.0f;

    if (route == 0 || route->points == 0 || route->count < 2u) {
        return;
    }

    travelled_cm = absolute_distance_cm(distance_cm);
    for (i = 0u; i + 1u < route->count; ++i) {
        float seg_len = auto_distance_cm(route->points[i].x_cm,
                                         route->points[i].y_cm,
                                         route->points[i + 1u].x_cm,
                                         route->points[i + 1u].y_cm);
        if (seg_len < 1.0f) {
            continue;
        }
        if ((accumulated_cm + seg_len) >= travelled_cm) {
            *segment_index = i;
            *segment_t = (travelled_cm - accumulated_cm) / seg_len;
            if (*segment_t < 0.0f) {
                *segment_t = 0.0f;
            } else if (*segment_t > 1.0f) {
                *segment_t = 1.0f;
            }
            return;
        }
        accumulated_cm += seg_len;
    }

    *segment_index = route->count - 2u;
    *segment_t = 1.0f;
}

static void route_point_ahead(const auto_route_t *route,
                              size_t segment_index,
                              float segment_t,
                              float lookahead_cm,
                              float *target_x_cm,
                              float *target_y_cm)
{
    size_t i;
    float remaining_cm = lookahead_cm;

    if (route == 0 || route->points == 0 || route->count == 0u ||
        target_x_cm == 0 || target_y_cm == 0) {
        return;
    }

    if (route->count == 1u) {
        *target_x_cm = route->points[0].x_cm;
        *target_y_cm = route->points[0].y_cm;
        return;
    }

    if (segment_index >= route->count - 1u) {
        segment_index = route->count - 2u;
        segment_t = 1.0f;
    }
    if (segment_t < 0.0f) {
        segment_t = 0.0f;
    } else if (segment_t > 1.0f) {
        segment_t = 1.0f;
    }

    for (i = segment_index; i + 1u < route->count; ++i) {
        const auto_waypoint_t *a = &route->points[i];
        const auto_waypoint_t *b = &route->points[i + 1u];
        float dx = b->x_cm - a->x_cm;
        float dy = b->y_cm - a->y_cm;
        float seg_len = auto_distance_cm(a->x_cm, a->y_cm,
                                         b->x_cm, b->y_cm);
        float start_t = (i == segment_index) ? segment_t : 0.0f;
        float available_cm = seg_len * (1.0f - start_t);

        if (seg_len < 1.0f) {
            continue;
        }
        if (remaining_cm <= available_cm) {
            float target_t = start_t + remaining_cm / seg_len;
            *target_x_cm = a->x_cm + dx * target_t;
            *target_y_cm = a->y_cm + dy * target_t;
            return;
        }
        remaining_cm -= available_cm;
    }

    *target_x_cm = route->points[route->count - 1u].x_cm;
    *target_y_cm = route->points[route->count - 1u].y_cm;
}

static size_t route_index_ahead_by_distance(const auto_route_t *route,
                                            size_t segment_index,
                                            float segment_t,
                                            float lookahead_cm)
{
    size_t i;
    float remaining_cm = lookahead_cm;

    if (route == 0 || route->points == 0 || route->count == 0u) {
        return 0u;
    }

    if (route->count == 1u) {
        return 0u;
    }

    if (segment_index >= route->count - 1u) {
        return route->count - 1u;
    }
    segment_t = auto_clampf(segment_t, 0.0f, 1.0f);

    for (i = segment_index; i + 1u < route->count; ++i) {
        const auto_waypoint_t *a = &route->points[i];
        const auto_waypoint_t *b = &route->points[i + 1u];
        float seg_len = auto_distance_cm(a->x_cm, a->y_cm,
                                         b->x_cm, b->y_cm);
        float start_t = (i == segment_index) ? segment_t : 0.0f;
        float available_cm = seg_len * (1.0f - start_t);

        if (seg_len < 1.0f) {
            continue;
        }
        if (remaining_cm <= available_cm) {
            return i + 1u;
        }
        remaining_cm -= available_cm;
    }

    return route->count - 1u;
}

static bool route_dynamic_drive_heading_at(const auto_route_t *route,
                                           size_t segment_index,
                                           float segment_t,
                                           float *drive_heading_deg)
{
    float h0;
    float h1;
    float delta;

    if (route == 0 || route->points == 0 || route->count == 0u ||
        drive_heading_deg == 0) {
        return false;
    }

    if (segment_index >= route->count) {
        segment_index = route->count - 1u;
    }

    if (!auto_route_get_dynamic_heading(segment_index, &h0)) {
        return false;
    }

    if (segment_index + 1u >= route->count ||
        !auto_route_get_dynamic_heading(segment_index + 1u, &h1)) {
        *drive_heading_deg = h0;
        return true;
    }

    segment_t = auto_clampf(segment_t, 0.0f, 1.0f);
    delta = auto_normalize_angle_deg(h1 - h0);
    *drive_heading_deg = auto_normalize_angle_deg(h0 + delta * segment_t);
    return true;
}

static bool route_dynamic_drive_heading_ahead_deg(const auto_route_t *route,
                                                  size_t segment_index,
                                                  float segment_t,
                                                  float lookahead_cm,
                                                  float *drive_heading_deg)
{
    size_t heading_index;
    float heading_deg;

    if (drive_heading_deg == 0) {
        return false;
    }

    heading_index = route_index_ahead_by_distance(route,
                                                  segment_index,
                                                  segment_t,
                                                  lookahead_cm);
    if (!auto_route_get_dynamic_heading(heading_index, &heading_deg)) {
        return false;
    }

    *drive_heading_deg = heading_deg;
    return true;
}

static float route_dynamic_steer_feedforward_percent(const auto_route_t *route,
                                                     size_t segment_index,
                                                     float segment_t,
                                                     bool driving_reverse)
{
    size_t steer_index;
    float steer_percent;
    float feedforward;

    steer_index = route_index_ahead_by_distance(
                      route,
                      segment_index,
                      segment_t,
                      AUTO_DYNAMIC_STEER_FEEDFORWARD_LOOKAHEAD_CM);
    if (!auto_route_get_dynamic_steer(steer_index, &steer_percent)) {
        return 0.0f;
    }

    if (auto_absf(steer_percent) <
        AUTO_DYNAMIC_STEER_FEEDFORWARD_DEAD_PERCENT) {
        return 0.0f;
    }

    feedforward =
        steer_percent * AUTO_DYNAMIC_STEER_FEEDFORWARD_GAIN;
    feedforward = auto_clampf(feedforward,
                              -AUTO_DYNAMIC_STEER_FEEDFORWARD_MAX_PERCENT,
                              AUTO_DYNAMIC_STEER_FEEDFORWARD_MAX_PERCENT);

    (void)driving_reverse;
    return feedforward;
}

static void route_project_pose(const auto_route_t *route,
                               size_t segment_index,
                               const auto_pose_t *pose,
                               float *segment_t)
{
    const auto_waypoint_t *a;
    const auto_waypoint_t *b;
    float t;

    if (route == 0 || route->points == 0 || route->count < 2u ||
        pose == 0 || segment_t == 0) {
        return;
    }

    if (segment_index >= route->count - 1u) {
        segment_index = route->count - 2u;
    }

    a = &route->points[segment_index];
    b = &route->points[segment_index + 1u];
    t = segment_projection_t(a->x_cm, a->y_cm,
                             b->x_cm, b->y_cm,
                             pose->x_cm, pose->y_cm);
    if (t < 0.0f) {
        t = 0.0f;
    } else if (t > 1.0f) {
        t = 1.0f;
    }
    *segment_t = t;
}

static void route_refine_segment_by_pose(const auto_route_t *route,
                                         size_t seed_segment_index,
                                         const auto_pose_t *pose,
                                         size_t *segment_index,
                                         float *segment_t)
{
    size_t first_segment;
    size_t last_segment;
    size_t i;
    size_t best_segment;
    float seed_t;
    float best_t = 0.0f;
    float best_dist_sq = 1.0e30f;
    float best_score_sq = 1.0e30f;

    if (route == 0 || route->points == 0 || route->count < 2u ||
        pose == 0 || segment_index == 0 || segment_t == 0) {
        return;
    }

    if (seed_segment_index >= route->count - 1u) {
        seed_segment_index = route->count - 2u;
    }

    first_segment = seed_segment_index >
                    AUTO_DYNAMIC_SEGMENT_REFINE_WINDOW_POINTS ?
                    seed_segment_index -
                    AUTO_DYNAMIC_SEGMENT_REFINE_WINDOW_POINTS :
                    0u;
    last_segment = seed_segment_index +
                   AUTO_DYNAMIC_SEGMENT_REFINE_WINDOW_POINTS;
    if (last_segment > route->count - 2u) {
        last_segment = route->count - 2u;
    }

    best_segment = seed_segment_index;
    seed_t = auto_clampf(*segment_t, 0.0f, 1.0f);
    for (i = first_segment; i <= last_segment; ++i) {
        const auto_waypoint_t *a = &route->points[i];
        const auto_waypoint_t *b = &route->points[i + 1u];
        float dx = b->x_cm - a->x_cm;
        float dy = b->y_cm - a->y_cm;
        float raw_t = segment_projection_t(a->x_cm,
                                           a->y_cm,
                                           b->x_cm,
                                           b->y_cm,
                                           pose->x_cm,
                                           pose->y_cm);
        float t = raw_t;
        float closest_x;
        float closest_y;
        float err_x;
        float err_y;
        float dist_sq;
        float score_sq;
        float drive_heading_deg;

        t = auto_clampf(t, 0.0f, 1.0f);
        closest_x = a->x_cm + dx * t;
        closest_y = a->y_cm + dy * t;
        err_x = pose->x_cm - closest_x;
        err_y = pose->y_cm - closest_y;
        dist_sq = err_x * err_x + err_y * err_y;
        score_sq = dist_sq;
        if (i != seed_segment_index) {
            size_t index_delta =
                (i > seed_segment_index) ? (i - seed_segment_index) :
                (seed_segment_index - i);
            float progress_penalty =
                (float)index_delta *
                AUTO_DYNAMIC_SEGMENT_INDEX_PENALTY_CM;
            score_sq += progress_penalty * progress_penalty;

            if ((raw_t <= 0.05f && seed_t > 0.15f) ||
                (raw_t >= 0.95f && seed_t < 0.85f) ||
                raw_t < -0.10f ||
                raw_t > 1.10f) {
                score_sq +=
                    AUTO_DYNAMIC_SEGMENT_ENDPOINT_PENALTY_CM *
                    AUTO_DYNAMIC_SEGMENT_ENDPOINT_PENALTY_CM;
            }
        }
        if (auto_route_is_dynamic() &&
            route_dynamic_drive_heading_at(route,
                                           i,
                                           t,
                                           &drive_heading_deg)) {
            float heading_delta = auto_absf(auto_normalize_angle_deg(
                                               drive_heading_deg -
                                               pose->heading_deg));
            if (heading_delta >
                AUTO_DYNAMIC_SEGMENT_HEADING_GATE_DEG) {
                float penalty =
                    (heading_delta -
                     AUTO_DYNAMIC_SEGMENT_HEADING_GATE_DEG) *
                    AUTO_DYNAMIC_SEGMENT_HEADING_PENALTY;
                score_sq += penalty * penalty;
            }
        }
        if (score_sq < best_score_sq) {
            best_score_sq = score_sq;
            best_dist_sq = dist_sq;
            best_segment = i;
            best_t = t;
        }
    }

    if (best_segment == seed_segment_index &&
        seed_segment_index > 0u &&
        best_t <= 0.05f) {
        const auto_waypoint_t *prev_a =
            &route->points[seed_segment_index - 1u];
        const auto_waypoint_t *prev_b =
            &route->points[seed_segment_index];
        float prev_t = segment_projection_t(prev_a->x_cm,
                                            prev_a->y_cm,
                                            prev_b->x_cm,
                                            prev_b->y_cm,
                                            pose->x_cm,
                                            pose->y_cm);
        bool allow_prev_segment = true;
        float prev_drive_heading_deg;
        if (auto_route_is_dynamic() &&
            route_dynamic_drive_heading_at(route,
                                           seed_segment_index - 1u,
                                           auto_clampf(prev_t, 0.0f, 1.0f),
                                           &prev_drive_heading_deg)) {
            float prev_heading_delta =
                auto_absf(auto_normalize_angle_deg(prev_drive_heading_deg -
                                                   pose->heading_deg));
            if (prev_heading_delta >
                (AUTO_DYNAMIC_SEGMENT_HEADING_GATE_DEG + 30.0f)) {
                allow_prev_segment = false;
            }
        }
        if (allow_prev_segment && prev_t < 0.95f) {
            best_segment = seed_segment_index - 1u;
            best_t = auto_clampf(prev_t, 0.0f, 1.0f);
        }
    }

    *segment_index = best_segment;
    *segment_t = best_t;
}

static bool route_end_reached(const auto_route_t *route,
                              const auto_pose_t *pose,
                              size_t nearest_index,
                              size_t target_index)
{
    const auto_waypoint_t *prev_wp;
    const auto_waypoint_t *last_wp;
    float dist_to_last_cm;
    float final_t;
    float route_length_cm;
    size_t last_index;

    if (route == 0 || pose == 0 ||
        route->points == 0 || route->count == 0u) {
        return false;
    }

    last_index = route->count - 1u;
    last_wp = &route->points[last_index];
    dist_to_last_cm = auto_distance_cm(pose->x_cm, pose->y_cm,
                                       last_wp->x_cm, last_wp->y_cm);

    route_length_cm = route_length_to_index(route, last_index);
    if (route_length_cm > 1.0f &&
        absolute_distance_cm(pose->distance_cm) + AUTO_ROUTE_DONE_MARGIN_CM >=
        route_length_cm &&
        dist_to_last_cm <= AUTO_ODOM_DONE_DISTANCE_CM) {
        return true;
    }

    if (nearest_index < last_index && target_index < last_index) {
        return false;
    }

    if (dist_to_last_cm <= AUTO_WAYPOINT_REACHED_CM) {
        return true;
    }

    if (route->count >= 2u) {
        prev_wp = &route->points[route->count - 2u];
        final_t = segment_projection_t(prev_wp->x_cm, prev_wp->y_cm,
                                       last_wp->x_cm, last_wp->y_cm,
                                       pose->x_cm, pose->y_cm);
        if (final_t >= 1.0f) {
            return true;
        }
    }

    return false;
}

static size_t find_progress_index(const auto_route_t *route,
                                  size_t current_index,
                                  const auto_pose_t *pose)
{
    size_t i;
    size_t start_seg;
    size_t end_seg;
    size_t search_window;
    size_t best_seg;
    size_t max_progress_index;
    size_t odometer_index;
    size_t progress_index;
    float best_dist_sq = 1.0e30f;
    float best_t = 0.0f;

    if (route->count < 2u || current_index >= route->count - 1u) {
        return current_index;
    }

    odometer_index = distance_limited_index(route, pose->distance_cm, 0u);
    max_progress_index = distance_limited_index(route,
                                                pose->distance_cm,
                                                AUTO_PROGRESS_INDEX_LEAD_LIMIT);
    if (max_progress_index <= current_index) {
        return current_index;
    }

    start_seg = current_index > 0u ? current_index - 1u : 0u;
    search_window = AUTO_NAV_SEARCH_WINDOW_POINTS;
    end_seg = current_index + search_window;
    if (end_seg > route->count - 2u) {
        end_seg = route->count - 2u;
    }

    best_seg = current_index;
    for (i = start_seg; i <= end_seg; ++i) {
        float ax = route->points[i].x_cm;
        float ay = route->points[i].y_cm;
        float bx = route->points[i + 1u].x_cm;
        float by = route->points[i + 1u].y_cm;
        float dx = bx - ax;
        float dy = by - ay;
        float len_sq = dx * dx + dy * dy;
        float t;
        float closest_x;
        float closest_y;
        float err_x;
        float err_y;
        float dist_sq;

        if (len_sq < 1.0f) {
            continue;
        }

        t = segment_projection_t(ax, ay, bx, by, pose->x_cm, pose->y_cm);
        if (i == current_index && t >= AUTO_PROGRESS_ADVANCE_T) {
            return current_index + 1u;
        }
        if (t < 0.0f) {
            t = 0.0f;
        } else if (t > 1.0f) {
            t = 1.0f;
        }

        closest_x = ax + dx * t;
        closest_y = ay + dy * t;
        err_x = pose->x_cm - closest_x;
        err_y = pose->y_cm - closest_y;
        dist_sq = err_x * err_x + err_y * err_y;

        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_seg = i;
            best_t = t;
        }
    }

    progress_index = best_seg;
    if ((best_t >= 0.9f &&
         best_dist_sq <=
         (AUTO_PROGRESS_MAX_CTE_CM * AUTO_PROGRESS_MAX_CTE_CM)) ||
        auto_distance_cm(pose->x_cm, pose->y_cm,
                         route->points[best_seg + 1u].x_cm,
                         route->points[best_seg + 1u].y_cm) <=
        AUTO_WAYPOINT_REACHED_CM) {
        progress_index = best_seg + 1u;
    }

    if (progress_index < odometer_index &&
        best_dist_sq <=
        (AUTO_ODOM_PROGRESS_MAX_CTE_CM *
         AUTO_ODOM_PROGRESS_MAX_CTE_CM)) {
        progress_index = odometer_index;
    }
    if (progress_index > max_progress_index) {
        progress_index = max_progress_index;
    }

    if (progress_index > current_index) {
        return progress_index;
    }

    return current_index;
}

/* 横向误差：车辆到路径段 AB 的带符号垂直距离（左正右负） */
static float cross_track_error(float ax, float ay,
                                float bx, float by,
                                float px, float py)
{
    float dx = bx - ax;
    float dy = by - ay;
    float seg_len = auto_distance_cm(ax, ay, bx, by);
    float ux, uy;
    if (seg_len < 1.0f) return 0.0f;
    ux = dx / seg_len;
    uy = dy / seg_len;
    /* 左侧法向量 (-uy, ux)，点积得横向误差 */
    return (px - ax)*(-uy) + (py - ay)*ux;
}

void auto_nav_init(auto_nav_t *nav)
{
    if (nav == 0) return;
    nav->nearest_index        = 0u;
    nav->target_index         = 0u;
    nav->initialized          = false;
    nav->dynamic_cte_abort_ticks = 0u;
    nav->prev_driving_reverse = false;
}

auto_status_t auto_nav_follow(auto_nav_t *nav,
                              const auto_route_t *route,
                              const auto_pose_t *pose,
                              auto_nav_result_t *out)
{
    const auto_waypoint_t *current_wp;
    const auto_waypoint_t *segment_start_wp;
    const auto_waypoint_t *segment_end_wp;
    float dist_to_current;
    float target_heading_deg;
    float raw_heading_err;
    float heading_err;
    float heading_kp;
    float steer_percent;
    float speed_percent;
    float reverse_err;
    float cte;
    float cte_steer;
    float steer_feedforward;
    float cte_abort_limit;
    float target_x_cm;
    float target_y_cm;
    float ref_x_cm;
    float ref_y_cm;
    float path_heading_deg;
    float dynamic_drive_heading_deg;
    float segment_t;
    float route_length_cm;
    float travelled_cm;
    float segment_heading_deg;
    float turn_distance_cm;
    size_t segment_index;
    bool driving_reverse;
    bool dynamic_route;
    bool target_behind_segment = false;
    bool safety_stop = false;
    bool distance_done = false;
    bool terminal_zone = false;
    bool terminal_overrun = false;
    bool turn_context = false;
    bool cte_abort_candidate = false;
    bool have_dynamic_drive_heading = false;

    if (nav == 0 || route == 0 || pose == 0 || out == 0 ||
        route->points == 0 || route->count < 2u) {
        return AUTO_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    dynamic_route = auto_route_is_dynamic() ? true : false;
    route_length_cm = route_length_to_index(route, route->count - 1u);
    travelled_cm = absolute_distance_cm(pose->distance_cm);
    if (dynamic_route && route_length_cm > 1.0f &&
        travelled_cm + AUTO_ROUTE_DONE_MARGIN_CM >= route_length_cm) {
        terminal_zone = true;
    }

    /* ── 路径点推进（每帧最多推进一次，避免while循环导致越级） ──────────────
     * 推进条件（满足其一）：
     *   A. 距当前点 <= WAYPOINT_REACHED_CM（到达半径）
     *   B. 车辆在当前段上的投影 t > 0.9（已基本过了该段终点）        */
    if (dynamic_route) {
        route_segment_at_distance(route,
                                  pose->distance_cm,
                                  &segment_index,
                                  &segment_t);
        route_refine_segment_by_pose(route,
                                     segment_index,
                                     pose,
                                     &segment_index,
                                     &segment_t);
        nav->nearest_index = segment_index;
    } else {
        nav->nearest_index = find_progress_index(route, nav->nearest_index, pose);
    }

    dist_to_current = auto_distance_cm(
        pose->x_cm, pose->y_cm,
        route->points[nav->nearest_index].x_cm,
        route->points[nav->nearest_index].y_cm);

    current_wp = &route->points[nav->nearest_index];
    /* 方向判断：FLAG_REVERSE优先；fallback heading_error已禁用（在回环路线上
     * 会把前视目标误判为"在车后方"，导致前后振荡）。
     * 如需倒车段，通过 AUTO_WAYPOINT_FLAG_REVERSE 标志明确指定。 */
    driving_reverse = ((current_wp->flags & AUTO_WAYPOINT_FLAG_REVERSE) != 0u);
    nav->prev_driving_reverse = driving_reverse;

    /* ── 前视点（沿路径积累160cm）────────────────────────────────────────── */
    nav->target_index = find_lookahead_index(route, nav->nearest_index,
                                              AUTO_NAV_LOOKAHEAD_DISTANCE_CM);
    if (!dynamic_route) {
        if (nav->nearest_index + 1u < route->count) {
            segment_index = nav->nearest_index;
        } else if (nav->nearest_index > 0u) {
            segment_index = nav->nearest_index - 1u;
        } else {
            segment_index = 0u;
        }
        segment_start_wp = &route->points[segment_index];
        segment_end_wp = &route->points[segment_index + 1u];
        segment_t = segment_projection_t(segment_start_wp->x_cm,
                                         segment_start_wp->y_cm,
                                         segment_end_wp->x_cm,
                                         segment_end_wp->y_cm,
                                         pose->x_cm,
                                         pose->y_cm);
    } else {
        segment_start_wp = &route->points[segment_index];
        segment_end_wp = &route->points[segment_index + 1u];
        route_project_pose(route, segment_index, pose, &segment_t);
    }
    route_point_ahead(route,
                      segment_index,
                      segment_t,
                      AUTO_NAV_LOOKAHEAD_DISTANCE_CM,
                      &target_x_cm,
                      &target_y_cm);
    segment_heading_deg = route_segment_heading_deg(route, segment_index);
    turn_distance_cm = route_distance_to_next_turn_cm(route,
                                                      segment_index,
                                                      segment_t,
                                                      segment_heading_deg);
    ref_x_cm = segment_start_wp->x_cm +
               (segment_end_wp->x_cm - segment_start_wp->x_cm) * segment_t;
    ref_y_cm = segment_start_wp->y_cm +
               (segment_end_wp->y_cm - segment_start_wp->y_cm) * segment_t;
    path_heading_deg = auto_heading_to_deg(ref_x_cm,
                                           ref_y_cm,
                                           target_x_cm,
                                           target_y_cm);
    if (dynamic_route) {
        target_behind_segment =
            route_target_is_behind_segment(route,
                                           segment_index,
                                           pose,
                                           target_x_cm,
                                           target_y_cm);
    }

    /* ── 横向误差纠偏（叠加到转向，使车辆贴回路径段） ─────────────────────
     * 使用当前路径段（nearest_index-1 → nearest_index）计算横向偏差。   */
    cte       = 0.0f;
    cte_steer = 0.0f;
    steer_feedforward = 0.0f;
    dynamic_drive_heading_deg = 0.0f;
    if (dynamic_route) {
        have_dynamic_drive_heading =
            route_dynamic_drive_heading_ahead_deg(route,
                                                  segment_index,
                                                  segment_t,
                                                  AUTO_DYNAMIC_HEADING_LOOKAHEAD_CM,
                                                  &dynamic_drive_heading_deg);
        if (have_dynamic_drive_heading) {
            target_heading_deg = driving_reverse ?
                auto_normalize_angle_deg(dynamic_drive_heading_deg - 180.0f) :
                dynamic_drive_heading_deg;
        } else {
            target_heading_deg = route_dynamic_heading_deg(segment_heading_deg,
                                                           path_heading_deg,
                                                           turn_distance_cm);
        }
        if (terminal_zone ||
            (target_behind_segment && !have_dynamic_drive_heading)) {
            target_heading_deg = segment_heading_deg;
        }
    } else {
        target_heading_deg = auto_heading_to_deg(pose->x_cm,
                                                 pose->y_cm,
                                                 target_x_cm,
                                                 target_y_cm);
    }
    if (AUTO_CROSS_TRACK_KP > 0.0f) {
        cte = cross_track_error(segment_start_wp->x_cm,
                                segment_start_wp->y_cm,
                                segment_end_wp->x_cm,
                                segment_end_wp->y_cm,
                                pose->x_cm,
                                pose->y_cm);
        if (dynamic_route) {
            cte_steer = auto_clampf(-AUTO_DYNAMIC_CROSS_TRACK_KP * cte,
                                    -AUTO_DYNAMIC_CROSS_TRACK_STEER_LIMIT,
                                    AUTO_DYNAMIC_CROSS_TRACK_STEER_LIMIT);
            target_heading_deg = auto_normalize_angle_deg(
                target_heading_deg +
                auto_clampf(-cte * 57.2957795f /
                            AUTO_DYNAMIC_CROSS_TRACK_HEADING_LOOKAHEAD_CM,
                            -AUTO_DYNAMIC_CROSS_TRACK_HEADING_LIMIT_DEG,
                            AUTO_DYNAMIC_CROSS_TRACK_HEADING_LIMIT_DEG));
        } else {
            cte_steer = auto_clampf(-AUTO_CROSS_TRACK_KP * cte,
                                    -AUTO_CROSS_TRACK_STEER_LIMIT,
                                    AUTO_CROSS_TRACK_STEER_LIMIT);
            target_heading_deg = auto_normalize_angle_deg(
                target_heading_deg +
                auto_clampf(-cte * 57.2957795f /
                            AUTO_CROSS_TRACK_HEADING_LOOKAHEAD_CM,
                            -AUTO_CROSS_TRACK_HEADING_LIMIT_DEG,
                            AUTO_CROSS_TRACK_HEADING_LIMIT_DEG));
        }
    }
    if (driving_reverse) {
        raw_heading_err = auto_normalize_angle_deg(
                              target_heading_deg + 180.0f -
                              pose->heading_deg);
    } else {
        raw_heading_err = auto_normalize_angle_deg(
                              target_heading_deg - pose->heading_deg);
    }
    heading_err = raw_heading_err;
    heading_err = auto_clampf(heading_err,
                              -AUTO_NAV_MAX_HEADING_ERROR_DEG,
                              AUTO_NAV_MAX_HEADING_ERROR_DEG);
    out->heading_error_deg = heading_err;
    heading_kp = AUTO_HEADING_KP;
    if (dynamic_route &&
        auto_absf(heading_err) > AUTO_DYNAMIC_TURN_GAIN_START_DEG) {
        float gain_blend =
            (auto_absf(heading_err) - AUTO_DYNAMIC_TURN_GAIN_START_DEG) /
            (AUTO_NAV_MAX_HEADING_ERROR_DEG -
             AUTO_DYNAMIC_TURN_GAIN_START_DEG);
        gain_blend = auto_clampf(gain_blend, 0.0f, 1.0f);
        heading_kp = AUTO_HEADING_KP +
                     (AUTO_DYNAMIC_TURN_HEADING_KP - AUTO_HEADING_KP) *
                     gain_blend;
    }

    /* ── 转向 + 速度指令 ────────────────────────────────────────────────── */
    if (driving_reverse) {
        reverse_err = heading_err;
        if (auto_absf(reverse_err) < AUTO_STEER_DEAD_DEG) {
            steer_percent = 0.0f;
        } else {
            steer_percent = auto_clampf(-reverse_err * heading_kp,
                                        -AUTO_MAX_STEER_PERCENT,
                                        AUTO_MAX_STEER_PERCENT);
        }
        speed_percent = current_wp->speed_percent;
        if (speed_percent > AUTO_SPEED_MIN_PERCENT) {
            speed_percent = -speed_percent;
        }
        if (speed_percent > -AUTO_SPEED_MIN_PERCENT &&
            speed_percent < AUTO_SPEED_MIN_PERCENT) {
            speed_percent = AUTO_REVERSE_SPEED_PERCENT;
        }
    } else {
        if (auto_absf(heading_err) < AUTO_STEER_DEAD_DEG) {
            steer_percent = 0.0f;
        } else {
            steer_percent = auto_clampf(heading_err * heading_kp,
                                        -AUTO_MAX_STEER_PERCENT,
                                        AUTO_MAX_STEER_PERCENT);
        }
        speed_percent = AUTO_CRUISE_SPEED_PERCENT;
    }

    if (dynamic_route && !terminal_zone) {
        steer_feedforward =
            route_dynamic_steer_feedforward_percent(route,
                                                    segment_index,
                                                    segment_t,
                                                    driving_reverse);
    }
    turn_context =
        dynamic_route &&
        (turn_distance_cm <= AUTO_NAV_LOOKAHEAD_DISTANCE_CM ||
         auto_absf(steer_feedforward) >=
         AUTO_DYNAMIC_STEER_FEEDFORWARD_DEAD_PERCENT ||
         auto_absf(raw_heading_err) >= AUTO_DYNAMIC_TURN_ABORT_START_DEG);

    /* ── 路线完成 ─────────────────────────────────────────────────────────── */
    steer_percent = auto_clampf(steer_percent +
                                steer_feedforward +
                                (driving_reverse ? -cte_steer : cte_steer),
                                -AUTO_MAX_STEER_PERCENT,
                                AUTO_MAX_STEER_PERCENT);
    cte_abort_limit = AUTO_DYNAMIC_ROUTE_ABORT_CTE_CM;
    if (turn_context) {
        cte_abort_limit = AUTO_DYNAMIC_TURN_ABORT_CTE_CM;
        if (cte_abort_limit < AUTO_DYNAMIC_TURN_CONTEXT_CTE_CM) {
            cte_abort_limit = AUTO_DYNAMIC_TURN_CONTEXT_CTE_CM;
        }
    }
    if (dynamic_route &&
        absolute_distance_cm(pose->distance_cm) >=
        AUTO_DYNAMIC_ROUTE_ABORT_AFTER_CM &&
        auto_absf(cte) > cte_abort_limit &&
        !(terminal_zone &&
          auto_absf(cte) <= AUTO_DYNAMIC_ROUTE_DONE_CTE_CM)) {
        cte_abort_candidate = true;
    }
    if (dynamic_route) {
        if (cte_abort_candidate) {
            if (nav->dynamic_cte_abort_ticks <
                AUTO_DYNAMIC_ROUTE_ABORT_CTE_CONFIRM_TICKS) {
                nav->dynamic_cte_abort_ticks++;
            }
            if (auto_absf(cte) >
                AUTO_DYNAMIC_ROUTE_ABORT_CTE_HARD_CM ||
                nav->dynamic_cte_abort_ticks >=
                AUTO_DYNAMIC_ROUTE_ABORT_CTE_CONFIRM_TICKS) {
                safety_stop = true;
            }
        } else {
            nav->dynamic_cte_abort_ticks = 0u;
        }
    } else {
        nav->dynamic_cte_abort_ticks = 0u;
    }
    if (dynamic_route &&
        absolute_distance_cm(pose->distance_cm) >=
        AUTO_DYNAMIC_ROUTE_ABORT_AFTER_CM &&
        auto_absf(raw_heading_err) > AUTO_DYNAMIC_ROUTE_ABORT_HEADING_DEG &&
        auto_absf(cte) > AUTO_DYNAMIC_ROUTE_ABORT_HEADING_MIN_CTE_CM &&
        !(terminal_zone &&
          auto_absf(cte) <= AUTO_DYNAMIC_ROUTE_DONE_CTE_CM)) {
        safety_stop = true;
    }

    if (dynamic_route && terminal_zone) {
        if (auto_absf(cte) <= AUTO_DYNAMIC_ROUTE_DONE_CTE_CM &&
            (target_behind_segment ||
             auto_absf(raw_heading_err) <= AUTO_DYNAMIC_ROUTE_DONE_HEADING_DEG)) {
            distance_done = true;
        } else if (travelled_cm >=
                   route_length_cm + AUTO_DYNAMIC_ROUTE_FINISH_OVERRUN_CM) {
            terminal_overrun = true;
            safety_stop = true;
        }
    }

    if (dynamic_route &&
        terminal_zone &&
        !distance_done &&
        !terminal_overrun &&
        speed_percent > AUTO_DYNAMIC_ROUTE_FINISH_SPEED_PERCENT) {
        speed_percent = AUTO_DYNAMIC_ROUTE_FINISH_SPEED_PERCENT;
    }

    if (dynamic_route) {
        out->route_done = safety_stop || distance_done;
    } else {
        out->route_done = safety_stop ||
                          distance_done ||
                          route_end_reached(route,
                                            pose,
                                            nav->nearest_index,
                                            nav->target_index);
    }

    out->nearest_index       = nav->nearest_index;
    out->target_distance_cm = dist_to_current;
    out->target_index       = nav->target_index;
    out->cross_track_error_cm = cte;
    out->cross_track_steer_percent = cte_steer;
    out->target_x_cm = target_x_cm;
    out->target_y_cm = target_y_cm;
    out->target_heading_deg = driving_reverse ?
        auto_normalize_angle_deg(target_heading_deg + 180.0f) :
        target_heading_deg;
    out->segment_t = segment_t;
    out->cmd.speed_percent  = out->route_done ? 0.0f : speed_percent;
    out->cmd.steer_percent  = out->route_done ? 0.0f : steer_percent;
    out->cmd.brake          = safety_stop;

    nav->initialized = true;
    return AUTO_OK;
}
