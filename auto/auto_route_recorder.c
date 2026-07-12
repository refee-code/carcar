#include "auto_route_recorder.h"

#include <stdio.h>
#include <string.h>

static const auto_recorded_waypoint_t *last_point(const auto_route_recorder_t *recorder)
{
    if (recorder == 0 || recorder->point_count == 0u) {
        return 0;
    }

    return &recorder->points[recorder->point_count - 1u];
}

void auto_route_recorder_init(auto_route_recorder_t *recorder)
{
    if (recorder == 0) {
        return;
    }

    memset(recorder, 0, sizeof(*recorder));
}

void auto_route_recorder_begin(auto_route_recorder_t *recorder)
{
    if (recorder == 0) {
        return;
    }

    recorder->active = true;
    recorder->dirty = true;
    /* printf removed: called from ISR, UART printf blocks and causes Trap */
}

void auto_route_recorder_end(auto_route_recorder_t *recorder)
{
    if (recorder == 0) {
        return;
    }

    recorder->active = false;
    recorder->dirty = true;
    /* printf/print_c_array removed: called from ISR context */
}

void auto_route_recorder_clear(auto_route_recorder_t *recorder)
{
    if (recorder == 0) {
        return;
    }

    recorder->point_count = 0u;
    recorder->dirty = true;
    /* printf removed: ISR context */
}

void auto_route_recorder_undo(auto_route_recorder_t *recorder)
{
    if (recorder == 0 || recorder->point_count == 0u) {
        return;
    }

    recorder->point_count--;
    recorder->dirty = true;
    /* printf removed: ISR context */
}

void auto_route_recorder_capture(auto_route_recorder_t *recorder,
                                 const auto_pose_t *pose,
                                 float speed_percent,
                                 uint8_t flags)
{
    auto_recorded_waypoint_t *point;

    if (recorder == 0 || pose == 0 || recorder->point_count >= AUTO_ROUTE_RECORDER_MAX_POINTS) {
        return;
    }

    point = &recorder->points[recorder->point_count];
    point->pose = *pose;
    point->speed_percent = speed_percent;
    point->flags = flags;
    recorder->point_count++;
    recorder->dirty = true;
    /* printf removed: this function is called from ISR (KEY2 handler).
     * UART printf is blocking and causes Trap when TX buffer is full. */
}

bool auto_route_recorder_is_active(const auto_route_recorder_t *recorder)
{
    return recorder != 0 && recorder->active;
}

bool auto_route_recorder_take_dirty(auto_route_recorder_t *recorder)
{
    bool dirty;

    if (recorder == 0) {
        return false;
    }

    dirty = recorder->dirty;
    recorder->dirty = false;
    return dirty;
}

void auto_route_recorder_format_line(const auto_route_recorder_t *recorder,
                                     const auto_pose_t *live_pose,
                                     unsigned line_index,
                                     char *buffer,
                                     size_t buffer_size)
{
    const auto_recorded_waypoint_t *last;

    if (buffer == 0 || buffer_size == 0u) {
        return;
    }

    if (recorder == 0) {
        snprintf(buffer, buffer_size, "REC null");
        return;
    }

    last = last_point(recorder);

    switch (line_index) {
    case 0u:
        snprintf(buffer, buffer_size, "CAIDIAN DIAN:%lu/%u",
                 (unsigned long)recorder->point_count,
                 AUTO_ROUTE_RECORDER_MAX_POINTS);
        break;
    case 1u:
        snprintf(buffer, buffer_size, "ZIDONG CAIJI ZHONG");
        break;
    case 2u:
        snprintf(buffer, buffer_size, "K1:JIESHU K3:CHEXIAO");
        break;
    case 3u:
        if (live_pose != 0) {
            snprintf(buffer, buffer_size, "X:%.0fcm Y:%.0fcm",
                     live_pose->x_cm,
                     live_pose->y_cm);
        } else {
            snprintf(buffer, buffer_size, "WEIZHI: --");
        }
        break;
    case 4u:
        if (live_pose != 0) {
            snprintf(buffer, buffer_size, "HX:%.1f JL:%.0fcm",
                     live_pose->heading_deg,
                     live_pose->distance_cm);
        } else {
            snprintf(buffer, buffer_size, "HX:-- JL:--");
        }
        break;
    case 5u:
        if (last != 0) {
            snprintf(buffer, buffer_size, "LAST X%.0f Y%.0f",
                     last->pose.x_cm,
                     last->pose.y_cm);
        } else {
            snprintf(buffer, buffer_size, "DENGDAI BAOCUN");
        }
        break;
    default:
        snprintf(buffer, buffer_size, "");
        break;
    }
}

void auto_route_recorder_print_c_array(const auto_route_recorder_t *recorder)
{
    uint32_t i;

    if (recorder == 0) {
        return;
    }

    printf("static const auto_waypoint_t s_recorded_points[] = {\r\n");
    for (i = 0u; i < recorder->point_count; ++i) {
        const auto_recorded_waypoint_t *point = &recorder->points[i];
        printf("    { %.1ff, %.1ff, %.1ff, %u }, /* %02lu hdg %.1f dst %.1f */\r\n",
               point->pose.x_cm,
               point->pose.y_cm,
               point->speed_percent,
               point->flags,
               (unsigned long)(i + 1u),
               point->pose.heading_deg,
               point->pose.distance_cm);
    }
    printf("};\r\n");
}
