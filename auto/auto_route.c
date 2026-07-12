#include "auto_route.h"

#include "auto_config.h"

/* Placeholder route in centimeters.
 * Replace these with field measurements or recorded waypoints.
 * Coordinate convention: start is (0,0), +X forward from the start line.
 */
static const auto_waypoint_t s_slalom_points[] = {
    {   0.0f,    0.0f, AUTO_SLOW_SPEED_PERCENT,   AUTO_WAYPOINT_FLAG_NONE },
    { 250.0f,    0.0f, AUTO_CRUISE_SPEED_PERCENT, AUTO_WAYPOINT_FLAG_NONE },
    { 430.0f,  110.0f, AUTO_SLOW_SPEED_PERCENT,   AUTO_WAYPOINT_FLAG_SLOW },
    { 610.0f, -110.0f, AUTO_SLOW_SPEED_PERCENT,   AUTO_WAYPOINT_FLAG_SLOW },
    { 790.0f,  110.0f, AUTO_SLOW_SPEED_PERCENT,   AUTO_WAYPOINT_FLAG_SLOW },
    { 970.0f, -110.0f, AUTO_SLOW_SPEED_PERCENT,   AUTO_WAYPOINT_FLAG_SLOW },
    {1160.0f,    0.0f, AUTO_CRUISE_SPEED_PERCENT, AUTO_WAYPOINT_FLAG_NONE },
    {1400.0f,    0.0f, AUTO_CRUISE_SPEED_PERCENT, AUTO_WAYPOINT_FLAG_NONE },
};

static const auto_waypoint_t s_garage_approach_points[] = {
    {1550.0f,    0.0f, AUTO_SLOW_SPEED_PERCENT, AUTO_WAYPOINT_FLAG_NONE },
    {1700.0f,  -80.0f, AUTO_SLOW_SPEED_PERCENT, AUTO_WAYPOINT_FLAG_NONE },
    {1850.0f, -150.0f, AUTO_SLOW_SPEED_PERCENT, AUTO_WAYPOINT_FLAG_GARAGE },
};

static const auto_route_t s_slalom_route = {
    s_slalom_points,
    sizeof(s_slalom_points) / sizeof(s_slalom_points[0])
};

static const auto_route_t s_garage_route = {
    s_garage_approach_points,
    sizeof(s_garage_approach_points) / sizeof(s_garage_approach_points[0])
};

/* Dynamic route override - set via auto_route_set_dynamic() */
static auto_route_t s_dynamic_route = { 0, 0 };
static const float *s_dynamic_headings_deg = 0;
static size_t s_dynamic_heading_count = 0u;
static const float *s_dynamic_steers_percent = 0;
static size_t s_dynamic_steer_count = 0u;
static bool s_dynamic_active = false;

void auto_route_set_dynamic(const auto_waypoint_t *points, size_t count)
{
    s_dynamic_headings_deg = 0;
    s_dynamic_heading_count = 0u;
    s_dynamic_steers_percent = 0;
    s_dynamic_steer_count = 0u;

    if (points != 0 && count > 0) {
        s_dynamic_route.points = points;
        s_dynamic_route.count = count;
        s_dynamic_active = true;
    } else {
        s_dynamic_route.points = 0;
        s_dynamic_route.count = 0;
        s_dynamic_active = false;
    }
}

void auto_route_set_dynamic_steers(const float *steers_percent, size_t count)
{
    if (s_dynamic_active &&
        steers_percent != 0 &&
        count == s_dynamic_route.count) {
        s_dynamic_steers_percent = steers_percent;
        s_dynamic_steer_count = count;
    } else {
        s_dynamic_steers_percent = 0;
        s_dynamic_steer_count = 0u;
    }
}

void auto_route_set_dynamic_headings(const float *headings_deg, size_t count)
{
    if (s_dynamic_active &&
        headings_deg != 0 &&
        count == s_dynamic_route.count) {
        s_dynamic_headings_deg = headings_deg;
        s_dynamic_heading_count = count;
    } else {
        s_dynamic_headings_deg = 0;
        s_dynamic_heading_count = 0u;
    }
}

bool auto_route_get_dynamic_heading(size_t index, float *heading_deg)
{
    if (!s_dynamic_active ||
        s_dynamic_headings_deg == 0 ||
        heading_deg == 0 ||
        index >= s_dynamic_heading_count) {
        return false;
    }

    *heading_deg = s_dynamic_headings_deg[index];
    return true;
}

bool auto_route_get_dynamic_steer(size_t index, float *steer_percent)
{
    if (!s_dynamic_active ||
        s_dynamic_steers_percent == 0 ||
        steer_percent == 0 ||
        index >= s_dynamic_steer_count) {
        return false;
    }

    *steer_percent = s_dynamic_steers_percent[index];
    return true;
}

int auto_route_is_dynamic(void)
{
    return s_dynamic_active ? 1 : 0;
}

const auto_route_t *auto_route_subject1_slalom(void)
{
    if (s_dynamic_active && s_dynamic_route.count > 0) {
        return &s_dynamic_route;
    }
    return &s_slalom_route;
}

const auto_route_t *auto_route_subject1_garage_approach(void)
{
    /* Garage approach always uses static route for now */
    return &s_garage_route;
}
