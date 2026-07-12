#ifndef AUTO_ROUTE_H
#define AUTO_ROUTE_H

#include <stddef.h>

#include "auto_types.h"

typedef struct {
    const auto_waypoint_t *points;
    size_t count;
} auto_route_t;

const auto_route_t *auto_route_subject1_slalom(void);
const auto_route_t *auto_route_subject1_garage_approach(void);

/* Dynamic route: call before Auto_Start() to override static waypoints.
 * Pass points=NULL / count=0 to revert to static defaults.
 */
void auto_route_set_dynamic(const auto_waypoint_t *points, size_t count);
void auto_route_set_dynamic_headings(const float *headings_deg, size_t count);
bool auto_route_get_dynamic_heading(size_t index, float *heading_deg);
void auto_route_set_dynamic_steers(const float *steers_percent, size_t count);
bool auto_route_get_dynamic_steer(size_t index, float *steer_percent);

/* Returns 1 if a dynamic (recorded) route is active, 0 if using static route */
int auto_route_is_dynamic(void);

#endif
