#ifndef AUTO_SCREEN_SEEKFREE_H
#define AUTO_SCREEN_SEEKFREE_H

#include <stdint.h>
#include <stddef.h>

#include "auto_diag.h"
#include "auto_types.h"

void auto_screen_seekfree_init(void);
const auto_diag_display_t *auto_screen_seekfree_display(void);
void auto_screen_seekfree_render_diag(const auto_diag_t *diag);
void auto_screen_seekfree_render_lines(const char lines[][32], unsigned line_count);
void auto_screen_seekfree_render_periodic(const auto_diag_t *diag, uint32_t now_ms);
void auto_screen_seekfree_force_clear(void);
void auto_screen_seekfree_debug_line(unsigned row, const char *text);

/* 路径地图：在屏幕上绘制录制的路径点和车辆当前位置（红点）。
 * 首次调用会清屏并绘制整条路径；后续调用只更新车辆位置。
 * 参数: points/count - 已录路径点数组；car_x/car_y - 车辆当前坐标(cm)。 */
void auto_screen_seekfree_render_map(const auto_waypoint_t *points,
                                     size_t count,
                                     float car_x,
                                     float car_y);
void auto_screen_seekfree_render_trace(const auto_waypoint_t *points,
                                       size_t count);

/* 重置地图缓存（切换到地图模式前调用，强制重绘路径） */
void auto_screen_seekfree_map_reset(void);

#endif
