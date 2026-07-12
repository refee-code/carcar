#include "auto_screen_seekfree.h"

#include <stdbool.h>
#include <string.h>

#include "auto_config.h"
#include "zf_common_headfile.h"

static bool s_screen_initialized = false;
static uint32_t s_last_render_ms = 0u;
static char s_last_lines[6][AUTO_SCREEN_MAX_TEXT_CHARS + 1u];

static void auto_screen_pad_line(char *line, unsigned line_size)
{
    unsigned len;
    unsigned i;
    unsigned max_chars;

    if (line == 0 || line_size == 0u) {
        return;
    }

    max_chars = AUTO_SCREEN_MAX_TEXT_CHARS;
    if (max_chars >= line_size) {
        max_chars = line_size - 1u;
    }

    len = (unsigned)strlen(line);
    if (len > max_chars) {
        line[max_chars] = '\0';
        len = max_chars;
    }

    for (i = len; i < max_chars; ++i) {
        line[i] = ' ';
    }
    line[max_chars] = '\0';
}

static void auto_screen_clear(void *ctx)
{
    (void)ctx;

#if AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_SPI || \
    AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_PAR8
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_clear();
#elif AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS114
    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
    ips114_clear();
#elif AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_TFT180
    tft180_set_color(RGB565_BLACK, RGB565_WHITE);
    tft180_clear();
#elif AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_OLED
    oled_clear();
#else
    (void)0;
#endif

    memset(s_last_lines, 0, sizeof(s_last_lines));
}

static void auto_screen_draw_text(void *ctx,
                                  unsigned row,
                                  unsigned col,
                                  const char *text)
{
    uint16 x;
    uint16 y;

    (void)ctx;

    x = (uint16)(col * AUTO_SCREEN_CHAR_WIDTH);
    y = (uint16)(row * AUTO_SCREEN_LINE_HEIGHT);

#if AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_SPI || \
    AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_PAR8
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(x, y, text);
#elif AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS114
    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
    ips114_show_string(x, y, text);
#elif AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_TFT180
    tft180_set_color(RGB565_BLACK, RGB565_WHITE);
    tft180_show_string(x, y, text);
#elif AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_OLED
    oled_show_string(x, y, text);
#else
    (void)text;
#endif
}

static const auto_diag_display_t s_auto_display = {
    0,
    auto_screen_clear,
    auto_screen_draw_text
};

void auto_screen_seekfree_init(void)
{
    if (s_screen_initialized) {
        return;
    }

#if AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_SPI
    ips200_init(IPS200_TYPE_SPI);
#elif AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_PAR8
    ips200_init(IPS200_TYPE_PARALLEL8);
#elif AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS114
    ips114_init();
#elif AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_TFT180
    tft180_init();
#elif AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_OLED
    oled_init();
#else
    (void)0;
#endif

    s_screen_initialized = true;
    auto_screen_clear(0);
    auto_screen_draw_text(0, 0u, 0u, "BOOT OK");
}

const auto_diag_display_t *auto_screen_seekfree_display(void)
{
    return &s_auto_display;
}

void auto_screen_seekfree_render_diag(const auto_diag_t *diag)
{
    char line[AUTO_SCREEN_MAX_TEXT_CHARS + 1u];
    unsigned i;

    auto_screen_seekfree_init();

#if AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_SPI || \
    AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_PAR8
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
#elif AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS114
    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
#elif AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_TFT180
    tft180_set_color(RGB565_BLACK, RGB565_WHITE);
#endif

    for (i = 0u; i < 6u; ++i) {
        auto_diag_format_line(diag, i, line, sizeof(line));
        auto_screen_pad_line(line, sizeof(line));
        if (strncmp(s_last_lines[i], line, sizeof(s_last_lines[i])) != 0) {
            auto_screen_draw_text(0, i, 0u, line);
            strncpy(s_last_lines[i], line, sizeof(s_last_lines[i]) - 1u);
            s_last_lines[i][sizeof(s_last_lines[i]) - 1u] = '\0';
        }
    }
}

void auto_screen_seekfree_render_lines(const char lines[][32], unsigned line_count)
{
    char line[AUTO_SCREEN_MAX_TEXT_CHARS + 1u];
    unsigned i;

    auto_screen_seekfree_init();

    for (i = 0u; i < 6u; ++i) {
        if (lines != 0 && i < line_count) {
            strncpy(line, lines[i], sizeof(line) - 1u);
            line[sizeof(line) - 1u] = '\0';
        } else {
            line[0] = '\0';
        }

        auto_screen_pad_line(line, sizeof(line));
        if (strncmp(s_last_lines[i], line, sizeof(s_last_lines[i])) != 0) {
            auto_screen_draw_text(0, i, 0u, line);
            strncpy(s_last_lines[i], line, sizeof(s_last_lines[i]) - 1u);
            s_last_lines[i][sizeof(s_last_lines[i]) - 1u] = '\0';
        }
    }
}

void auto_screen_seekfree_force_clear(void)
{
    auto_screen_seekfree_init();
    auto_screen_clear(0);
}

void auto_screen_seekfree_debug_line(unsigned row, const char *text)
{
    char line[AUTO_SCREEN_MAX_TEXT_CHARS + 1u];

    auto_screen_seekfree_init();
    if (text == 0) {
        text = "";
    }

    strncpy(line, text, sizeof(line) - 1u);
    line[sizeof(line) - 1u] = '\0';
    auto_screen_pad_line(line, sizeof(line));
    auto_screen_draw_text(0, row, 0u, line);
}

void auto_screen_seekfree_render_periodic(const auto_diag_t *diag, uint32_t now_ms)
{
    if ((now_ms - s_last_render_ms) < AUTO_SCREEN_REFRESH_MS) {
        return;
    }

    s_last_render_ms = now_ms;
    auto_screen_seekfree_render_diag(diag);
}

/* ═══════════════════════════════════════════════════════════════════
 * 路径地图显示
 * 屏幕布局（IPS200 竖屏 240×320）：
 *   行0-2  (y 0~47):   状态文字（航向、里程、点数）
 *   地图区 (y 50~249): 200×200 像素路径地图
 *   行6-7  (y 260+):   底部状态文字
 * ═══════════════════════════════════════════════════════════════════ */
#define MAP_LEFT    (20)
#define MAP_TOP     (80)
#define MAP_SIZE    (176)
#define MAP_RIGHT   (MAP_LEFT + MAP_SIZE)
#define MAP_BOTTOM  (MAP_TOP  + MAP_SIZE)

#define COLOR_BLACK   (0x0000u)
#define COLOR_WHITE   (0xFFFFu)
#define COLOR_RED     (0xF800u)
#define COLOR_GRAY    (0x7BEFu)
#define COLOR_YELLOW  (0xFFE0u)
#define COLOR_CYAN    (0x07FFu)

static bool  s_map_drawn        = false;
static int16 s_prev_car_sx      = -1;
static int16 s_prev_car_sy      = -1;

/* 地图缩放状态（首次绘路径时计算） */
static float s_map_min_x   = 0.0f;
static float s_map_min_y   = 0.0f;
static float s_map_scale   = 1.0f;  /* pixels / cm */

static int16 world_to_screen_x(float x_cm)
{
    int16 sx = (int16)(MAP_LEFT + (x_cm - s_map_min_x) * s_map_scale);
    if (sx < MAP_LEFT)  sx = MAP_LEFT;
    if (sx > MAP_RIGHT) sx = (int16)MAP_RIGHT;
    return sx;
}

static int16 world_to_screen_y(float y_cm)
{
    /* 翻转 Y：坐标系 Y 朝上，屏幕 Y 朝下 */
    int16 sy = (int16)(MAP_TOP + (y_cm - s_map_min_y) * s_map_scale);
    if (sy < MAP_TOP)    sy = MAP_TOP;
    if (sy > MAP_BOTTOM) sy = (int16)MAP_BOTTOM;
    return sy;
}

static void map_draw_point(int16 sx, int16 sy, uint16 color)
{
#if AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_SPI || \
    AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_PAR8
    int16 dx, dy;
    for (dx = -1; dx <= 1; dx++) {
        for (dy = -1; dy <= 1; dy++) {
            int16 px = (int16)(sx + dx);
            int16 py = (int16)(sy + dy);
            if (px >= MAP_LEFT && px <= MAP_RIGHT &&
                py >= MAP_TOP  && py <= MAP_BOTTOM) {
                ips200_draw_point((uint16)px, (uint16)py, color);
            }
        }
    }
#else
    (void)sx; (void)sy; (void)color;
#endif
}

static void map_draw_car(int16 sx, int16 sy, uint16 color)
{
    /* 5 像素半径填充圆 */
#if AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_SPI || \
    AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_PAR8
    int16 dx, dy;
    for (dx = -4; dx <= 4; dx++) {
        for (dy = -4; dy <= 4; dy++) {
            if (dx*dx + dy*dy <= 16) {
                int16 px = (int16)(sx + dx);
                int16 py = (int16)(sy + dy);
                if (px >= MAP_LEFT && px <= MAP_RIGHT &&
                    py >= MAP_TOP  && py <= MAP_BOTTOM) {
                    ips200_draw_point((uint16)px, (uint16)py, color);
                }
            }
        }
    }
#else
    (void)sx; (void)sy; (void)color;
#endif
}

void auto_screen_seekfree_map_reset(void)
{
    s_map_drawn   = false;
    s_prev_car_sx = -1;
    s_prev_car_sy = -1;
}

void auto_screen_seekfree_render_trace(const auto_waypoint_t *points,
                                       size_t count)
{
    size_t i;

    if (points == 0 || count == 0u || !s_map_drawn) {
        return;
    }

    for (i = 0u; i < count; ++i) {
        int16 sx = world_to_screen_x(points[i].x_cm);
        int16 sy = world_to_screen_y(points[i].y_cm);
        map_draw_point(sx, sy, COLOR_CYAN);

        if (i + 1u < count) {
            int16 sx2 = world_to_screen_x(points[i + 1u].x_cm);
            int16 sy2 = world_to_screen_y(points[i + 1u].y_cm);
#if AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_SPI || \
    AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_PAR8
            ips200_draw_line((uint16)sx, (uint16)sy,
                             (uint16)sx2, (uint16)sy2, COLOR_CYAN);
#endif
        }
    }
}

void auto_screen_seekfree_render_map(const auto_waypoint_t *points,
                                     size_t count,
                                     float car_x,
                                     float car_y)
{
    size_t i;

    if (points == 0 || count == 0u) {
        return;
    }

    auto_screen_seekfree_init();

    if (!s_map_drawn) {
        float min_x, max_x, min_y, max_y, range;
        char hdr[32];

        /* ── 计算缩放比例 ─────────────────────────── */
        min_x = points[0].x_cm;  max_x = points[0].x_cm;
        min_y = points[0].y_cm;  max_y = points[0].y_cm;
        for (i = 1u; i < count; ++i) {
            if (points[i].x_cm < min_x) min_x = points[i].x_cm;
            if (points[i].x_cm > max_x) max_x = points[i].x_cm;
            if (points[i].y_cm < min_y) min_y = points[i].y_cm;
            if (points[i].y_cm > max_y) max_y = points[i].y_cm;
        }
        /* 留 10% 边距 */
        min_x -= (max_x - min_x) * 0.05f;
        min_y -= (max_y - min_y) * 0.05f;
        range = (max_x - min_x);
        if ((max_y - min_y) > range) range = (max_y - min_y);
        if (range < 50.0f) range = 50.0f;  /* 最小显示范围 50cm */

        s_map_min_x = min_x;
        s_map_min_y = min_y;
        s_map_scale = (float)(MAP_SIZE - 4) / range;

        /* ── 清屏，绘标题文字 ─────────────────────── */
#if AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_SPI || \
    AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_PAR8
        ips200_clear();
#endif
        snprintf(hdr, sizeof(hdr), "LUJING: %lu DIAN", (unsigned long)count);
        auto_screen_draw_text(0, 0, 0u, hdr);
        auto_screen_draw_text(0, 1, 0u, "ZIDONG JIASHIZHONG");

        /* ── 绘制地图边框 ─────────────────────────── */
#if AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_SPI || \
    AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_PAR8
        ips200_draw_line(MAP_LEFT,  MAP_TOP,    MAP_RIGHT, MAP_TOP,    COLOR_GRAY);
        ips200_draw_line(MAP_LEFT,  MAP_BOTTOM, MAP_RIGHT, MAP_BOTTOM, COLOR_GRAY);
        ips200_draw_line(MAP_LEFT,  MAP_TOP,    MAP_LEFT,  MAP_BOTTOM, COLOR_GRAY);
        ips200_draw_line(MAP_RIGHT, MAP_TOP,    MAP_RIGHT, MAP_BOTTOM, COLOR_GRAY);
#endif

        /* ── 绘制路径：点 + 相邻点之间的连线 ─────── */
        for (i = 0u; i < count; ++i) {
            int16 sx = world_to_screen_x(points[i].x_cm);
            int16 sy = world_to_screen_y(points[i].y_cm);
            map_draw_point(sx, sy, COLOR_WHITE);

            if (i + 1u < count) {
                int16 sx2 = world_to_screen_x(points[i + 1u].x_cm);
                int16 sy2 = world_to_screen_y(points[i + 1u].y_cm);
#if AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_SPI || \
    AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_PAR8
                ips200_draw_line((uint16)sx, (uint16)sy,
                                 (uint16)sx2, (uint16)sy2, COLOR_GRAY);
#endif
            }
        }

        /* ── 标记起点（黄色）和终点（绿色） ─────── */
        {
            int16 s0x = world_to_screen_x(points[0].x_cm);
            int16 s0y = world_to_screen_y(points[0].y_cm);
            int16 sex = world_to_screen_x(points[count - 1u].x_cm);
            int16 sey = world_to_screen_y(points[count - 1u].y_cm);
            map_draw_point(s0x, s0y, COLOR_YELLOW);      /* 起点黄色 */
            map_draw_point(sex, sey, (uint16)0x07E0u);   /* 终点绿色 */
        }

        s_map_drawn   = true;
        s_prev_car_sx = -1;
        s_prev_car_sy = -1;
    }

    /* ── 更新车辆位置（红点） ──────────────────────── */
    {
        int16 car_sx = world_to_screen_x(car_x);
        int16 car_sy = world_to_screen_y(car_y);

        if (car_sx != s_prev_car_sx || car_sy != s_prev_car_sy) {
            /* 擦除旧红点：用白色恢复背景（之前用黑色会留下黑色轨迹残影） */
            if (s_prev_car_sx >= 0) {
                map_draw_car(s_prev_car_sx, s_prev_car_sy, COLOR_WHITE);

                /* 重绘被红点覆盖的路径线段和路径点 */
                for (i = 0u; i < count; ++i) {
                    int16 px = world_to_screen_x(points[i].x_cm);
                    int16 py = world_to_screen_y(points[i].y_cm);
                    int16 ddx = (int16)(px - s_prev_car_sx);
                    int16 ddy = (int16)(py - s_prev_car_sy);
                    if (ddx < 0) ddx = (int16)-ddx;
                    if (ddy < 0) ddy = (int16)-ddy;
                    if (ddx <= 6 && ddy <= 6) {
                        /* 重绘该点及其连线 */
                        map_draw_point(px, py, COLOR_WHITE);
                        if (i + 1u < count) {
                            int16 px2 = world_to_screen_x(points[i + 1u].x_cm);
                            int16 py2 = world_to_screen_y(points[i + 1u].y_cm);
#if AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_SPI || \
    AUTO_SCREEN_DEVICE == AUTO_SCREEN_DEVICE_IPS200_PAR8
                            ips200_draw_line((uint16)px, (uint16)py,
                                             (uint16)px2, (uint16)py2, COLOR_GRAY);
#endif
                        }
                    }
                }
            }

            /* 绘制新红点 */
            map_draw_car(car_sx, car_sy, COLOR_RED);
            s_prev_car_sx = car_sx;
            s_prev_car_sy = car_sy;
        }
    }
}
