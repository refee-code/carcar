#ifndef AUTO_DIAG_H
#define AUTO_DIAG_H

#include <stddef.h>

#include "auto_types.h"

typedef struct {
    void *ctx;
    void (*clear)(void *ctx);
    void (*draw_text)(void *ctx, unsigned row, unsigned col, const char *text);
} auto_diag_display_t;

const char *auto_diag_state_name(auto_subject1_state_t state);
const char *auto_diag_status_name(auto_status_t status);
void auto_diag_format_line(const auto_diag_t *diag,
                           unsigned line_index,
                           char *buffer,
                           size_t buffer_size);
void auto_diag_render(const auto_diag_display_t *display,
                      const auto_diag_t *diag);

#endif
