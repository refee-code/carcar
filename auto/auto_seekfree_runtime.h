#ifndef AUTO_SEEKFREE_RUNTIME_H
#define AUTO_SEEKFREE_RUNTIME_H

#include "auto_app.h"

auto_status_t auto_seekfree_runtime_init(void);
auto_status_t auto_seekfree_runtime_start(void);
auto_status_t auto_seekfree_runtime_stop(void);
auto_status_t auto_seekfree_runtime_update10ms(void);
void auto_seekfree_runtime_loop(void);
auto_app_t *auto_seekfree_runtime_app(void);

#endif
