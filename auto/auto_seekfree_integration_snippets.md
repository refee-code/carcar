# 逐飞工程接入片段

这些片段用于复制到实际逐飞工程中，不作为 `.c` 文件直接编译。

## cpu0_main.c 接入

在 `core0_main()` 中按下面方式接入：

```c
#include "auto.h"
#include "zf_common_headfile.h"

int core0_main(void)
{
    clock_init();
    debug_init();

    auto_seekfree_runtime_init();
    auto_seekfree_runtime_start();

    pit_ms_init(CCU60_CH0, AUTO_CONTROL_PERIOD_MS);

    cpu_wait_event_ready();

    while (TRUE) {
        auto_seekfree_runtime_loop();
        system_delay_ms(20);
    }
}
```

## isr.c 接入

在 `CCU60_CH0` 的 PIT 中断中，`pit_clear_flag(CCU60_CH0);` 后调用：

```c
(void)auto_seekfree_runtime_update10ms();
```

完整形态类似：

```c
IFX_INTERRUPT(cc60_pit_ch0_isr, CCU6_0_CH0_INT_VECTAB_NUM, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);

    (void)auto_seekfree_runtime_update10ms();
}
```

不要同时调用手动驾驶的 `Unified_Control_Update()`，否则手动控制和自动控制会同时抢底盘输出。

