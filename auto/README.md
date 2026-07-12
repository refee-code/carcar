# auto 自动驾驶模块

这是卡丁快跑组科目一的独立自动驾驶框架。

本目录刻意和 `E3_04_drv8701e_double_motor_contro_demo` 手动驾驶工程隔离。`auto` 核心代码不包含逐飞库或英飞凌库头文件，所有硬件访问都通过 `auto_platform_t` 平台接口注入，后续集成时只需要把底盘、IMU、GPS、编码器等实际函数接进来。

## 当前范围

- 不使用摄像头。
- 使用预设路线点完成绕桩路线跟踪。
- 使用预设车库入口路线点接近倒车入库区域。
- 使用分段动作完成倒车入库。
- 默认要求 IMU 航向角有效。
- GPS 和编码器在 `auto_config.h` 中默认暂时设为可选，方便先做台架和底盘联调。

## 目录职责

- `auto_app.c/.h`：自动驾驶总入口，负责初始化、启动、停止和 10ms 周期更新。
- `auto_task_subject1.c/.h`：科目一状态机，串联发车、绕桩、车库接近、姿态对准、倒车入库和停车。
- `auto_route.c/.h`：预设路线点，目前是占位点，需要按实测场地修改。
- `auto_route_recorder.c/.h`：推车采点模块，按键保存路线点并通过串口打印。
- `auto_nav.c/.h`：路线点跟踪，根据车辆位姿计算速度和转向指令。
- `auto_pose.c/.h`：车辆位姿估计，融合 IMU、GPS 和编码器输入。
- `auto_parking.c/.h`：倒车入库分段动作。
- `auto_chassis.c/.h`：底盘输出封装，统一限幅和刹车处理。
- `auto_safety.c/.h`：急停、传感器超时和路线超时保护。
- `auto_platform.c/.h`：平台端口表，隔离硬件依赖。
- `auto_diag.c/.h`：屏幕诊断显示辅助，负责把状态快照格式化成多行文本。
- `auto_screen_seekfree.c/.h`：逐飞屏幕直接适配层，默认使用 IPS200 SPI。
- `auto_seekfree_port.c/.h`：逐飞平台适配层，连接底盘、转向、可选 IMU/GNSS/编码器。
- `auto_seekfree_runtime.c/.h`：逐飞运行封装，提供初始化、启动、10ms 更新和屏幕刷新入口。
- `auto_config.h`：调参集中入口。
- `auto_types.h`：公共类型、状态和错误码。

## 集成入口

在拥有硬件驱动的主工程中创建一个自动驾驶实例：

```c
static auto_app_t g_auto;

Auto_Init(&g_auto, &platform);
Auto_Start(&g_auto);

/* 每 10ms 调用一次 */
Auto_Update10ms(&g_auto);
```

如果按逐飞例程直接集成，推荐使用封装后的入口：

```c
auto_seekfree_runtime_init();
pit_ms_init(CCU60_CH0, AUTO_CONTROL_PERIOD_MS);
```

当前为了推车采点安全，逐飞工程上电只初始化，不自动调用 `auto_seekfree_runtime_start()`。

在 `CCU60_CH0` 的 10ms 中断里调用：

```c
auto_seekfree_runtime_update10ms();
```

在主循环里刷新屏幕：

```c
auto_seekfree_runtime_loop();
system_delay_ms(20);
```

如果车上有屏幕，建议每 100ms 刷新一次诊断信息：

```c
auto_diag_t diag;
Auto_GetDiag(&g_auto, &diag);
auto_screen_seekfree_render_periodic(&diag, system_getval_ms());
```

`platform` 至少需要实现：

- `now_ms`
- `set_drive_percent`
- `set_steer_percent`
- `set_brake`

推荐继续接入：

- `read_imu_heading_deg`
- `read_gps_lat_lon`
- `read_encoder_distance_cm`
- `is_emergency_stop`

屏幕适配只需要实现：

- `clear`
- `draw_text`

如果使用逐飞屏幕，已经提供了直接适配层，初始化时调用：

```c
auto_screen_seekfree_init();
```

主循环里周期刷新：

```c
auto_diag_t diag;
Auto_GetDiag(&g_auto, &diag);
auto_screen_seekfree_render_periodic(&diag, system_getval_ms());
```

默认屏幕是 IPS200 SPI。如果不是这个屏幕，在 `auto_config.h` 里修改：

```c
#define AUTO_SCREEN_DEVICE AUTO_SCREEN_DEVICE_IPS200_SPI
```

可选值：

- `AUTO_SCREEN_DEVICE_IPS200_SPI`
- `AUTO_SCREEN_DEVICE_IPS200_PAR8`
- `AUTO_SCREEN_DEVICE_IPS114`
- `AUTO_SCREEN_DEVICE_TFT180`
- `AUTO_SCREEN_DEVICE_OLED`
- `AUTO_SCREEN_DEVICE_NONE`

## 逐飞硬件开关

`auto_seekfree_port.c` 已经把底盘、转向、屏幕接好。IMU、GNSS、编码器先默认关闭，避免未接线时影响编译和调试。

在 `auto_config.h` 中按实际硬件打开：

```c
#define AUTO_SEEKFREE_USE_IMU660RA 1u
#define AUTO_SEEKFREE_USE_GNSS     1u
#define AUTO_SEEKFREE_USE_ENCODER  1u
```

只打开实际存在的模块。编码器默认引脚只是示例，启用前必须改成实际接线：

```c
#define AUTO_SEEKFREE_ENCODER_INDEX TIM5_ENCODER
#define AUTO_SEEKFREE_ENCODER_A_PIN TIM5_ENCODER_CH1_P10_3
#define AUTO_SEEKFREE_ENCODER_B_PIN TIM5_ENCODER_CH2_P10_1
```

## 调试顺序

1. 先只接底盘输出和急停，确认自动模块能安全停车。
2. 接入 IMU 航向角，确认角度正方向和 `auto_route.c` 坐标系一致。
3. 接入编码器距离，确认前进时距离增加。
4. 替换 `auto_route.c` 里的占位路线点。
5. 调整 `auto_config.h` 中的速度、前视距离和转向增益。
6. 调整 `auto_parking.c` 中的倒车入库分段距离和转向角。
7. 最后再启用 GPS/编码器强制有效检查。

## 推车采点模式

车上 4 个按键已经接入采点功能。进入采点模式后，电机输出会被持续置零，只更新姿态和里程，适合手推小车沿路线走一遍。

当前自动驾驶工程上电后只初始化，不会自动起跑。未进入采点、未主动启动自动任务时，底盘保持零输出。

按键分配：

- 非采点时，屏幕有 `CAIDIAN` 和 `ZIDONG` 两项，`>` 为当前光标。`CAIDIAN` 表示采点，`ZIDONG` 表示自动驾驶。
- 非采点时 `KEY2`：光标放到 `CAIDIAN`。
- 非采点时 `KEY3`：光标放到 `ZIDONG`。
- 非采点时 `KEY1`：执行当前光标项。光标在 `CAIDIAN` 时开始采点，光标在 `ZIDONG` 时开始自动驾驶。
- 采点中 `KEY1`：结束采点，并把光标自动放到 `ZIDONG`。
- 采点中 `KEY2`：保存当前点。
- 采点中 `KEY3`：撤销上一个点。
- `KEY4`：清空本次采点。

推荐操作：

1. 把车放到起点，车头朝向赛道前方。
2. 确认屏幕光标在 `CAIDIAN`，短按 `KEY1`，屏幕显示 `REC ON`，此时可以开始推车。
3. 推着车沿希望自动跑的路线走。
4. 到关键位置短按 `KEY2` 保存点。
5. 点错了短按 `KEY3` 撤销。
6. 全部走完后短按 `KEY1` 结束采点，此时光标会自动放到 `ZIDONG`。
7. 确认车在安全位置，短按 `KEY1` 开始自动驾驶。

每次保存点时，调试串口会打印一行类似：

```c
{ 250.0f, 0.0f, 16.0f, 0 }, /* 01 hdg 0.0 dst 250.0 */
```

结束采点时，串口会打印完整数组。把这些点复制到 `auto_route.c` 中替换占位路线点即可。

注意：采点要想得到真实 `x/y/distance`，需要启用并校准编码器；只靠 IMU 时只能记录航向，位置不会可靠变化。

## 重要约定

- 坐标单位为厘米。
- 起点为 `(0, 0)`。
- 默认 `+X` 是发车时车头前方。
- `heading_deg = 0` 表示朝向 `+X`。
- 角度正方向由平台适配层保证，建议统一为逆时针为正。
- `speed_percent` 和 `steer_percent` 是抽象百分比，最终 PWM 映射由平台适配层负责。

## 需要实测填写的内容

- 绕桩路线点。
- 车库入口路线点。
- 车库对准航向角。
- 倒车入库每段距离。
- 倒车入库每段转向角。
- 巡航、慢速、倒车速度百分比。
- IMU 航向角零点和方向。
- 编码器距离比例系数。
