# 科目三遥控状态电机配置说明

本文档按当前代码整理科目三遥控录制状态下的电机控制方式，只说明配置和信号流，不改变代码。

## 1. 生效状态

科目三只有在 `AUTO_SUBJECT3_RECORDING` 录制状态下才进入遥控电机控制。

主循环入口在 `auto/auto_seekfree_runtime.c`：

- 10ms 周期先更新惯导位姿：`auto_seekfree_runtime_subject3_update_pose_only()`
- 然后读取遥控器并输出电机：`auto_seekfree_runtime_subject3_remote_update()`
- 此状态下不走 `Unified_Control_Update()`
- 此状态下不使用踏板 ADC、前进/倒车 GPIO、急停 GPIO 那套统一驱动逻辑

录制结束后，科目三进入自动原路返回，电机控制切回自动路线跟随逻辑。

## 2. 遥控器接收机接口

LoRa 遥控器接收机配置在 `auto_subject1_tc377/code/zf_device_lora3a22.h`：

| 项目 | 当前配置 |
| --- | --- |
| 串口 | `UART_2` |
| 单片机 TX | `UART2_TX_P10_5` |
| 单片机 RX | `UART2_RX_P10_6` |
| 波特率 | `115200` |
| 帧长 | `18` |

摇杆数组含义：

| 摇杆数据 | 含义 | 科目三用途 |
| --- | --- | --- |
| `joystick[0]` | 左摇杆左右 | 科目三未使用 |
| `joystick[1]` | 左摇杆上下 | 后轮驱动 |
| `joystick[2]` | 右摇杆左右 | 前轮方向电机 |
| `joystick[3]` | 右摇杆上下 | 科目三未使用 |

科目三当前读取：

```c
s_subject3_drive_joy = lora3a22_uart_transfer.joystick[1];
s_subject3_steer_joy = lora3a22_uart_transfer.joystick[2];
```

## 3. 后轮驱动电机 PWM 引脚

后轮驱动电机 PWM 配置在 `auto_subject1_tc377/code/motor.h`：

| 电机 | PWM1 | PWM2 |
| --- | --- | --- |
| 左后轮 | `ATOM0_CH4_P02_4` | `ATOM0_CH5_P02_5` |
| 右后轮 | `ATOM0_CH2_P21_4` | `ATOM0_CH3_P21_5` |

初始化在 `Motor_Init()`：

```c
pwm_init(LEFT_PWM1, 17000, 0);
pwm_init(LEFT_PWM2, 17000, 0);
pwm_init(RIGHT_PWM1, 17000, 0);
pwm_init(RIGHT_PWM2, 17000, 0);
```

也就是左右后轮 PWM 频率都是 `17 kHz`，初始占空比为 `0`。

## 4. 后轮驱动输出规则

科目三遥控驱动现在是后轮编码器平均速度闭环，位置在 `auto/auto_seekfree_runtime.c`。摇杆先通过四次曲线生成目标速度，再用左右后轮编码器平均计数做 PI 修正，最后同一个闭环 PWM 同时输出给左右后轮。

| 参数 | 当前值 |
| --- | --- |
| 驱动摇杆死区 | `AUTO_SUBJECT3_DRIVE_DEAD_ZONE = 300` |
| 后轮电机实际起转死区 | 约 `700 PWM` |
| 后轮最小闭环输出 | `AUTO_SUBJECT3_DRIVE_PWM_MIN = 700` |
| 最大驱动 PWM | `AUTO_SEEKFREE_DRIVE_PWM_MAX = 8000.0f` |
| 最大目标速度 | `AUTO_SUBJECT3_DRIVE_SPEED_MAX_CM_S = 220.0f` |
| 速度闭环 KP | `AUTO_SUBJECT3_DRIVE_KP = 8.0f` |
| 速度闭环 KI | `AUTO_SUBJECT3_DRIVE_KI = 0.4f` |
| 速度闭环积分限幅 | `AUTO_SUBJECT3_DRIVE_I_LIMIT = 2500.0f` |
| 正向摇杆标定 | `JOY_DRIVE_POS_MAX = 1938` |
| 反向摇杆标定 | `JOY_DRIVE_NEG_MAX = 2052` |
| 每 10ms 最大变化量 | `AUTO_SUBJECT3_DRIVE_PWM_STEP = 400` |
| 映射函数 | `auto_seekfree_runtime_map_joystick_expo4()` |

四次曲线含义：

- 摇杆在 `-300 ~ +300` 内，输出 `drive_pwm = 0`
- 超过死区后，按四次函数增长
- 相比三次曲线，中段输出更低，摇杆推到中段时加速更柔
- 最大逻辑输出限制为 `8000`

后轮电机本体约有 `700 PWM` 的实际起转死区。当前闭环代码已经加了 `AUTO_SUBJECT3_DRIVE_PWM_MIN = 700`，也就是只要闭环输出非零但小于 `700`，会抬到 `±700`，避免 `D` 有数但车不动。

如果这个 `700 PWM` 是按 `Set_Left_Pwm(700)` / `Set_Right_Pwm(700)` 这种函数入参测试出来的，可以直接按 `700` 理解；如果是按驱动桥两路 PWM 的实际占空比差测试出来的，因为后轮函数内部还有 `* 0.7`，对应的代码命令值大约需要 `700 / 0.7 = 1000`。

科目三后轮闭环核心流程：

```c
drive_pwm = auto_seekfree_runtime_map_joystick_expo4(...);
auto_seekfree_runtime_subject3_drive_closed_loop(drive_pwm);
```

闭环内部会读取最近 10ms 的左右后轮编码器计数，并取平均值：

```c
auto_seekfree_encoder_get_debug_counts(&left_count, &right_count);
left_actual_count = left_count * AUTO_SEEKFREE_LEFT_ENCODER_SIGN;
right_actual_count = right_count * AUTO_SEEKFREE_RIGHT_ENCODER_SIGN;
avg_actual_count = (left_actual_count + right_actual_count) * 0.5f;
```

然后按平均速度做 PI，并把同一个 PWM 给左右后轮：

```c
drive_output_pwm = auto_seekfree_runtime_subject3_drive_pi(...);
s_subject3_left_drive_cmd = s_subject3_drive_cmd;
s_subject3_right_drive_cmd = s_subject3_drive_cmd;
Set_Left_Pwm((int16)-s_subject3_left_drive_cmd);
Set_Right_Pwm((int16)-s_subject3_right_drive_cmd);
```

也就是后轮不再是开环 PWM，而是由左右编码器平均速度闭环修正。左右后轮现在拿到同一个闭环输出，避免某一侧编码器反馈异常时把这一侧单独压停。最终 PWM 仍然每 10ms 最多变化 `400 PWM`，用来兼顾加速响应和中段平顺。

屏幕上显示的 `D` 是左右后轮最终逻辑 PWM 的平均值，实际送进左右电机函数时仍然取负号：

| 屏幕 `D` / `s_subject3_drive_cmd` | 实际入参 | 说明 |
| --- | --- | --- |
| `D = 0` | `Set_Left_Pwm(0)`, `Set_Right_Pwm(0)` | 后轮停止 |
| `D > 0` | 左右轮最终 PWM 取负号输出 | 逻辑前进 |
| `D < 0` | 左右轮最终 PWM 取负号输出 | 逻辑后退 |

`motor.c` 中左右后轮的方向逻辑相同：

```c
if (PWM >= 0) {
    PWM1 = PWM_DUTY_MAX;
    PWM2 = PWM_DUTY_MAX - PWM * 0.7;
} else {
    PWM = -PWM;
    PWM2 = PWM_DUTY_MAX;
    PWM1 = PWM_DUTY_MAX - PWM * 0.7;
}
```

注意：后轮电机的有效 PWM 差值会乘 `0.7`，例如输入 `8000` 时，实际用于占空比差的量约为 `5600`。

## 5. 前轮方向电机 PWM 引脚

方向电机 PWM 配置在 `auto_subject1_tc377/code/motor.h`：

| 电机 | PWM1 | PWM2 |
| --- | --- | --- |
| 前轮方向电机 | `ATOM0_CH0_P21_2` | `ATOM0_CH1_P21_3` |

初始化在 `Motor_Init()`：

```c
pwm_init(STEER_PWM1, 17000, 0);
pwm_init(STEER_PWM2, 17000, 0);
```

方向电机 PWM 频率也是 `17 kHz`，初始占空比为 `0`。

## 6. 方向电机输出规则

科目三遥控方向现在使用位置闭环。摇杆左右值映射成目标角度百分比，方向反馈 ADC 进入 PID；摇杆回中时目标回到中位，前轮会自动回正。

| 参数 | 当前值 |
| --- | --- |
| 方向摇杆死区 | `AUTO_SUBJECT3_STEER_DEAD_ZONE = 250` |
| 左打满 ADC | `STEER_ADC_LEFT_MAX = 1117` |
| 中位 ADC | `STEER_ADC_CENTER = 1799` |
| 右打满 ADC | `STEER_ADC_RIGHT_MAX = 2482` |
| 方向 PID KP | `AUTO_SEEKFREE_STEER_KP = 8.0f` |
| 方向 PID KI | `AUTO_SEEKFREE_STEER_KI = 0.0f` |
| 方向 PID KD | `AUTO_SEEKFREE_STEER_KD = 0.0f` |
| 方向 PID 输出上限 | `STEER_PWM_MAX = 4500` |
| 方向 ADC 到位死区 | `STEER_DEAD_ZONE = 20` |
| 科目三方向目标斜坡 | `AUTO_SUBJECT3_STEER_PERCENT_STEP = 25` |
| 科目三方向最小启动 PWM | `AUTO_SUBJECT3_STEER_PWM_MIN = 1500` |
| 正向摇杆标定 | `JOY_STEER_POS_MAX = 1956` |
| 反向摇杆标定 | `JOY_STEER_NEG_MAX = 2037` |
| 映射函数 | `auto_seekfree_runtime_map_joystick_deadzone()` |

方向 ADC 标定按 `ADC_12BIT`、`3.3V` 参考电压换算：左打满 `0.9V -> 1117`，中位约 `1.45V -> 1799`，右打满 `2.0V -> 2482`。

输出流程：

1. 摇杆在 `-250 ~ +250` 内，目标方向百分比为 `0`
2. 超过死区后，线性映射到 `-100 ~ +100`
3. 目标百分比每 10ms 最多变化 `25%`
4. 调用 `Steering_Set_Target_Angle()` 设置目标 ADC
5. 调用 `Steering_PID_Calc()` 计算方向 PID
6. PID 非零但小于 `1500 PWM` 时抬到 `±1500 PWM`
7. 调用 `Set_Steering_Pwm()` 输出方向电机 PWM

科目三实际输出：

```c
steer_percent = auto_seekfree_runtime_map_joystick_deadzone(...);
s_subject3_steer_percent_cmd =
    auto_seekfree_runtime_slew_i32(s_subject3_steer_percent_cmd,
                                   steer_percent,
                                   AUTO_SUBJECT3_STEER_PERCENT_STEP);
Steering_Set_Target_Angle((float)-s_subject3_steer_percent_cmd);
steer_pwm = Steering_PID_Calc();
if (steer_pwm > 0 && steer_pwm < AUTO_SUBJECT3_STEER_PWM_MIN) {
    steer_pwm = AUTO_SUBJECT3_STEER_PWM_MIN;
} else if (steer_pwm < 0 && steer_pwm > -AUTO_SUBJECT3_STEER_PWM_MIN) {
    steer_pwm = -AUTO_SUBJECT3_STEER_PWM_MIN;
}
Set_Steering_Pwm(steer_pwm);
s_subject3_steer_cmd = steer_pwm;
```

屏幕上显示的 `S` 是方向 PID 实际输出 PWM：

| 屏幕 `S` / `s_subject3_steer_cmd` | 实际入参 | 说明 |
| --- | --- | --- |
| `S = 0` | PID 认为已经到位 | 方向电机停止输出 |
| `S > 0` | `Set_Steering_Pwm(S)` | 方向 PID 正向修正 |
| `S < 0` | `Set_Steering_Pwm(S)` | 方向 PID 反向修正 |

`motor.c` 中方向电机的方向逻辑：

```c
if (PWM >= 0) {
    STEER_PWM1 = PWM_DUTY_MAX;
    STEER_PWM2 = PWM_DUTY_MAX - PWM;
} else {
    PWM = -PWM;
    STEER_PWM2 = PWM_DUTY_MAX;
    STEER_PWM1 = PWM_DUTY_MAX - PWM;
}
```

注意：方向电机没有乘 `0.7`，输入多少 PWM 就直接用于占空比差。

## 7. 失控保护和停车

LoRa 在线判断：

```c
lora3a22_state_flag == 1 &&
lora3a22_response_time <= LORA_TIMEOUT_TICKS
```

`LORA_TIMEOUT_TICKS = 50`，代码注释按 `10ms * 50` 理解，大约是 `500ms`。

如果遥控器离线或超时，科目三会直接清零全部原始输出：

```c
Set_Left_Pwm(0);
Set_Right_Pwm(0);
Set_Steering_Pwm(0);
```

在科目三非录制状态、科目选择页、科目二保持页等情况下，也会清零原始输出，防止电机误动。

## 8. 屏幕调试显示

科目三录制时屏幕显示：

| 显示项 | 含义 |
| --- | --- |
| `J1` | `joystick[1]`，左摇杆上下，驱动输入 |
| `J2` | `joystick[2]`，右摇杆左右，方向输入 |
| `D` | `s_subject3_drive_cmd`，左右后轮共同使用的闭环逻辑 PWM |
| `S` | `s_subject3_steer_cmd`，方向 PID 实际输出 PWM |

注意：`D` 是左右后轮共同使用的闭环逻辑 PWM；`S` 是方向 PID 实际输出 PWM，不是摇杆原始值。

## 9. 和编码器、惯导的关系

科目三遥控录制状态下，后轮和方向都是闭环：

- 后轮电机：遥控摇杆给目标速度，左右后轮编码器平均反馈做速度 PI
- 方向电机：遥控摇杆给目标角度百分比，`Tern_Motor_Read_ADC()` 反馈给方向 PID
- 不使用自动驾驶路线跟随 PID
- 不使用踏板控制驱动
- 惯导仍用于更新航向、录制路线和屏幕绘制
- 后轮编码器同时用于闭环控速、位移估计、录制路线和屏幕绘制

当前后轮方向编码器配置在 `auto/auto_config.h`：

| 编码器 | 定时器 | 计数引脚 | 方向引脚 | 显示符号 |
| --- | --- | --- | --- | --- |
| 左后轮 | `TIM2_ENCODER` | `TIM2_ENCODER_CH1_P33_7` | `TIM2_ENCODER_CH2_P33_6` | `+1.0` |
| 右后轮 | `TIM5_ENCODER` | `TIM5_ENCODER_CH1_P10_3` | `TIM5_ENCODER_CH2_P10_1` | `-1.0` |

`AUTO_SEEKFREE_ENCODER_USE_AVERAGE = 1u`，说明位移估计使用左右后轮编码器平均值。

闭环试车时要先确认：车辆前进时，乘上显示符号后的左/右计数都应该是正增长。如果某一侧符号反了或编码器没反馈，PI 会认为速度不够并继续加 PWM，表现为 `D` 变大但车速/计数不对。

## 10. 调参提示

起步仍然太猛时，优先看这几个量：

- 增大 `AUTO_SUBJECT3_DRIVE_DEAD_ZONE`
- 降低 `AUTO_SUBJECT3_DRIVE_SPEED_MAX_CM_S`
- 继续调整四次曲线，让小摇杆段和中段更软
- 减小 `AUTO_SUBJECT3_DRIVE_PWM_STEP`，让加速斜坡更慢
- 减小 `AUTO_SUBJECT3_DRIVE_KP` 或 `AUTO_SUBJECT3_DRIVE_KI`

起步太肉或推杆后不动时：

- 减小 `AUTO_SUBJECT3_DRIVE_DEAD_ZONE`
- 增大 `AUTO_SUBJECT3_DRIVE_PWM_MIN`，参考后轮实际起转死区约 `700 PWM`
- 增大 `AUTO_SUBJECT3_DRIVE_PWM_STEP`，让油门响应更快
- 适当增大 `AUTO_SUBJECT3_DRIVE_KP`

方向太慢时：

- 适当降低 `AUTO_SUBJECT3_STEER_DEAD_ZONE`
- 增大 `AUTO_SEEKFREE_STEER_KP`
- 增大 `AUTO_SUBJECT3_STEER_PWM_MIN`
- 检查 `STEER_ADC_LEFT_MAX`、`STEER_ADC_CENTER`、`STEER_ADC_RIGHT_MAX` 是否标定正确

方向太飘或抖动时：

- 增大 `AUTO_SUBJECT3_STEER_DEAD_ZONE`
- 减小 `AUTO_SEEKFREE_STEER_KP`
- 适当增大 `STEER_DEAD_ZONE`
- 减小 `AUTO_SUBJECT3_STEER_PWM_MIN`
- 检查遥控器中位是否稳定
- 检查方向反馈 ADC 是否稳定

方向左右反了时，检查科目三这里的目标符号：

```c
Steering_Set_Target_Angle((float)-s_subject3_steer_percent_cmd);
```

驱动前后反了时，检查最终输出符号：

```c
Set_Left_Pwm((int16)-s_subject3_left_drive_cmd);
Set_Right_Pwm((int16)-s_subject3_right_drive_cmd);
```
