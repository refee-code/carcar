/*
 * remote_control.c
 *
 *  Created on: 2026年
 *      Author: cat
 */
#include "drive.h"

//==========================================================
//  模块内部变量
//==========================================================
static ctrl_source_e drive_source = CTRL_SRC_NONE;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键 GPIO 初始化
//-------------------------------------------------------------------------------------------------------------------
void Button_Init(void)
{
    // 上拉输入：松开=高电平，按下=低电平
    gpio_init(PIN_DRIVE_MODE,     GPI, 1, GPI_PULL_UP);
    // 急停开关：松开=低电平，按下=高电平
    gpio_init(PIN_EMERGENCY_STOP, GPI, 1, GPI_PULL_UP);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     非对称摇杆映射（正负方向极限不同）
// 参数说明     joy_val     摇杆的原始ADC值，中心点为 0
// 参数说明     pos_max     正方向的理论最大ADC值（正数）
// 参数说明     neg_max     负方向的理论最大ADC值（正数，代表绝对值）。
// 参数说明     out_max     期望输出的最大值（正数）。输出范围为[-out_max, out_max]。
// 返回参数     int32       非对称摇杆映射后的输出值
//-------------------------------------------------------------------------------------------------------------------
static int32 Joystick_Map_Asymmetric(int16 joy_val, int16 pos_max, int16 neg_max, int32 out_max)
{
    int32 result;

    // 死区处理，消除摇杆回中时的微小抖动或零漂，避免设备在"松开"状态下仍有微小输出
    if (joy_val > -JOYSTICK_DEAD_ZONE && joy_val < JOYSTICK_DEAD_ZONE)
    {
        return 0;
    }

    if (joy_val > 0)    //摇杆偏向正方向
    {
        //输出 = (当前摇杆值 / 正方向最大值) * 输出最大值
        result = (int32)joy_val * out_max / (int32)pos_max;
        if (result > out_max)
            result = out_max;
    }
    else    //摇杆偏向负方向
    {
        //输出 = (当前摇杆值 / 负方向最大值) * 输出最大值
        result = (int32)joy_val * out_max / (int32)neg_max;
        if (result < -out_max)
            result = -out_max;
    }

    return result;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断遥控器是否在线（由Unified_Control_Update内部调用）
// 注意： Unified_Control_Update需放在 10ms 中断调用，如果不在 10ms 中断需要在drive.h里修改LORA_TIMEOUT_TICKS的值
//-------------------------------------------------------------------------------------------------------------------
static uint8 Remote_Is_Online(void)
{
    return (lora3a22_state_flag == 1 && lora3a22_response_time <= LORA_TIMEOUT_TICKS);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     统一控制更新函数（10ms 定时器中断中调用）
// 优先级：急停 > 遥控 > 踏板
//-------------------------------------------------------------------------------------------------------------------
void Unified_Control_Update(void)
{
    int16 drive_joy;         //存储遥控器驱动电机值
    int16 steer_joy;         //存储遥控器转向电机值
    int32 remote_drive_pwm;  //遥控驱动值映射后的PWM值
    float steer_percent;     //转向百分比（-100%~100%）
    int16 final_drive_pwm;   //最终确定的驱动PWM值

    uint8 estop_pressed;        // 急停按钮状态
    uint8 reverse_pressed;      // 倒车按键状态
    uint8 brake_active;         // 刹车状态
    //==========================================================
    //  0. 读取按键状态（倒车按键低电平 = 按下，急停按键高电平 = 按下）
    //==========================================================
    estop_pressed = 0;
    reverse_pressed = (gpio_get_level(PIN_DRIVE_MODE)     == 0);    // 0=按下=倒车

    uint16 brake_adc = BRAKE_Read_ADC();
    brake_active     = (brake_adc >= BRAKE_ADC_THRESHOLD);      // 超过阈值 = 刹车踩下
    //==========================================================
    // 急停检测（最高优先级，按下就停，什么都不管）
    //==========================================================
    if (estop_pressed)
    {
        // 驱动电机立即停止
        Set_Left_Pwm(0);
        Set_Right_Pwm(0);

        drive_source = CTRL_SRC_ESTOP;

        // 转向 PID
        // Steering_Control_Update();
        return;     // 直接返回，不执行后续任何驱动逻辑
    }

    //==========================================================
    // 刹车检测（第二优先级，高于油门和遥控）
    //==========================================================
    if (brake_active)
    {
        Set_Left_Pwm(0);
        Set_Right_Pwm(0);
        drive_source = CTRL_SRC_BRAKE;

        // 刹车时转向 PID 仍然运行（保持转向能力）
        // 转向仲裁放在下面统一处理，所以不 return，跳过驱动部分即可
        // 但为了跳过第4步驱动仲裁，用 goto 或标志位
        goto STEERING_CONTROL;
    }

    //==========================================================
    // 超时计数
    //==========================================================
    if (lora3a22_response_time <= LORA_TIMEOUT_TICKS + 10)
    {
        lora3a22_response_time++;   //遥控器回调函数会清零这个lora3a22_response_time，所以这里这个值如果能累加到很多就说明遥控器离线
    }

    if (lora3a22_response_time > LORA_TIMEOUT_TICKS)
    {
        lora3a22_state_flag = 0;
    }

    //==========================================================
    // 读取所有输入源
    //==========================================================

    // —— 踏板 ——
    uint16 pedal_adc = Throttle_Read_ADC();             //读取油门踏板ADC
    int16  pedal_pwm = Throttle_To_PWM(pedal_adc);      //油门踏板线性前进模式映射
    int16  back_pwm  = Back_To_PWM(pedal_adc);          //油门踏板线性倒车模式映射

    // —— 遥控摇杆 ——
    drive_joy = lora3a22_uart_transfer.joystick[1];     //读取遥控器驱动电机值
    steer_joy = lora3a22_uart_transfer.joystick[2];     //读取遥控器转向电机值

    // 映射遥控驱动值
    remote_drive_pwm = Joystick_Map_Asymmetric(drive_joy,JOY_DRIVE_POS_MAX,JOY_DRIVE_NEG_MAX,THROTTLE_PWM_MAX);

    //==========================================================
    // 驱动电机仲裁：踏板优先
    //==========================================================
    if (pedal_pwm > 0)
    {
        //--------------------------------------------------
        // 踏板有输入 → 踏板控制（最高驱动优先级）
        // 根据模式按键决定前进/倒车
        //--------------------------------------------------
        drive_source = CTRL_SRC_PEDAL;

        if (reverse_pressed)
        {
            // 倒车模式：P02_6 按下
            // pedal_pwm 是正值，取正输出 = 反转
            Set_Left_Pwm ((int16)back_pwm);
            Set_Right_Pwm((int16)back_pwm);
        }
        else
        {
            // 前进模式：P02_6 松开（默认）
            // 负值 = 前进（与你原来一致）
            Set_Left_Pwm (-(int16)pedal_pwm);
            Set_Right_Pwm(-(int16)pedal_pwm);
        }
    }
    else if (Remote_Is_Online() && (drive_joy > JOYSTICK_DEAD_ZONE || drive_joy < -JOYSTICK_DEAD_ZONE))
    {
        //--------------------------------------------------
        // 遥控摇杆有输入 → 遥控控制（遥控自带方向，不受模式按键影响）
        //--------------------------------------------------
        final_drive_pwm = (int16)remote_drive_pwm;
        drive_source = CTRL_SRC_REMOTE;

        Set_Left_Pwm(-(int16)final_drive_pwm);
        Set_Right_Pwm(-(int16)final_drive_pwm);
    }
    else
    {
        //--------------------------------------------------
        // 踏板松开 且 遥控无输入 → 停车
        //--------------------------------------------------
        drive_source = CTRL_SRC_NONE;
        Set_Left_Pwm(0);
        Set_Right_Pwm(0);
    }

    //==========================================================
    // 转向电机仲裁
    //==========================================================
    STEERING_CONTROL:
        // 这里需要重新读取摇杆值（刹车分支跳过了摇杆值的读取）
        steer_joy = lora3a22_uart_transfer.joystick[2];
        if (Remote_Is_Online() && (steer_joy > JOYSTICK_DEAD_ZONE || steer_joy < -JOYSTICK_DEAD_ZONE))
        {
            steer_percent = (float)Joystick_Map_Asymmetric(steer_joy,JOY_STEER_POS_MAX,JOY_STEER_NEG_MAX,100);
            Steering_Set_Target_Angle(-steer_percent);
        }
        else
        {
            Steering_Set_Target_Angle(0);
        }

        Steering_Control_Update();
}

//-------------------------------------------------------------------------------------------------------------------
//  调试用：获取当前驱动控制源
//-------------------------------------------------------------------------------------------------------------------
ctrl_source_e Get_Drive_Source(void)
{
    return drive_source;
}
