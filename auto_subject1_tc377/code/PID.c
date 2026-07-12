/*
 * steering_pid.c
 *
 *  Created on: 2026年1月8日
 *      Author: cat
 */
#include "zf_common_headfile.h"

//==========================================================
//  全局 PID 实例
//==========================================================
static Steering_PID_t steer_pid;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID 参数初始化
// 参数说明     kp              比例系数
// 参数说明     ki              积分系数
// 参数说明     kd              微分系数
// 参数说明     integral_limit  积分限幅值（正值）
//-------------------------------------------------------------------------------------------------------------------
void Steering_PID_Init(float kp, float ki, float kd, float integral_limit)
{
    steer_pid.kp = kp;
    steer_pid.ki = ki;
    steer_pid.kd = kd;

    steer_pid.target     = STEER_ADC_CENTER;    // 默认目标：居中
    steer_pid.actual     = 0;
    steer_pid.error      = 0;
    steer_pid.last_error = 0;
    steer_pid.integral   = 0.0f;

    steer_pid.integral_max =  integral_limit;
    steer_pid.integral_min = -integral_limit;

    steer_pid.output = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置目标位置（直接用 ADC 原始值）
// 参数说明     target_adc      目标 ADC 值 (STEER_ADC_LEFT_MAX ~ STEER_ADC_RIGHT_MAX)
//-------------------------------------------------------------------------------------------------------------------
void Steering_Set_Target(int16 target_adc)
{
    if (target_adc < STEER_ADC_LEFT_MAX)        // 不能超过左极限（小值）
        target_adc = STEER_ADC_LEFT_MAX;
    if (target_adc > STEER_ADC_RIGHT_MAX)       // 不能超过右极限（大值）
        target_adc = STEER_ADC_RIGHT_MAX;

    steer_pid.target = target_adc;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置目标位置（百分比方式）
// 参数说明     angle_percent   -100.0 表示最左, 0 表示居中, +100.0 表示最右
//-------------------------------------------------------------------------------------------------------------------
void Steering_Set_Target_Angle(float angle_percent)
{
    int16 target_adc;

    // 限幅
    if (angle_percent < -100.0f) angle_percent = -100.0f;
    if (angle_percent >  100.0f) angle_percent =  100.0f;

    if (angle_percent < 0)
    {
        //左转
        target_adc = (int16)(STEER_ADC_CENTER + (angle_percent / 100.0f) * (STEER_ADC_CENTER - STEER_ADC_LEFT_MAX));
    }
    else
    {
        //右转
        target_adc = (int16)(STEER_ADC_CENTER + (angle_percent / 100.0f) * (STEER_ADC_RIGHT_MAX - STEER_ADC_CENTER));
    }

    Steering_Set_Target(target_adc);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     位置式 PID 核心计算
// 返回参数     int16    PID 输出值（正值=某方向，负值=反方向）
//
//  公式:  output = Kp * error + Ki * ∫error + Kd * (error - last_error)
//-------------------------------------------------------------------------------------------------------------------
int16 Steering_PID_Calc(void)
{
    float p_out, i_out, d_out, pid_output;

    //========== 1. 读取当前位置 ==========
    steer_pid.actual = (int16)Tern_Motor_Read_ADC();

    //========== 2. 计算误差 ==========
    steer_pid.error = steer_pid.target - steer_pid.actual;

    //========== 3. 死区处理：误差很小时不动作 ==========
    if (steer_pid.error > -STEER_DEAD_ZONE && steer_pid.error < STEER_DEAD_ZONE)
    {
        steer_pid.error    = 0;
        steer_pid.integral = 0;     // 到位后清除积分，防缓慢漂移
        steer_pid.output   = 0;
        steer_pid.last_error = 0;
        return 0;
    }

    //========== 4. 积分累加（带抗饱和限幅）==========
    steer_pid.integral += (float)steer_pid.error;

    // 积分限幅
    if (steer_pid.integral > steer_pid.integral_max)
        steer_pid.integral = steer_pid.integral_max;
    if (steer_pid.integral < steer_pid.integral_min)
        steer_pid.integral = steer_pid.integral_min;

    // 积分分离：误差过大时关闭积分，防止大幅度超调
    if (steer_pid.error > 500 || steer_pid.error < -500)
        steer_pid.integral = 0;

    //========== 5. PID 三项计算 ==========
    p_out = steer_pid.kp * (float)steer_pid.error;
    i_out = steer_pid.ki * steer_pid.integral;
    d_out = steer_pid.kd * (float)(steer_pid.error - steer_pid.last_error);

    pid_output = p_out + i_out + d_out;

    //========== 6. 输出限幅 ==========
    if (pid_output >  (float)STEER_PWM_MAX) pid_output =  (float)STEER_PWM_MAX;
    if (pid_output < -(float)STEER_PWM_MAX) pid_output = -(float)STEER_PWM_MAX;

    steer_pid.output = (int16)pid_output;

    //========== 7. 更新上一次误差 ==========
    steer_pid.last_error = steer_pid.error;

    return steer_pid.output;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     转向控制总更新函数（在定时器中断或主循环中周期性调用）
// 建议调用周期  5ms ~ 20ms
//-------------------------------------------------------------------------------------------------------------------
void Steering_Control_Update(void)
{
    int16 pid_out;

    pid_out = Steering_PID_Calc();      // 计算 PID
    Set_Steering_Pwm(pid_out);          // 输出到电机
}

//-------------------------------------------------------------------------------------------------------------------
//  调试用：获取内部状态
//-------------------------------------------------------------------------------------------------------------------
int16 Steering_Get_Error(void)  { return steer_pid.error;  }
int16 Steering_Get_Output(void) { return steer_pid.output; }
int16 Steering_Get_Actual(void) { return steer_pid.actual; }
