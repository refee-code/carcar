/*
 * motor.c
 *
 *  Created on: 2026年1月8日
 *      Author: cat
 */
#include "zf_common_headfile.h"

void Motor_Init(void)
{
    pwm_init(LEFT_PWM1, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
    pwm_init(LEFT_PWM2, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
    pwm_init(RIGHT_PWM1, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
    pwm_init(RIGHT_PWM2, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
    pwm_init(STEER_PWM1, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
    pwm_init(STEER_PWM2, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
    adc_init(TERN_MOTOR_CHANNEL, ADC_12BIT);  // 12-bit 精度，范围 0~4095
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取转向电机 ADC 值（带均值滤波）
// 返回参数     uint16    滤波后的 ADC 值 (0~4095)
//-------------------------------------------------------------------------------------------------------------------
uint16 Tern_Motor_Read_ADC(void)
{
    return adc_mean_filter_convert(TERN_MOTOR_CHANNEL, THROTTLE_FILTER_COUNT);
}

void Set_Left_Pwm(int16 PWM)
{
    if(PWM>=0)
    {
        pwm_set_duty(LEFT_PWM1,PWM_DUTY_MAX);                  // 计算占空比
        pwm_set_duty(LEFT_PWM2,PWM_DUTY_MAX-PWM*0.7);
    }
    else
    {
        PWM=-PWM;
        pwm_set_duty(LEFT_PWM2,PWM_DUTY_MAX);                  // 计算占空比
        pwm_set_duty(LEFT_PWM1,PWM_DUTY_MAX-PWM*0.7);
    }
}

void Set_Right_Pwm(int16 PWM)
{
    if(PWM>=0)
    {
        pwm_set_duty(RIGHT_PWM1,PWM_DUTY_MAX);                  // 计算占空比
        pwm_set_duty(RIGHT_PWM2,PWM_DUTY_MAX-PWM*0.7);
    }
    else
    {
        PWM=-PWM;
        pwm_set_duty(RIGHT_PWM2,PWM_DUTY_MAX);                  // 计算占空比
        pwm_set_duty(RIGHT_PWM1,PWM_DUTY_MAX-PWM*0.7);
    }
}

void Set_Steering_Pwm(int16 PWM)
{
    if(PWM>=0)
    {
        pwm_set_duty(STEER_PWM1,PWM_DUTY_MAX);                  // 计算占空比
        pwm_set_duty(STEER_PWM2,PWM_DUTY_MAX-PWM);
    }
    else
    {
        PWM=-PWM;
        pwm_set_duty(STEER_PWM2,PWM_DUTY_MAX);                  // 计算占空比
        pwm_set_duty(STEER_PWM1,PWM_DUTY_MAX-PWM);
    }
}
