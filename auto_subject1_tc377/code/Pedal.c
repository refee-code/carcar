/*
 * Pedal.c
 *
 *  Created on: 2026年
 *      Author: cat
 */

#include "Pedal.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     油门踏板初始化
// 备注信息     初始化 ADC 通道，12-bit 精度
//-------------------------------------------------------------------------------------------------------------------
void Pedal_Init(void)
{
    adc_init(THROTTLE_ADC_CHANNEL, ADC_12BIT);  // 12-bit 精度，范围 0~4095
    adc_init(BRAKE_ADC_CHANNEL  , ADC_12BIT);  // 12-bit 精度，范围 0~4095
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取油门踏板 ADC 值（带均值滤波）
// 返回参数     uint16    滤波后的 ADC 值 (0~4095)
//-------------------------------------------------------------------------------------------------------------------
uint16 Throttle_Read_ADC(void)
{
    return adc_mean_filter_convert(THROTTLE_ADC_CHANNEL, THROTTLE_FILTER_COUNT);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取刹车踏板 ADC 值（带均值滤波）
// 返回参数     uint16    滤波后的 ADC 值 (0~4095)
//-------------------------------------------------------------------------------------------------------------------
uint16 BRAKE_Read_ADC(void)
{
    return adc_mean_filter_convert(BRAKE_ADC_CHANNEL, THROTTLE_FILTER_COUNT);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 ADC 值线性映射为前进 PWM 占空比
// 参数说明     adc_val     ADC 采样值
// 返回参数     int16       PWM 占空比 (0 ~ THROTTLE_PWM_MAX)
// 备注信息
//   - adc_val < THROTTLE_ADC_MIN  → 输出 0（死区，防止误触）
//   - adc_val > THROTTLE_ADC_MAX  → 输出 THROTTLE_PWM_MAX（饱和保护）
//   - 中间区域线性映射
//-------------------------------------------------------------------------------------------------------------------
int16 Throttle_To_PWM(uint16 adc_val)
{
    int32 pwm_val;

    // 低于死区下限，输出 0
    if (adc_val <= THROTTLE_ADC_MIN)
    {
        return 0;
    }

    // 高于满量程上限，输出最大值
    if (adc_val >= THROTTLE_ADC_MAX)
    {
        return (int16)THROTTLE_PWM_MAX;
    }

    // 线性映射: pwm = (adc - min) / (max - min) * PWM_MAX
    // 用 int32 防止溢出
    pwm_val = (int32)(adc_val - THROTTLE_ADC_MIN) * THROTTLE_PWM_MAX / (THROTTLE_ADC_MAX - THROTTLE_ADC_MIN);

    return (int16)pwm_val;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 ADC 值线性映射为倒车 PWM 占空比
// 参数说明     adc_val     ADC 采样值
// 返回参数     int16       PWM 占空比 (0 ~ BACK_PWM_MAX)
// 备注信息
//   - adc_val < THROTTLE_ADC_MIN  → 输出 0（死区，防止误触）
//   - adc_val > THROTTLE_ADC_MAX  → 输出 THROTTLE_PWM_MAX（饱和保护）
//   - 中间区域线性映射
//-------------------------------------------------------------------------------------------------------------------
int16 Back_To_PWM(uint16 adc_val)
{
    int32 pwm_val;

    // 低于死区下限，输出 0
    if (adc_val <= THROTTLE_ADC_MIN)
    {
        return 0;
    }

    // 高于满量程上限，输出最大值
    if (adc_val >= THROTTLE_ADC_MAX)
    {
        return (int16)BACK_PWM_MAX;
    }

    // 线性映射: pwm = (adc - min) / (max - min) * PWM_MAX
    // 用 int32 防止溢出
    pwm_val = (int32)(adc_val - THROTTLE_ADC_MIN) * BACK_PWM_MAX / (THROTTLE_ADC_MAX - THROTTLE_ADC_MIN);

    return (int16)pwm_val;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     油门控制主函数
// 备注信息     读取 ADC → 映射为 PWM → 输出到左右电机
//              在主循环或定时器中断中周期性调用
//-------------------------------------------------------------------------------------------------------------------
void Throttle_Control(void)
{
    uint16 adc_val;
    int16  pwm_out;

    // 第一步：读取油门踏板 ADC 值（带滤波）
    adc_val = Throttle_Read_ADC();

    // 第二步：线性映射为 PWM 占空比
    pwm_out = Throttle_To_PWM(adc_val);

    // 第三步：输出给电机
    Set_Left_Pwm(-pwm_out);
    Set_Right_Pwm(-pwm_out);

}
