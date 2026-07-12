/*
 * Pedal.h
 *
 *  Created on: 2026年
 *      Author: cat
 */

#ifndef CODE_PEDAL_H_
#define CODE_PEDAL_H_

#include "zf_common_headfile.h"

// ======================== 用户可配置参数 ========================

// 油门踏板 ADC 通道（选择一个空闲的ADC引脚接踏板信号输出）
#define THROTTLE_ADC_CHANNEL    (ADC0_CH2_A2)
#define BRAKE_ADC_CHANNEL       (ADC0_CH1_A1)

// 油门踏板 ADC 死区下限（踏板完全松开时的ADC值 ,松开踏板的ADC值1060 +20~50 作为死区，避免误触）
#define THROTTLE_ADC_MIN        (1080)

// 油门踏板 ADC 满量程上限（踏板完全踩下时的ADC值3160 -50~100 确保能到满）
// 建议比实际踩到底的值稍低一点，确保能达到满占空比
#define THROTTLE_ADC_MAX        (3110)

// 输出 PWM 占空比上限，限制最大输出功率
// PWM_DUTY_MAX = 10000，即 10000 = 100% 占空比
#define THROTTLE_PWM_MAX        (8000)
#define BACK_PWM_MAX            (5000)

//刹车 ADC 阈值
#define BRAKE_ADC_THRESHOLD     (1400)

// 均值滤波采样次数
#define THROTTLE_FILTER_COUNT   (10)

void Pedal_Init(void);
uint16 Throttle_Read_ADC(void);
uint16 BRAKE_Read_ADC(void);
int16 Throttle_To_PWM(uint16 adc_val);
int16 Back_To_PWM(uint16 adc_val);
void Throttle_Control(void);

#endif /* CODE_PEDAL_H_ */
