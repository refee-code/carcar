/*
 * pid.h
 *
 *  Created on: 2026��1��8��
 *      Author: cat
 */

#ifndef CODE_STEERING_PID_H_
#define CODE_STEERING_PID_H_

#include "zf_common_headfile.h"

//==========================================================
//  ת�� ADC �궨ֵ
//==========================================================
#define STEER_ADC_LEFT_MAX      (1117)            // �����̴�����ʱ�� ADC ֵ
#define STEER_ADC_CENTER        (1798)          // �����̾���ʱ�� ADC ֵ
#define STEER_ADC_RIGHT_MAX     (2482)          // �����̴�����ʱ�� ADC ֵ

//==========================================================
//  PID ����޷�
//==========================================================
#define STEER_PWM_MAX           (7500)  // PWM �������
#define STEER_DEAD_ZONE         (20)            // ���������С�ڴ�ֵ��������������

//==========================================================
//  PID �ṹ��
//==========================================================
typedef struct
{
    // ���� PID ���� ����
    float kp;               // ����ϵ��
    float ki;               // ����ϵ��
    float kd;               // ΢��ϵ��

    // ���� ���̱��� ����
    int16 target;           // Ŀ��ֵ��ADC ֵ��
    int16 actual;           // ʵ��ֵ��ADC ֵ��
    int16 error;            // ��ǰ��� = target - actual
    int16 last_error;       // ��һ�����
    float integral;         // �������ۼ�

    // ���� �����޷��������ֱ��ͣ�����
    float integral_max;     // ��������
    float integral_min;     // �������ޣ�ͨ��Ϊ -integral_max��

    // ���� ��� ����
    int16 output;           // PID ��������������ţ���/����������
} Steering_PID_t;

//==========================================================
//  ��������
//==========================================================
void  Steering_PID_Init(float kp, float ki, float kd, float integral_limit);
void  Steering_Set_Target(int16 target_adc);
void  Steering_Set_Target_Angle(float angle_percent);  // -100.0 ~ +100.0
int16 Steering_PID_Calc(void);
void  Steering_Control_Update(void);                    // ���ڵ��õ��ܿغ���

// ��ȡ��ǰ״̬�������ԣ�
int16 Steering_Get_Error(void);
int16 Steering_Get_Output(void);
int16 Steering_Get_Actual(void);

#endif /* PID_H_ */
