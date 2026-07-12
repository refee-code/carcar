/*
 * motor.h
 *
 *  Created on: 2026��1��8��
 *      Author: cat
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#define LEFT_PWM1               (ATOM0_CH4_P02_4)
#define LEFT_PWM2               (ATOM0_CH5_P02_5)
#define RIGHT_PWM1              (ATOM0_CH2_P21_4)
#define RIGHT_PWM2              (ATOM0_CH3_P21_5)
//ת����
#define STEER_PWM1              (ATOM0_CH0_P21_2)
#define STEER_PWM2              (ATOM0_CH1_P21_3)

#define TERN_MOTOR_CHANNEL      (ADC0_CH4_A4)

void   Motor_Init(void);
uint16 Tern_Motor_Read_ADC(void);
void   Set_Left_Pwm(int16 PWM);
void   Set_Right_Pwm(int16 PWM);
void   Set_Steering_Pwm(int16 PWM);

#endif /* CODE_MOTOR_H_ */
