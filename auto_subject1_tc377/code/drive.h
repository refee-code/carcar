/*
 * remote_control.h
 *
 *  Created on: 2026年
 *      Author: cat
 */

#ifndef CODE_DRIVE_H_
#define CODE_DRIVE_H_

#include "zf_common_headfile.h"

//==========================================================
//  摇杆标定值（从实测数据填入）
//==========================================================

// 左摇杆上下 (joystick[1]) → 驱动
#define JOY_DRIVE_POS_MAX       (1938)
#define JOY_DRIVE_NEG_MAX       (2052)

// 右摇杆左右 (joystick[2]) → 转向
#define JOY_STEER_POS_MAX       (1956)
#define JOY_STEER_NEG_MAX       (2037)

// 摇杆死区
#define JOYSTICK_DEAD_ZONE      (80)

// 失控保护超时 （中断周期*此值）（10ms × 50 = 500ms）
#define LORA_TIMEOUT_TICKS      (50)

//==========================================================
//  按键引脚定义（默认上拉，按下为低电平）
//==========================================================
#define PIN_DRIVE_MODE          (P02_6)     // 驾驶模式：松开=前进, 按下=倒车
#define PIN_EMERGENCY_STOP      (P02_7)     // 急停按钮：按下=驱动电机全部停止

//==========================================================
//  控制源标识
//==========================================================
typedef enum
{
    CTRL_SRC_NONE    = 0,       // 无控制（全部停止）
    CTRL_SRC_PEDAL   = 1,       // 踏板控制驱动
    CTRL_SRC_REMOTE  = 2,       // 遥控器控制驱动
    CTRL_SRC_ESTOP   = 3,       // 急停状态
    CTRL_SRC_BRAKE   = 4,       // 刹车状态
} ctrl_source_e;

//==========================================================
//  函数声明
//==========================================================
void          Button_Init(void);                // 按键 GPIO 初始化
void          Unified_Control_Update(void);     // 统一控制函数
ctrl_source_e Get_Drive_Source(void);           // 获取当前驱动控制源

#endif /* CODE_DRIVE_H_ */
