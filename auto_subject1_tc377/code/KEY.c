/*
 *  KEY.c
 *
 *  Created on: 2026年1月8日
 *      Author: cat
 */
#include "zf_common_headfile.h"

bool Key_Status[4];         //按键状态
bool Key_Status_Last[4];    //上次按键状态

Key Key_Value = None;       //按键值
void Key_init(void)
{
    gpio_init(KEY1, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY2, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY3, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY4, GPI, 1, GPI_PULL_UP);

    for(int i=0; i<10; i++)
    {
        key_scan();
        system_delay_ms(10);
    }
    // 强制清空 Key_Value，无视上电瞬间产生的信号
    Key_Value = None;
}


void key_scan(void)
{
    Key_Status_Last[0] = Key_Status[0];
    Key_Status_Last[1] = Key_Status[1];
    Key_Status_Last[2] = Key_Status[2];
    Key_Status_Last[3] = Key_Status[3];

    Key_Status[0] = gpio_get_level(KEY1);
    Key_Status[1] = gpio_get_level(KEY2);
    Key_Status[2] = gpio_get_level(KEY3);
    Key_Status[3] = gpio_get_level(KEY4);

    //检测到按键按下之后  并放开置位标志位
    if     (Key_Status[0] && !Key_Status_Last[0])   Key_Value = Key1;
    else if(Key_Status[1] && !Key_Status_Last[1])   Key_Value = Key2;
    else if(Key_Status[2] && !Key_Status_Last[2])   Key_Value = Key3;
    else if(Key_Status[3] && !Key_Status_Last[3])   Key_Value = Key4;
    else    Key_Value = None;
}
