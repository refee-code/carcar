/*
 *  KEY.h
 *
 *  Created on: 2026Äê1ÔÂ8ÈÕ
 *      Author: cat
 */
#ifndef KEY_H_
#define KEY_H_

#include "zf_common_headfile.h"

#define KEY1  P20_6
#define KEY2  P20_7
#define KEY3  P11_2
#define KEY4  P11_3

typedef enum
{
    None = 0,
    Key1,
    Key2,
    Key3,
    Key4,
}Key;

extern Key Key_Value;

void key_scan(void);
void Key_init(void);


#endif /* CODE_KEY_H_ */
