#ifndef __LCD_H
#define __LCD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "i2c.h"
#include <stdint.h>

// 函数声明
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowStr(uint8_t x, uint8_t y, char *str);

#ifdef __cplusplus
}
#endif

#endif
