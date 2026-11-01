#ifndef DMP_PLATFORM_H
#define DMP_PLATFORM_H

#include "stm32f1xx_hal.h"

int i2c_write(unsigned char slave, unsigned char reg, unsigned char length,
              unsigned char const *data);
int i2c_read(unsigned char slave, unsigned char reg, unsigned char length,
             unsigned char *data);
void delay_ms(unsigned long ms);
void get_ms(unsigned long *count);
void DMP_PlatformClearError(void);
int DMP_PlatformHasError(void);
void DMP_PlatformSetRunning(int running);
void DMP_PlatformSetStartupPoll(void (*poll)(void));

#endif
