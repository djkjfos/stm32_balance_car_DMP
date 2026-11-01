#ifndef DMP_ATTITUDE_H
#define DMP_ATTITUDE_H

#include "stm32f1xx_hal.h"

/* All functions run in the main loop; not from an interrupt. */
HAL_StatusTypeDef DMP_AttitudeInit(void);
HAL_StatusTypeDef DMP_AttitudePoll(void);
void DMP_AttitudeResetAngle(float angle_deg);
float DMP_AttitudeGetAngle(float gyro_deg_s);
/* 0=none, 1=I2C, 2=invalid FIFO/quaternion, 3=stale, 4=backlog,
 * 5=catch-up read failed, 6=invalid norm or angle. */
unsigned DMP_AttitudeLastError(void);

#endif
