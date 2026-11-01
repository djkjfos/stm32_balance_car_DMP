#include "mpu.h"
#include "i2c.h"
#include "uart_cmd.h"
void MPU6050_Test(void)
{
    MPU6050_Raw_t sample;
    HAL_StatusTypeDef status = MPU6050_ReadAll(&hi2c2, &sample);
    if (status != HAL_OK) { UART_Printf("MPU read failed: %u\r\n", (unsigned)status); return; }
    UART_Printf("AX=%d AY=%d AZ=%d GX=%d GY=%d GZ=%d\r\n",
                sample.ax,sample.ay,sample.az,sample.gx,sample.gy,sample.gz);
}
