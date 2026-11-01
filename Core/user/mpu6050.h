/*
 * 传感器接口与寄存器定义。MPU6050_Raw_t 的加速度、角速度和温度均为原始有符号计数。
 * 在本配置下，加速度除以 16384 得到 g，角速度扣偏置后除以 131 得到度/秒。
 * MPU6050_ADDR 是给 HAL 使用的左移地址；WHO_AM_I 是芯片身份值。
 */
#ifndef MPU6050_H
#define MPU6050_H
#include "stm32f1xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Keep the existing controller API names; the DMP version supports MPU6050. */
#define MPU6050_ADDR (0x68U << 1)
#define MPU6050_ID 0x68U
#define MPU6500_ID 0x70U
#define MPU6050_SMPLRT_DIV 0x19U                        //采样率分频寄存器
#define MPU6050_CONFIG 0x1AU                            //外部同步帧 + 数字低通滤波 DLPF 配置寄存器
#define MPU6050_GYRO_CONFIG 0x1BU                       //陀螺仪配置寄存器
#define MPU6050_ACCEL_CONFIG 0x1CU                      //加速度计配置寄存器
#define MPU6500_ACCEL_CONFIG2 0x1DU                     //MPU6500 独有寄存器，加速度低通滤波
#define MPU6050_ACCEL_XOUT_H 0x3BU                      //加速度 X 轴高字节
#define MPU6050_GYRO_XOUT_H 0x43U                       //陀螺仪 X 轴高字节。
#define MPU6050_PWR_MGMT_1 0x6BU                        //电源管理 1
#define MPU6050_PWR_MGMT_2 0x6CU                        //电源管理 2，轴开关
#define MPU6050_WHO_AM_I 0x75U                          //芯片 ID 只读寄存器
/*
 * 控制步长 0.005 秒=5ms，与 TIM1 的 200Hz 节拍及传感器采样设置一致。
 */
#define DT 0.005f
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
} MPU6050_Raw_t;
extern MPU6050_Raw_t *raw;
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *bus);
HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *bus, MPU6050_Raw_t *sample);
unsigned MPU6050_LastReadStage(void); /* 1=raw register read, 2=DMP FIFO */
float MPU6050_AccelAngleX(const MPU6050_Raw_t *sample);
float MPU6050_GetAngleX(const MPU6050_Raw_t *sample, float gyro_deg_s);
float MPU6050_GetGyroX(const MPU6050_Raw_t *sample, float bias_counts);
void MPU6050_ResetAngle(float angle_deg);
#ifdef __cplusplus
}
#endif
#endif
