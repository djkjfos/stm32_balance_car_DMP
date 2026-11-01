/*
 * 电机引脚和输出上限集中在这里：PB12~PB15 为方向，PB0 为 STBY，
 * PB6/PB7 为 TIM4_CH1/CH2。修改接线时要同时核对 gpio.c、tim.c 及 I2C.ioc。
 * MOTOR_A_SIGN/B_SIGN 改变驱动方向；ENCODER_A_SIGN/B_SIGN 改变速度反馈方向，两者含义不同。
 * PWM_MAX=3599 与定时器 ARR 配套；MotorA/B_SetSpeed 接收带正负号的 PWM 控制量。
 */
#ifndef MOTOR_H
#define MOTOR_H
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif
#define AIN1_PORT GPIOB
#define AIN1_PIN GPIO_PIN_12
#define AIN2_PORT GPIOB
#define AIN2_PIN GPIO_PIN_13
#define BIN1_PORT GPIOB
#define BIN1_PIN GPIO_PIN_14
#define BIN2_PORT GPIOB
#define BIN2_PIN GPIO_PIN_15
#define STBY_PORT GPIOB
#define STBY_PIN GPIO_PIN_0
#define PWMA_CH TIM_CHANNEL_1
#define PWMB_CH TIM_CHANNEL_2
#define PWM_MAX 3599U
#define MOTOR_A_SIGN (1)
#define MOTOR_B_SIGN (1)
extern float motor_deadband;
HAL_StatusTypeDef motor_Init(void);
void Motor_Enable(void);
void Motor_Disable(void);
void MotorA_SetSpeed(float speed);
void MotorB_SetSpeed(float speed);
#ifdef __cplusplus
}
#endif
#endif
