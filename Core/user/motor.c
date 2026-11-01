#include "motor.h"
#include "tim.h"
#include <math.h>


float motor_deadband = 65.0f;                                                            //死区补偿，65为佳值 ，非负数
static volatile uint8_t enabled;                                                        //软件标志位，0为关闭，1为开启

/*===================软件标志位拉低，STBY拉低，清除两路PWM=====================*/
void Motor_Disable(void)
{
    enabled = 0U;
    HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);
    if (htim4.Instance != NULL) {
        __HAL_TIM_SET_COMPARE(&htim4, PWMA_CH, 0U);
        __HAL_TIM_SET_COMPARE(&htim4, PWMB_CH, 0U);
    }
}

/*============初始化PWM判断两路PWM是否出错，且一路出错两路清零防止单转=============*/
HAL_StatusTypeDef motor_Init(void)
{
    HAL_StatusTypeDef status;
    Motor_Disable();
    if ((status = HAL_TIM_PWM_Start(&htim4, PWMA_CH)) != HAL_OK) return status;
    if ((status = HAL_TIM_PWM_Start(&htim4, PWMB_CH)) != HAL_OK) {
        Motor_Disable(); return status;                                                 //重新关闭PWMA，防止单转时，PWMB停止A在转。
    }
    return HAL_OK;
}

/*==================清零PWM，启动时软件标志位拉高，拉高STBY电机硬件打开===============*/
void Motor_Enable(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, PWMA_CH, 0U);
    __HAL_TIM_SET_COMPARE(&htim4, PWMB_CH, 0U);
    enabled = 1U;
    HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);
}
/*
 * isfinite 判断输入是否为有限数，例如无穷大（+∞或-∞）和NaN空就不是。
 * H桥电机驱动方向引脚 11主动刹车，不是断电  00悬空  10,01控制正反转
 * 超过小命令区后：比较值=min(abs(command)+死区补偿,3599)。
 */
static void set_speed(float command, GPIO_TypeDef *port1, uint16_t pin1,
                      GPIO_TypeDef *port2, uint16_t pin2, uint32_t channel)
{
    float duty;
    if (!enabled || !isfinite(command)) {                                                       //未使能或输入不是有限数时输出为零。
        __HAL_TIM_SET_COMPARE(&htim4, channel, 0U); return;
    }                                                                                           
    if (fabsf(command) <= 15.0f) {                                                              //绝对值 <=15 时输出零，用于启动和摔倒后启动。
        HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim4, channel, 0U); return;
    }
    HAL_GPIO_WritePin(port1, pin1, command > 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);             //方向引脚
    HAL_GPIO_WritePin(port2, pin2, command > 0.0f ? GPIO_PIN_RESET : GPIO_PIN_SET);
    duty = fminf(fabsf(command) + fmaxf(motor_deadband,0.0f), (float)PWM_MAX);                              //CCR，死区补偿与外部输入总和，判定上限
    __HAL_TIM_SET_COMPARE(&htim4, channel, (uint32_t)duty);
}

/*=======================================A轮控制函数输出PWM======================================*/
void MotorA_SetSpeed(float speed)
{
    set_speed(speed * MOTOR_A_SIGN, AIN1_PORT, AIN1_PIN, AIN2_PORT, AIN2_PIN, PWMA_CH);
}

/*=======================================B轮控制函数输出PWM======================================*/
void MotorB_SetSpeed(float speed)
{
    set_speed(speed * MOTOR_B_SIGN, BIN1_PORT, BIN1_PIN, BIN2_PORT, BIN2_PIN, PWMB_CH);
}
