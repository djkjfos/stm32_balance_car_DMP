/*
 * 用 TIM2/TIM3 的硬件编码器模式累计 A B 相脉冲，再计算相邻两次读取的差值。
 * 本文件不把脉冲换成 rpm 或 m/s；正常每 5ms 调用一次时，返回计数/5ms。
 */
#include "encoder.h"
#include "tim.h"
static uint16_t last_cntA;
static uint16_t last_cntB;
/* 启动 A，B 轮 定时器，并记录起始计数，判断启动是否出错 */
HAL_StatusTypeDef EncoderA_Init(void)
{   
    if(htim2.Instance == NULL) return HAL_ERROR;
    HAL_StatusTypeDef status = HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    last_cntA = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    return status;
}
HAL_StatusTypeDef EncoderB_Init(void)
{
    if(htim3.Instance == NULL) return HAL_ERROR;
    HAL_StatusTypeDef status = HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    last_cntB = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    return status;
}
/*=========================两差绝对值小于 32768即可，得出的值只是编码器的脉冲非速度 ===========================*/
int16_t EncoderA_GetSpeed(void)
{
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);                     //uint32_t → uint16_t
    int16_t delta = (int16_t)(uint16_t)(now - last_cntA);                       //强转 uint16_t 把负数转换成正数
    last_cntA = now;
    return (int16_t)(ENCODER_A_SIGN * (int32_t)delta);
}
int16_t EncoderB_GetSpeed(void)
{
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);                     //uint32_t → uint16_t
    int16_t delta = (int16_t)(uint16_t)(now - last_cntB);                       //强转 uint16_t 把负数转换成正数
    last_cntB = now;
    return (int16_t)(ENCODER_B_SIGN * (int32_t)delta);
}
