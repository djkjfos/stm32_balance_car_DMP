/*
 * 编码器接口及方向设置：A 轮 TIM2/PA0/PA1，B 轮 TIM3/PA6/PA7。
 * 方向符号用于统一两轮“向前”为正；推动车体前进时两轮增量应同为正。
 * GetSpeed 的结果是本采样间隔的脉冲计数差，正负表示转动方向。
 */
#ifndef ENCODER_H
#define ENCODER_H
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Pushing the car forward must give positive counts on BOTH wheels. */
#define ENCODER_A_SIGN (1)
#define ENCODER_B_SIGN (-1)
HAL_StatusTypeDef EncoderA_Init(void);
HAL_StatusTypeDef EncoderB_Init(void);
int16_t EncoderA_GetSpeed(void);
int16_t EncoderB_GetSpeed(void);
#ifdef __cplusplus
}
#endif
#endif
