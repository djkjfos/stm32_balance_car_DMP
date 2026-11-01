/* ST7789 240x240：异步初始化、DMA 分块刷新；主循环调用 Poll，禁止中断内绘图。 */
#ifndef ST7789_H
#define ST7789_H
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif
#define ST7789_WIDTH 240U
#define ST7789_HEIGHT 240U
/* 面板偏移与方向集中在这里；默认沿用本机 st7789 工程的竖屏配置。 */
#define ST7789_X_OFFSET 0U
#define ST7789_Y_OFFSET 0U
#define ST7789_MADCTL 0x00U
typedef struct {
    float angle, gyro, encoder_a, encoder_b;
    uint16_t pwm_a, pwm_b;
    uint8_t state, fault, stby, key;
} ST7789_Status;
/* 状态：0=初始化/清屏，1=正常刷新，2=传输故障后停用屏幕。无屏幕接线不能检测。 */
void ST7789_Init(void);
void ST7789_SetStatus(const ST7789_Status *status);
void ST7789_Poll(void);
uint8_t ST7789_GetStatus(void);
#ifdef __cplusplus
}
#endif
#endif
