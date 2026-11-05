#ifndef DISPLAY_LINK_H
#define DISPLAY_LINK_H
#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef struct {
    float angle, gyro, encoder_a, encoder_b;
    uint16_t pwm_a, pwm_b;
    uint8_t state, fault, stby, key;
} DisplayStatus;

// Call once in main after MX_USART1_UART_Init().
HAL_StatusTypeDef DisplayLink_Init(UART_HandleTypeDef *uart);
// Call in main every 200 ms; returns HAL_BUSY and drops the frame if still transmitting.
HAL_StatusTypeDef DisplayLink_SendStatus(const DisplayStatus *s);
// Raw bytes received from ESP32; call from main only. 1 = byte read, 0 = no data.
uint8_t DisplayLink_ReadByte(uint8_t *out);
// Sticky error/overflow indication. On non-zero discard the current input frame.
uint8_t DisplayLink_TakeRxError(void);
// Consume incoming control frames. Returns 1 only for a complete valid pair.
uint8_t DisplayLink_PollCommand(float *speed, float *turn);
// These are dispatch targets, NOT HAL callbacks. Call from the existing HAL callbacks.
void DisplayLink_OnTxComplete(UART_HandleTypeDef *uart);
void DisplayLink_OnRxComplete(UART_HandleTypeDef *uart);
void DisplayLink_OnError(UART_HandleTypeDef *uart);
#endif
