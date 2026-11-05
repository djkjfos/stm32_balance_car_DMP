#pragma once
#include <Arduino.h>
#include "st7789.h"
// Candidate pins only: ensure your specific board exposes them and does not use them for memory or peripherals.
#define BALANCE_UART_RX 16
#define BALANCE_UART_TX 17
#define BALANCE_UART_BAUD 115200
void BalanceLink_Init();
void BalanceLink_Poll();
// Optional raw data to STM32 UART1. Does not define or execute any control command.
bool BalanceLink_Send(const uint8_t *data, size_t length);
// Both values use the STM32 controller's encoder-count units, not percent.
// Call repeatedly (<500 ms) while control is active. Range: -327.68..327.67.
bool BalanceLink_SendTargets(float speed, float turn);
