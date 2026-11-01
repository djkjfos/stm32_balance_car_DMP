/*
 * 串口模块接口与六通道遥测相关变量声明；extern 的实际定义在 uart_cmd.c。
 * UART_Start 初始化一次，UART_Poll 在主循环中反复调用，UART_Printf 用于整行消息。
 * 命令以实际换行字节结尾；推荐上位机文本模式发送 NAME value，再附加换行。
 */
#ifndef UART_CMD_H
#define UART_CMD_H
#include "usart.h"
#ifdef __cplusplus
extern "C" {
#endif
extern volatile float angle, gyro_filt, speed_filt, speedA, speedB;
extern volatile float speed_avg, balance_pwm, raw_gx;
HAL_StatusTypeDef UART_Start(UART_HandleTypeDef *uart);
uint8_t UART_SendByte(uint8_t byte);
void UART_Printf(const char *format, ...);
void UART_Poll(void);
uint8_t rx_line_available(void);
void Frame_Process(void);
#ifdef __cplusplus
}
#endif
#endif
