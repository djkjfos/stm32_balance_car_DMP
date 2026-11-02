/*
 * 控制模块的对外接口：其他 .c 文件通过本头文件访问状态和调度函数。
 * 枚举值按声明顺序从 0 开始，因此日志 RUNNING(2)、FAULT(3) 与下面定义对应。
 * FAULT_TILT=2 允许扶正后自动恢复，其他保护原因的恢复规则见 balance.c。
 */
#ifndef BALANCE_H
#define BALANCE_H
#include "main.h"
#include "config.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef enum { BALANCE_CALIBRATING, BALANCE_READY, BALANCE_RUNNING, BALANCE_FAULT } BalanceState;
typedef enum { FAULT_NONE, FAULT_IMU, FAULT_TILT, FAULT_OVERRUN, FAULT_HARDWARE } BalanceFault;
/*
 * extern 只声明变量，实际存储在 balance.c；volatile 用于中断与主循环共享状态。
 * Balance_OnTickISR 由中断调用；Poll/KeyPoll/PrintDiagnostics 由主循环调用。
 * Init/Calibrate 中有初始化延时和 I2C 操作，应在电机关闭时、主循环环境中调用。
 */
extern volatile BalanceState balance_state;
extern volatile BalanceFault balance_fault;

HAL_StatusTypeDef Balance_Init(void);
void Balance_OnTickISR(void);
void Balance_Poll(void);
void Balance_KeyPoll(void);
void Balance_PrintDiagnostics(void);
uint8_t Balance_Arm(uint8_t enable);
HAL_StatusTypeDef Balance_Calibrate(void);
#ifdef __cplusplus
}
#endif
#endif
