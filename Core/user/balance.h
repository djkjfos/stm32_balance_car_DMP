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
typedef enum {
    BALANCE_CALIBRATING,  // 校准中：上电后静止采200个样本，电机全程关着
    BALANCE_READY,        // 就绪：校准通过，等按键/自启才能跑
    BALANCE_RUNNING,      // 运行中：闭环PID生效，电机在出力平衡
    BALANCE_FAULT         // 故障：触发保护，电机关闭
} BalanceState;

typedef enum {
    FAULT_NONE,        // 无故障（正常状态，不是FAULT时用这个）
    FAULT_IMU,         // IMU错误：I2C读MPU6050失败、或算出NaN/无穷大
    FAULT_TILT,        // 倾倒故障：实测角≥45°。唯一允许"扶正后自动恢复"的故障
    FAULT_OVERRUN,     // 节拍超时：主循环卡死，pending_ticks积压≥2
    FAULT_HARDWARE     // 硬件错误：电机/编码器初始化失败
} BalanceFault;


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
