//
// Created by 15902 on 2026/11/2.
//

#ifndef F1_BALANCE_DMP_CONFIG_H
#define F1_BALANCE_DMP_CONFIG_H

/*
 * 控制步长 0.005 秒=5ms，与 TIM1 的 200Hz 节拍及传感器采样设置一致。
 */
#define DT 0.005f

#define PWM_MAX 3599U
extern float MDB;        // 电机死区补偿

extern float target_speed;
extern float target_turn;

extern float DKP;
extern float DKD;
extern float VKP;
extern float VKI;
extern float PKP;
extern float TKP;
extern float V_integral_max;
extern float V_output_max;
extern float P_output_max;



#endif //F1_BALANCE_DMP_CONFIG_H
