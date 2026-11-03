//
// Created by 15902 on 2026/11/2.
//

#ifndef F1_BALANCE_DMP_CONFIG_H
#define F1_BALANCE_DMP_CONFIG_H

/*控制步长 0.005 秒=5ms，与 TIM1 的 200Hz 节拍及传感器采样设置一致。*/
#define DT 0.005f

#define PWM_MAX 3599U

/* 校准样本总数：200个。采样率200Hz(5ms/次)，200×5ms=1秒，即上电后要静止1秒 */
#define CALIBRATION_SAMPLES 200U
/* 倾倒停机阈值：实测角(或相对零点偏差)的绝对值≥45°就立刻切电机进FAULT_TILT */
#define TILT_STOP_DEG 45.0f
/* 倾倒恢复阈值：扶正后|angle|要回到10°以内才允许自动重启，和45°停机形成滞回，避免在倒地边缘反复启停 */
#define TILT_RESUME_DEG 10.0f
/* 恢复时允许的最大角速度：|陀螺X|≥20°/s说明还在晃，不允许恢复 */
#define TILT_RESUME_GYRO 20.0f
/* 恢复所需连续满足的样本数：40个。5ms×40=200ms，即扶正后要稳定保持200ms才真启动 */
#define TILT_RESUME_SAMPLES 40U
/* 前后是一半前和一半后 */
#define CAL_GYRO_ABS_LIMIT (20.0f * 131.0f)             //瞬间角速度不能超过 20°/s
#define CAL_GYRO_SD_LIMIT 2.0f                          //取零点角速度最大偏移量
#define CAL_ACCEL_SD_LIMIT 0.03f                        //取零点角加速度最大偏移量
#define CAL_ANGLE_SD_LIMIT 1.0f                         //取零点角度最大偏移量
#define CAL_GYRO_DRIFT_LIMIT 1.0f                       //取零点角速度前后最大差
#define CAL_ACCEL_DRIFT_LIMIT 0.025f                    //取零点角加速度前后最大差
#define CAL_ANGLE_DRIFT_LIMIT 0.5f                      //取零点角度前后最大差

extern float MDB;        // 电机死区补偿

extern float target_speed;      //目标速度
extern float target_turn;       //目标转速度

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
