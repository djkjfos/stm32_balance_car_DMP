/*
 * PID 数据结构与函数声明，计算实现见 pid.c。
 * 每个控制器各有一份 pid_t，不能共用同一份积分和输出历史。
 * 本工程调用位置决定单位：直立/转向输出 PWM 计数，速度输出角度修正量。
 */
#ifndef __PID_H_
#define __PID_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * kp/ki/kd：比例/积分/微分增益；error：本次目标值与实测值的差。
 * integral：已保存的积分贡献；integral_max：积分贡献绝对值上限。
 * output：最近一次控制输出；output_max：总输出绝对值上限。
 */
typedef struct{
    float kp;
    float ki;
    float kd;
    float error;
    float integral;
    float integral_max;
    float output;
    float output_max;
}pid_t;

extern pid_t pid_balance;
extern pid_t pid_speed;
extern pid_t pid_position;
extern pid_t pid_turn;

/* 位移单位为平均编码器计数；速度目标单位为计数/5ms。 */
extern float position_counts;
extern float position_speed_target;

void PID_Init(pid_t *pid ,float kp, float ki, float kd, float integral_max, float output_max);
void PID_PositionReset(void);
float PID_PositionUpdate(float wheel_delta, float commanded_speed);
float pid_pd(pid_t *pid , float angle, float raw_gx, float target_angle);
float pid_pi(pid_t *pid, float speed, float target_speed);
float pid_p(pid_t *pid, float current, float target);

#endif
