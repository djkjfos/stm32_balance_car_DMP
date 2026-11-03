#include "pid.h"
#include <math.h>

pid_t pid_balance;
pid_t pid_speed;
pid_t pid_position;
pid_t pid_turn;
/* -------------------------------相对上次启动、倾倒恢复或手动行驶结束时的位置-------------------------------*/
float position_counts, position_speed_target;

void PID_Init(pid_t *pid ,float kp, float ki, float kd, float integral_max, float output_max){
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_max = integral_max;
    pid->output_max = output_max;
    pid->error = 0;
    pid->integral = 0;
    pid->output = 0;
}


/*
 * wheel_delta 是本周期两轮平均编码器增量；commanded_speed 是 ASP 速度指令。
 * 有行驶指令时跟随指令并给加速值；指令为零时，按相对位移计算返回速度。
 * 输出值就是pi环的目标速度
 */
float PID_PositionUpdate(float wheel_delta, float commanded_speed)
{
    if (commanded_speed != 0.0f) {
        position_counts = 0.0f;
        pid_position.error = pid_position.output = 0.0f;
        position_speed_target = commanded_speed;
    } else {
        position_counts += wheel_delta;
        position_speed_target = pid_p(&pid_position, position_counts, 0.0f);
    }
    return position_speed_target;
}

/*
 * 直立 PD：输出=Kp*(目标角-当前角)-Kd*陀螺仪角速度。
 * P 按倾斜误差纠正，D 按转动速度提供阻尼.
 * 最后将输出限制在 ±output_max，直立环的输出单位为 PWM 计数。
 */
float pid_pd(pid_t *pid , float angle, float raw_gx, float target_angle){
    pid->error = target_angle - angle;
    pid->output = pid->kp * pid->error + pid->kd * (-raw_gx);
    pid->output = fminf(fmaxf(pid->output, -pid->output_max), pid->output_max);               //浮点数比大小函数
    return pid->output;
}

/*
 * 速度 PI：P 根据目标轮速与当前轮速之差输出，I 逐周期累积速度偏差。
 * 本工程 Ki 按“每次 5ms 调用”定义，计算中没有额外乘 DT，不能直接套用连续时间 Ki,ki = (ki + dt)。
 */
float pid_pi(pid_t *pid, float speed, float target_speed){
    pid->error = target_speed - speed;
    float candidate = pid->integral + pid->ki * pid->error;
    candidate = fminf(fmaxf(candidate, -pid->integral_max), pid->integral_max);
    float proposed = pid->kp * pid->error + candidate;
/*==================饱和本身不是问题，问题是下次积分是加深饱和还是缓解饱和===================*/
    if ((proposed <= pid->output_max && proposed >= -pid->output_max)                       //proposed在区间内
        || (proposed > pid->output_max && candidate < pid->integral)                        //proposed超出区间,但是积分在减小的方向上走，故下一次积分是缓和饱和
        || (proposed < -pid->output_max && candidate > pid->integral)) {                    //proposed超出区间,但是积分在增大的方向上走，故下一次积分是加深饱和
        pid->integral = candidate;                                                          //即便超出输出最大值但是下次积分是缓和饱和的一样更新，否则积分保持不变。
    }

    pid->output = pid->kp * pid->error + pid->integral;

    pid->output = fminf(fmaxf(pid->output, -pid->output_max), pid->output_max);
    return pid->output;
}

/*===============转向 和位置环共用PID母板：输出=Kp*(目标两轮速度差-实际两轮速度差)，最后限幅===============*/
float pid_p(pid_t *pid, float current, float target){
    pid->error = target - current;
    pid->output = pid->kp * pid->error;
    pid->output = fminf(fmaxf(pid->output, -pid->output_max), pid->output_max);
    return pid->output;
}
