/*
 * 本文件把 MPU 姿态、编码器、PID 和电机连接起来；具体硬件操作在各驱动文件。
 * 状态编号：0=校准中，1=就绪/手动停止，2=运行，3=保护停机。
 * 正常控制周期为 5ms；只有 RUNNING 才向电机写入闭环控制量。
 * 上电校准完成自动启动；PB1 低电平按下切换启停；倾倒停机允许扶正后恢复。
 */
#include "balance.h"
#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "dmp_attitude.h"
#include "pid.h"
#include "uart_cmd.h"
#include "i2c.h"
#include "tim.h"
#include <math.h>

/*
 * volatile 表示状态可能被中断修改，每次使用都要重新读取；它不等于线程锁。
 * static 变量只在本文件内可见；这些静态变量在多次函数调用之间保留数值。
 * angle_zero：校准时的直立参考角，单位度；gyro_zero：X 轴零偏，单位原始计数。
 * target_speed：每 5ms 的目标平均编码器计数；target_turn：目标两轮计数之差。
 */
volatile BalanceState balance_state = BALANCE_CALIBRATING;                          //当前车体状态：校准中/就绪/运行/故障
volatile BalanceFault balance_fault = FAULT_NONE;                                   //当前故障原因：无/IMU错误/倾倒/节拍超时/硬件错误。只有状态是FAULT时才有意义
static volatile uint8_t pending_ticks;                                              //TIM1中断每5ms标志位，一次中断++
static uint16_t calibration_count;                                                  //校准阶段已经采了多少个样本。采满200个就判断稳定性
static float angle_sum, gyro_sum, angle_zero, gyro_zero;
static uint32_t last_sample_ms;                                                     //最后一次成功读到IMU的时间戳(ms)
static uint8_t hardware_ready;                                                      //电机和编码器初始化是否成功。0=没好，1=好了
static uint8_t sample_valid;                                                        //最近一次读IMU是否成功。1=数据有效，0=读失败
static HAL_StatusTypeDef last_read_status = HAL_BUSY;                               //最近一次读IMU的HAL返回值
/*
 * 是否允许"扶正后自动恢复运行"。
 * 只有运行中触发的FAULT_TILT才置1；其他故障锁死不恢复
 */
static volatile uint8_t tilt_resume_pending;
/*
 * 倾倒恢复连续稳定计数。
 * 扶正后要连续40拍(200ms)都满足恢复条件才真正重启，这个计数器数到40就恢复
 */
static uint16_t tilt_resume_count;

/*
 * 存放某次的平均值数组，存放倾斜角某次的偏差平方和数组，存放一半时候的平均值数组
 * 数组内单位分别是度、度/秒、g；用于计算方差。
 * SD 检查抖动大小（角度，角速度，加速度），DRIFT 比较前后半段均值，防止缓慢移动也被当成静止。
 * cal_last_block 对应串口 LAST，记录最近一次校准被拒绝的原因。
 */
static float cal_mean[7], cal_m2[7], cal_first_half[7];
static float cal_sd_angle, cal_sd_gyro, cal_sd_accel;
static float cal_drift_angle, cal_drift_gyro, cal_drift_accel;
/* 标记：本次校准有没有收满过一次完整200样本窗口？
 * 串口打印 CAL WINDOW 时才用，没算过就不打印，避免输出脏数据
 */
static uint8_t cal_window_checked;
/* 最近一次校准被拒绝的原因字符串，比如 "ANGLE_NOISE"/"WHEEL_MOVED"/"GYRO_DRIFT"
 * 串口输入 CAL LAST 就能看到，用来定位"为什么车一直校准不成功"
 */
static const char *cal_last_block = "NONE";
/* 校准窗口被作废重收的累计次数。
 * 如果这个数很大，说明你上电后一直在动车/桌子在震/手在抖
 */
static uint32_t cal_restarts;
#define KEY_DEBOUNCE_MS 30U
/* key_raw_pressed：按键当前读到的原始电平(0=按下接地)
 * key_stable_pressed：消抖后稳定的电平，持续30ms没变才更新它
 */
static uint8_t key_raw_pressed, key_stable_pressed;
/* 记录原始电平最后一次变化的时刻(ms)，用来算"已经持续了多久没变" */
static uint32_t key_changed_ms;
/*
 * 上电自动启动是一次性请求；手动停止或显式 CAL 会取消它。只有初始化成功后才置1，其他时间都是拉低了
 * 它与 tilt_resume_pending 分开：后者只用于“原本正在运行，后来因倾倒停机”的恢复。
 */
static volatile uint8_t boot_auto_start_pending;

/*
 * 丢弃当前校准窗口，重新开始采。
 * 什么时候调它：
 *   - 校准中发现动了（角度超15°/陀螺超20°/轮子转了）
 *   - 200个采完但统计不合格（SD/DRIFT超限）
 *   - 手动发CAL命令重新校准
 * 这里不清除最近一次失败的诊断信息，便于在串口定位反复校准的原因。
 */
static void reset_calibration_window(void)
{
    calibration_count = 0U;
    angle_sum = gyro_sum = 0.0f;
}
/*
 * 先排除原始角速度明显越界。允许一定直流零偏，后面用静止均值扣除。
 * 三个轴都检查，避免车体沿另一个轴转动时被直接接受。
 */
static uint8_t calibration_gyro_plausible(void)
{
    return fabsf((float)raw->gx) < CAL_GYRO_ABS_LIMIT
        && fabsf((float)raw->gy) < CAL_GYRO_ABS_LIMIT
        && fabsf((float)raw->gz) < CAL_GYRO_ABS_LIMIT;
}
/*
 * accel_angle为加速度计算出的角度
 * 每个有效样本调用一次。用 Welford 方法逐次更新均值和离均差平方和，
 * 不需要把 200 组样本全部存在 RAM。第 100 个样本时保存前半段均值。
 * 另外累加 X 角度和原始 gx，最终分别得到机械零点和陀螺仪零偏。
 */
static void calibration_accumulate(float accel_angle)
{
    float values[7] = {accel_angle, raw->gx / 131.0f, raw->gy / 131.0f,
                      raw->gz / 131.0f, raw->ax / 16384.0f,
                      raw->ay / 16384.0f, raw->az / 16384.0f};
    ++calibration_count;
    for (unsigned i = 0U; i < 7U; ++i) {
        if (calibration_count == 1U) {
            cal_mean[i] = values[i];
            cal_m2[i] = 0.0f;
            cal_first_half[i] = values[i];
        } else {
            float delta = values[i] - cal_mean[i];                          //除第一轮开始values已经是最新值，cal_mean为上一轮values值
            cal_mean[i] += delta / calibration_count;                       //前后两平均值
            cal_m2[i] += delta * (values[i] - cal_mean[i]);                 //两个数的分散度累加
        }
        if (calibration_count == CALIBRATION_SAMPLES / 2U)                  //计入前100次的平均值放到数组中
            cal_first_half[i] = cal_mean[i];
    }
    angle_sum += accel_angle;
    gyro_sum += raw->gx;
}
/*
 * 求角度标准差差cal_sd_angle表示角度摆动幅度,后面同理
 * 收满 200 个样本后才判断整段是否稳定，小幅单点波动不会立刻清零。
 * 总体均值=(前半均值+后半均值)/2，所以两半均值之差=2*(总体均值-前半均值)。
 * 任一噪声或漂移指标超限就返回 0，并记录具体原因；合格返回 1。
 */
static uint8_t calibration_window_stable(void)
{
    float variance = cal_m2[0] / CALIBRATION_SAMPLES;
    cal_window_checked = 1U;
    if (variance < 0.0f) variance = 0.0f;                                   //方差小于0则为0
    cal_sd_angle = sqrtf(variance);
    /*两半平均值之差*/
    cal_drift_angle = 2.0f * fabsf(cal_mean[0] - cal_first_half[0]);
    cal_sd_gyro = cal_sd_accel = cal_drift_gyro = cal_drift_accel = 0.0f;
    for (unsigned i = 1U; i < 7U; ++i) {
        float variance_axis = cal_m2[i] / CALIBRATION_SAMPLES;
        if (variance_axis < 0.0f) variance_axis = 0.0f;
        float sd = sqrtf(variance_axis);
        float drift = 2.0f * fabsf(cal_mean[i] - cal_first_half[i]);
        if (i <= 3U) {
            /*轮3次把xyz的偏差值对比，看谁最大，最后角速度取最大偏差值*/
            if (sd > cal_sd_gyro) cal_sd_gyro = sd;
            if (drift > cal_drift_gyro) cal_drift_gyro = drift;
        } else {
            /*同样，后面3轮为加速度对比*/
            if (sd > cal_sd_accel) cal_sd_accel = sd;
            if (drift > cal_drift_accel) cal_drift_accel = drift;
        }
    }
    if (cal_sd_angle > CAL_ANGLE_SD_LIMIT) cal_last_block = "ANGLE_NOISE";
    else if (cal_sd_gyro > CAL_GYRO_SD_LIMIT) cal_last_block = "GYRO_NOISE";
    else if (cal_sd_accel > CAL_ACCEL_SD_LIMIT) cal_last_block = "ACCEL_NOISE";
    else if (cal_drift_angle > CAL_ANGLE_DRIFT_LIMIT) cal_last_block = "ANGLE_DRIFT";
    else if (cal_drift_gyro > CAL_GYRO_DRIFT_LIMIT) cal_last_block = "GYRO_DRIFT";
    else if (cal_drift_accel > CAL_ACCEL_DRIFT_LIMIT) cal_last_block = "ACCEL_DRIFT";
    else { cal_last_block = "NONE"; return 1U; }
    return 0U;
}

/*
 * 启停或保护切换时清除积分、上一次输出及速度滤波状态，
 * 避免重新启动后继续使用停机前的积分和电机控制量。
 */
static void reset_controllers(void)
{
    pid_balance.integral = pid_balance.output = 0.0f;
    pid_speed.integral = pid_speed.output = 0.0f;
    pid_position.error = pid_position.integral = pid_position.output = 0.0f;
    position_counts = position_speed_target = 0.0f;
    pid_turn.integral = pid_turn.output = 0.0f;
    balance_pwm = speed_filt = 0.0f;
}

/*
 * 保护停机统一入口：关闭两路 PWM、拉低 STBY、清除控制器状态。
 * 只有运行中触发的 FAULT_TILT 才保留自动恢复意图；其他故障保持锁定。
 * PRIMASK 保存/恢复原来的中断屏蔽状态，使嵌套调用也不会误开中断。
 */
static void fault_stop(BalanceFault reason)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    /* 如果只是角度大了点，但现在本来就没在运行，那就别停了 */
    if (reason == FAULT_TILT && balance_state != BALANCE_RUNNING) {
        __set_PRIMASK(saved);
        return;
    }
    /* 倾斜角大于45度和运行生效时候才可以扶正自动复位 */
    tilt_resume_pending = (uint8_t)(reason == FAULT_TILT && balance_state == BALANCE_RUNNING);
    tilt_resume_count = 0U;
    boot_auto_start_pending = 0U;
    Motor_Disable();
    if (reason == FAULT_TILT) target_speed = target_turn = 0.0f;
    balance_fault = reason;
    balance_state = BALANCE_FAULT;
    reset_controllers();
    __set_PRIMASK(saved);
}

/*
 * 上电判断初始化函数
 * 显式 CAL 或上电初始化都会经过这里：先停电机，再初始化 IMU，开始收集样本。
 * 本函数只“开始校准”，不会阻塞等待收满 200 个样本；采样由 Balance_Poll 逐次完成。
 * 显式 CAL 不自动启动；Balance_Init 会在它成功后单独设置上电自动启动请求。
 */
HAL_StatusTypeDef Balance_Calibrate(void)
{
    uint32_t saved;
    /* Explicit CAL cancels both automatic startup and tilt recovery. */
    boot_auto_start_pending = 0U;
    tilt_resume_pending = 0U;
    tilt_resume_count = 0U;
    Motor_Disable();
    if (!hardware_ready) { fault_stop(FAULT_HARDWARE); return HAL_ERROR; }
    balance_state = BALANCE_CALIBRATING;
    balance_fault = FAULT_NONE;
    target_speed = target_turn = 0.0f;
    reset_calibration_window();
    cal_restarts = 0U;
    cal_last_block = "NONE";
    cal_window_checked = 0U;
    sample_valid = 0U;
    last_read_status = HAL_BUSY;
    reset_controllers();
    if (MPU6050_Init(&hi2c2) != HAL_OK) {
        fault_stop(FAULT_IMU); return HAL_ERROR;
    }
    saved = __get_PRIMASK(); __disable_irq();
    pending_ticks = 0U; __set_PRIMASK(saved);
    return HAL_OK;
}

/*
 * 初始化入口：记录按键实际电平，设置三个控制器，启动电机 PWM 和两路编码器，
 * 最后开始 IMU 校准。初始化 PWM 外设不代表电机已使能，STBY 此时仍为低。
 * 默认直立 Kp=60、Kd=0，速度和转向增益均为 0；串口调参只改 RAM，复位会恢复这里的值。
 */
HAL_StatusTypeDef Balance_Init(void)
{
    HAL_StatusTypeDef status;
    boot_auto_start_pending = 0U;
    tilt_resume_pending = 0U;
    tilt_resume_count = 0U;
    hardware_ready = 0U;
    balance_state = BALANCE_CALIBRATING;
    balance_fault = FAULT_NONE;
    pending_ticks = 0U;
    /* A key held through reset is the initial state, not a new press event. */
    key_raw_pressed = key_stable_pressed = (uint8_t)
        (HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_RESET);
    key_changed_ms = HAL_GetTick();
    PID_Init(&pid_balance, DKP, 0.0f, DKD, 0.0f, (float)PWM_MAX);
    PID_Init(&pid_speed, VKP, VKI, 0.0f, V_integral_max, V_output_max);
    PID_Init(&pid_position, PKP, 0.0f, 0.0f, 0.0f, P_output_max);
    PID_Init(&pid_turn, TKP, 0.0f, 0.0f, 0.0f, (float)PWM_MAX * 0.5f);
    if (motor_Init() != HAL_OK || EncoderA_Init() != HAL_OK || EncoderB_Init() != HAL_OK) {
        fault_stop(FAULT_HARDWARE); return HAL_ERROR;
    }
    hardware_ready = 1U;
    status = Balance_Calibrate();
    if (status == HAL_OK) boot_auto_start_pending = 1U;
    return status;
}

/*
 * 由 TIM1 的 HAL 回调调用，运行在中断里。只提交节拍，不在这里读 I2C 或打印。
 * 主循环若积压两次未处理节拍，立即关闭驱动并记录超时故障。
 * 因此主循环不能用长延时、整屏 OLED 刷新等操作占住控制任务。
 */
void Balance_OnTickISR(void)
{
    if (pending_ticks < 255U) ++pending_ticks;
    if (balance_state == BALANCE_RUNNING && pending_ticks >= 2U) {
        Motor_Disable();
        tilt_resume_pending = 0U;
        balance_fault = FAULT_OVERRUN;
        balance_state = BALANCE_FAULT;
    }
}

/*enable 0 and 1 is stop and start. from_key to 0 and 1 is uart command or key command */
/*
 * 最终启停判定：enable=0 停止并取消自动启动/恢复；enable=1 请求启用。
 * 启用要求 READY、姿态接近零点、传感器样本不超过 20ms。
 * 串口或自动启动则要求实体键松开。所有启用仍须通过姿态和数据时效检查。
 */
static uint8_t arm_output(uint8_t enable, uint8_t from_key)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    if (!enable) {
        boot_auto_start_pending = 0U;
        tilt_resume_pending = 0U;
        tilt_resume_count = 0U;
        Motor_Disable();
        if (balance_state == BALANCE_FAULT && balance_fault == FAULT_TILT) {
            balance_state = BALANCE_READY;
            balance_fault = FAULT_NONE;
        }
        /* 按键按下或者判定不使能，再正常状态下停止运行且清除积分 */
        if (balance_state == BALANCE_RUNNING) balance_state = BALANCE_READY;
        reset_controllers();
        target_speed = target_turn = 0.0f;
        __set_PRIMASK(saved); return 1U;
    }
    /* 没有就绪，数据超20ms没关系，零角偏差10度，按键按着或者软件标志位没启动都不算启动成功，否则就启动成功退出 */
    if (balance_state != BALANCE_READY || HAL_GetTick() - last_sample_ms > 20U
        || fabsf(angle - angle_zero) > 10.0f
        || (!from_key && HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_RESET)) {
        __set_PRIMASK(saved); return 0U;
    }
    reset_controllers();
    tilt_resume_pending = 0U;
    tilt_resume_count = 0U;
    pending_ticks = 0U;
    balance_state = BALANCE_RUNNING;
    Motor_Enable();
    __set_PRIMASK(saved); return 1U;
}

/* ARM for enable is only uart command  */
uint8_t Balance_Arm(uint8_t enable)
{
    return arm_output(enable, 0U);
}

/*
 * 不堵塞，用轮询
 * PB1 上拉：松开=1，按下接地=0。电平变化后持续 30ms 才认定按键状态改变。
 * 只有“松开到按下”的事件切换启停；释放只为下次按下做准备，长按不会反复切换。
 * 校准中按下取消上电自动启动；倾倒等待中按下取消自动恢复，保持手动停止。
 */
void Balance_KeyPoll(void)
{
    /* 按下pressed为1 */
    uint8_t pressed = (uint8_t)
        (HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_RESET);
    uint32_t now = HAL_GetTick();
    if (pressed != key_raw_pressed) {
        key_raw_pressed = pressed;
        key_changed_ms = now;
    }
    /* 两个条件任一满足就返回：
     *   ① 距上次变化不到30ms → 还在消抖期，不认
     *   ② 新电平和稳定电平一样 → 没变化，不认
     */
    if ((uint32_t)(now - key_changed_ms) < KEY_DEBOUNCE_MS
        || pressed == key_stable_pressed) return;
    /* 超30ms，读取完成更新key状态 */
    key_stable_pressed = pressed;
    if (!pressed) return;
    /* 情况1：校准中按下 + 上电自启还在等待 → 取消自动启动
     * 情况2：运行中按下，或倾倒恢复等待中按下 → 停止
     * 情况3：就绪状态按下 → 请求启动
     * 情况4：其他状态按下 → 启动被拒，打印原因和诊断
     */
    if (balance_state == BALANCE_CALIBRATING && boot_auto_start_pending) {
        boot_auto_start_pending = 0U;
        UART_Printf("KEY STOP: boot auto-start canceled; press PB1 after READY to start\r\n");
    } else if (balance_state == BALANCE_RUNNING
               || (balance_state == BALANCE_FAULT && balance_fault == FAULT_TILT && tilt_resume_pending)) {
        (void)arm_output(0U, 1U);
        UART_Printf("KEY STOP; press PB1 again to start\r\n");
    } else if (balance_state == BALANCE_READY && arm_output(1U, 1U)) {
        UART_Printf("KEY START; press PB1 again to stop\r\n");
    } else {
        UART_Printf("KEY START blocked: require READY, upright and fresh sensor; release and retry\r\n");
        Balance_PrintDiagnostics();
    }
}

/*
 * 主循环反复调用，但只有收到 TIM1 节拍时才执行一次采样/控制。
 * 建议按下列顺序阅读：取节拍 -> 读 IMU/编码器 -> 校准分支 -> 倾倒恢复分支
 * -> 运行状态检查 -> 45 度停机 -> 速度 PI -> 直立 PD -> 转向 P -> 两路电机输出。
 */
void Balance_Poll(void)
{
    uint8_t due;
    uint32_t saved = __get_PRIMASK();
    __disable_irq(); due = pending_ticks; pending_ticks = 0U; __set_PRIMASK(saved);
    /* Keep observing tilt while outputs are off so upright recovery is possible. */
    if (!due || (balance_state == BALANCE_FAULT && balance_fault != FAULT_TILT)) return;
    if (due > 1U && balance_state == BALANCE_RUNNING) { fault_stop(FAULT_OVERRUN); return; }
    last_read_status = MPU6050_ReadAll(&hi2c2, raw);
    if (last_read_status != HAL_OK) {
        sample_valid = 0U;
        fault_stop(FAULT_IMU);
        UART_Printf("IMU DETAIL stage=%u dmp=%u HAL=%u I2CERR=0x%08lX\r\n",
                    MPU6050_LastReadStage(), DMP_AttitudeLastError(),
                    (unsigned)last_read_status, (unsigned long)HAL_I2C_GetError(&hi2c2));
        return;
    }
    sample_valid = 1U;
    last_sample_ms = HAL_GetTick();
    /*
     * 编码器速度实际是“本次与上次读取之间的计数差”。正常 5ms 周期下，
     * 单位是计数/5ms；不是转速 rpm。raw_gx 已扣除零偏并换成度/秒，名字虽有 raw 也不是原始计数。
     */
    speedA = EncoderA_GetSpeed(); speedB = EncoderB_GetSpeed();
    speed_avg = (speedA + speedB) * 0.5f;
    raw_gx = MPU6050_GetGyroX(raw, gyro_zero);
    /*
     * 上电校准阶段电机始终关闭。先检查角度、原始角速度、重力和轮动，
     * 再累积 200 个样本并检验统计稳定性。gravity_sq 是加速度模长的平方，单位 g²。
     * 校准不合格会重新收集；不会因为等待时间到了就绕过条件强行启动。
     */
    if (balance_state == BALANCE_CALIBRATING) {
        float accel_angle = MPU6050_AccelAngleX(raw);
        float gravity_sq = ((float)raw->ax * raw->ax + (float)raw->ay * raw->ay
                          + (float)raw->az * raw->az) / (16384.0f * 16384.0f);
        angle = accel_angle;
        /* Bad orientation, motion or sampling gaps still invalidate the window. */
        const char *block = NULL;
        if (due != 1U || pending_ticks != 0U) block = "SAMPLE_GAP";
        else if (fabsf(accel_angle) >= 15.0f) block = "ANGLE_RANGE";
        else if (!calibration_gyro_plausible()) block = "GYRO_RANGE";
        else if (gravity_sq <= 0.64f || gravity_sq >= 1.44f) block = "GRAVITY_RANGE";
        else if (speedA != 0.0f || speedB != 0.0f) block = "WHEEL_MOVED";
        if (block != NULL) {
            cal_last_block = block;
            if (calibration_count != 0U) ++cal_restarts;
            reset_calibration_window();
            return;
        }
        calibration_accumulate(accel_angle);
        if (calibration_count >= CALIBRATION_SAMPLES) {
            if (!calibration_window_stable()) {
                ++cal_restarts;
                reset_calibration_window();
                return;
            }
            /*
             * 把当前静止直立姿态的平均角记为零点，而不是要求传感器显示恰好为 0 度。
             * 滤波器也从该角度开始，避免启动时从旧角度慢慢收敛。
             * gyro_zero 保留原始计数单位，后面先减偏置再除以 131 转成度/秒。
             */
            angle_zero = angle_sum / CALIBRATION_SAMPLES;
            gyro_zero = gyro_sum / CALIBRATION_SAMPLES;
            MPU6050_ResetAngle(angle_zero);
            angle = angle_zero;
            raw_gx = MPU6050_GetGyroX(raw, gyro_zero);
            balance_state = BALANCE_READY;
            UART_Printf("CALIBRATED angle_zero=%.2f gyro_zero=%.1f counts\r\n", angle_zero, gyro_zero);
            /* Consume the boot request exactly once; STOP or fault recovery never re-arms it. */
            if (boot_auto_start_pending) {
                boot_auto_start_pending = 0U;
                if (!key_raw_pressed && !key_stable_pressed && arm_output(1U, 0U))
                    UART_Printf("AUTO START: RUNNING; press PB1 to stop\r\n");
                else UART_Printf("READY: auto-start blocked; release and press PB1 to start\r\n");
            } else UART_Printf("READY: press PB1 to start\r\n");
        }
        return;
    }
    angle = MPU6050_GetAngleX(raw, raw_gx);
    if (!isfinite(angle) || !isfinite(raw_gx)) { fault_stop(FAULT_IMU); return; }
    /*
     * 倾倒停机时继续读取姿态，但保持电机关闭。
     * 只有原本的运行请求尚在、角度回到 ±10 度、数据/重力合理、轮子不动、按键松开，
     * 并连续满足 40 个采样周期，才恢复闭环；沿用原零点，无需重新做上电校准。
     * 同时检查加速度角和滤波角，避免滤波滞后时提前开启仍然倾倒的车体。
     */
    if (balance_state == BALANCE_FAULT && balance_fault == FAULT_TILT) {
        float accel_angle = MPU6050_AccelAngleX(raw);
        float gravity_sq = ((float)raw->ax * raw->ax + (float)raw->ay * raw->ay
                          + (float)raw->az * raw->az) / (16384.0f * 16384.0f);
        /* Hysteresis plus consecutive fresh samples avoids restart at the fall boundary.
         * Check raw acceleration too, so filter lag cannot enable a still-fallen car. */
        if (!tilt_resume_pending || due != 1U || pending_ticks != 0U
            || fabsf(angle) > TILT_RESUME_DEG || fabsf(angle - angle_zero) > TILT_RESUME_DEG
            || fabsf(accel_angle) > TILT_RESUME_DEG || fabsf(accel_angle - angle_zero) > TILT_RESUME_DEG
            || fabsf(raw_gx) >= TILT_RESUME_GYRO
            || gravity_sq <= 0.64f || gravity_sq >= 1.44f
            || speedA != 0.0f || speedB != 0.0f
            || key_raw_pressed || key_stable_pressed
            || HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_RESET) {
            tilt_resume_count = 0U;
            return;
        }
        if (++tilt_resume_count >= TILT_RESUME_SAMPLES) {
            uint8_t resumed = 0U;
            saved = __get_PRIMASK(); __disable_irq();
            if (balance_state == BALANCE_FAULT && balance_fault == FAULT_TILT
                && tilt_resume_pending && pending_ticks == 0U) {
                balance_state = BALANCE_READY;
                balance_fault = FAULT_NONE;
                resumed = arm_output(1U, 0U);
                if (!resumed) {
                    balance_state = BALANCE_FAULT;
                    balance_fault = FAULT_TILT;
                    tilt_resume_count = 0U;
                }
            } else tilt_resume_count = 0U;
            __set_PRIMASK(saved);
            if (resumed) UART_Printf("TILT RESUME: RUNNING; press PB1 to stop\r\n");
        }
        return;
    }
    if (balance_state != BALANCE_RUNNING) return;
    /*
     * 防倒判断在 PID 和电机输出之前执行：显示角绝对值或相对零点偏差任一达到 45 度即停机。
     * 这里判断的是沿现有安装方向计算的 X 轴滤波角；STBY 关闭驱动，不是切断电池电源。
     */
    if (fabsf(angle) >= TILT_STOP_DEG || fabsf(angle - angle_zero) >= TILT_STOP_DEG) {
        fault_stop(FAULT_TILT);
        UART_Printf("TILT STOP: ANGLE=%.2f ZERO=%.2f LIMIT=%.0f; motors OFF; hold upright to resume\r\n",
                    angle, angle_zero, TILT_STOP_DEG);
        return;
    }
    /*
     * 速度先做一阶低通：新滤波值=0.9*旧滤波值+0.1*当前平均轮速，减轻计数抖动。
     * 速度外环 PI 输出的是“目标角度修正量 offset”（度），不是直接加到电机的 PWM。
     */
    speed_filt += 0.1f * (speed_avg - speed_filt);
    float speed_target = PID_PositionUpdate(speed_avg, target_speed);
    float offset = pid_pi(&pid_speed, speed_filt, speed_target);
    /*
     * 直立内环 PD：让实测角 angle 跟随 angle_zero+offset，输出带符号的 PWM 控制量。
     * Kd 使用陀螺仪角速度做阻尼；当前默认 Kd=0，需要实车调参才能形成有效阻尼。
     */
    balance_pwm = pid_pd(&pid_balance, angle, raw_gx, angle_zero + offset);
    /*
     * 转向环 P 根据两轮速度差产生 turn。最终 A=直立量+turn，B=直立量-turn。
     * 这样共同分量用于前后平衡，差动分量用于两轮转向，最终仍由 motor.c 分别限幅。
     */
    float turn = pid_p(&pid_turn, speedA - speedB, target_turn);

    if (!isfinite(balance_pwm) || !isfinite(turn)) { fault_stop(FAULT_IMU); return; }
    /* The timer ISR may have faulted during the read/calculation. Never re-enable here. */
    if (balance_state == BALANCE_RUNNING) {
        MotorA_SetSpeed(balance_pwm + turn);
        MotorB_SetSpeed(balance_pwm - turn);
    }
}

/* Observe raw counters without consuming encoder deltas or changing motor state.
 * Called from the main loop, never from a timer interrupt. */
/*
 * 串口诊断在主循环中执行。DBG 看状态/样本，CTRL 看启动意图和倾角偏差，
 * CAL 的 LAST 看最近校准失败条件，CAL WINDOW 看完整窗口的统计量。
 * IO 的 ENC_A/B 是原始定时器累计值；PWM_A/B 是比较寄存器值；STBY 是驱动使能脚电平。
 * 诊断读取累计计数而不调用 GetSpeed，所以不会“吃掉”下一次控制所需的编码器增量。
 */
void Balance_PrintDiagnostics(void)
{
    BalanceState state = balance_state;
    const char *name = "UNKNOWN";
    uint32_t pwm_a = 0U, pwm_b = 0U;
    switch (state) {
    case BALANCE_CALIBRATING: name = "CALIBRATING"; break;
    case BALANCE_READY: name = "READY"; break;
    case BALANCE_RUNNING: name = "RUNNING"; break;
    case BALANCE_FAULT: name = "FAULT"; break;
    default: break;
    }
    UART_Printf("DBG STATE=%s(%u) FAULT=%u SAMPLE=%u AGE=%lu IMU=%u ANGLE=%.2f ZERO=%.2f\r\n",
                name, (unsigned)state, (unsigned)balance_fault, (unsigned)sample_valid,
                (unsigned long)(HAL_GetTick() - last_sample_ms), (unsigned)last_read_status,
                angle, angle_zero);
    if (state == BALANCE_FAULT && balance_fault == FAULT_IMU)
        UART_Printf("IMU DETAIL stage=%u dmp=%u HAL=%u I2CERR=0x%08lX\r\n",
                    MPU6050_LastReadStage(), DMP_AttitudeLastError(),
                    (unsigned)last_read_status, (unsigned long)HAL_I2C_GetError(&hi2c2));
    UART_Printf("CTRL BOOT_AUTO=%u TILT_LIMIT=%.0f REL=%.2f\r\n",
                (unsigned)boot_auto_start_pending, TILT_STOP_DEG, angle - angle_zero);
    UART_Printf("POS=%.1f VPS=%.2f PKP=%.4f PLIM=%.2f\r\n",
                position_counts, position_speed_target, pid_position.kp, pid_position.output_max);
    if (state == BALANCE_FAULT && balance_fault == FAULT_TILT)
        UART_Printf("TILT WAIT: AUTO=%u RECOVER=%u/%u UPRIGHT=%.0f; hold upright 200ms; PB1 cancels\r\n",
                    (unsigned)tilt_resume_pending, (unsigned)tilt_resume_count,
                    (unsigned)TILT_RESUME_SAMPLES, TILT_RESUME_DEG);
    if (state == BALANCE_CALIBRATING && sample_valid) {
        float accel_angle = MPU6050_AccelAngleX(raw);
        float gravity_sq = ((float)raw->ax * raw->ax + (float)raw->ay * raw->ay
                          + (float)raw->az * raw->az) / (16384.0f * 16384.0f);
        UART_Printf("CAL=%u/200 ANGLE_OK=%u GYRO_OK=%u GRAVITY_OK=%u RESTARTS=%lu LAST=%s\r\n",
                    (unsigned)calibration_count, (unsigned)(fabsf(accel_angle) < 15.0f),
                    (unsigned)calibration_gyro_plausible(),
                    (unsigned)(gravity_sq > 0.64f && gravity_sq < 1.44f),
                    (unsigned long)cal_restarts, cal_last_block);
        if (cal_window_checked)
            UART_Printf("CAL WINDOW: SD_A=%.2f SD_G=%.2f SD_ACC=%.3f DRIFT_A=%.2f DRIFT_G=%.2f DRIFT_ACC=%.3f\r\n",
                        cal_sd_angle, cal_sd_gyro, cal_sd_accel,
                        cal_drift_angle, cal_drift_gyro, cal_drift_accel);
    }
    if (htim4.Instance != NULL) {
        pwm_a = __HAL_TIM_GET_COMPARE(&htim4, PWMA_CH);
        pwm_b = __HAL_TIM_GET_COMPARE(&htim4, PWMB_CH);
    }
    UART_Printf("IO ENC_A=%u ENC_B=%u PWM_A=%lu PWM_B=%lu STBY=%u KEY_PRESSED=%u\r\n",
                htim2.Instance != NULL ? (unsigned)(uint16_t)__HAL_TIM_GET_COUNTER(&htim2) : 0U,
                htim3.Instance != NULL ? (unsigned)(uint16_t)__HAL_TIM_GET_COUNTER(&htim3) : 0U,
                (unsigned long)pwm_a, (unsigned long)pwm_b,
                (unsigned)(HAL_GPIO_ReadPin(STBY_PORT, STBY_PIN) == GPIO_PIN_SET),
                (unsigned)(HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_RESET));
    if (state == BALANCE_READY)
        UART_Printf("HINT READY: press PB1 to START; press again to STOP\r\n");
}

/*
 * HAL 的周期中断回调：stm32f1xx_it.c 中的 HAL_TIM_IRQHandler 最终会调用这里。
 * 多个定时器可共用此回调，所以先检查 timer->Instance 是否为 TIM1。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
    if (timer->Instance == TIM1) Balance_OnTickISR();
}
