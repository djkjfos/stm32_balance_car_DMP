/*
 * MPU6050 DMP 驱动与 X 轴姿态计算，使用 I2C2（PB10/PB11）。
 * 初始化负责确认型号、配置量程/滤波/采样率并读回校验；控制时每次读 14 个连续字节。
 * DMP 输出四元数，由 dmp_attitude.c 转换为控制角度；本版本只支持 MPU6050。
 */
#include "mpu6050.h"
#include "dmp_attitude.h"
#include "uart_cmd.h"
#include "i2c.h"
#include <math.h>
static MPU6050_Raw_t raw_data;
MPU6050_Raw_t *raw = &raw_data;
static float filtered_angle;
static unsigned last_read_stage;
unsigned MPU6050_LastReadStage(void) { return last_read_stage; }
/*
 * 报错函数
 * 记录当次 HAL 错误及 SCL/SDA 电平。先打印原错误，再尝试另一个地址，
 * 否则后一次 I2C 操作会覆盖前一次 HAL 错误状态，不利于排查。
 */
static void log_failure(I2C_HandleTypeDef *bus, const char *operation,
                        uint8_t reg, HAL_StatusTypeDef status)
{
    UART_Printf("MPU %s reg=0x%02X HAL=%u ERR=0x%08lX SCL=%u SDA=%u\r\n",
                operation, (unsigned)reg, (unsigned)status,
                (unsigned long)HAL_I2C_GetError(bus),
                (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10),
                (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11));
}
/*======================只用于诊断 AD0 可能接高而改变地址的情况。=====================*/
static void probe_alternate_address(I2C_HandleTypeDef *bus)
{
    uint8_t identity = 0U;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(bus, (0x69U << 1),
        MPU6050_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &identity, 1U, 20U);
    UART_Printf("MPU probe addr=0x69 HAL=%u ERR=0x%08lX\r\n",
                (unsigned)status, (unsigned long)HAL_I2C_GetError(bus));
    if (status == HAL_OK)
        UART_Printf("MPU addr=0x69 WHO=0x%02X; check AD0/address\r\n", (unsigned)identity);
}

/*=========================等待总线引脚达到目标电平，单次最长约 2ms=============================*/
static HAL_StatusTypeDef wait_bus_pin(uint16_t pin, GPIO_PinState level)
{
    uint32_t started = HAL_GetTick();
    while (HAL_GPIO_ReadPin(GPIOB, pin) != level) {
        if ((uint32_t)(HAL_GetTick() - started) >= 2U) return HAL_TIMEOUT;
        HAL_Delay(1U);
    }
    return HAL_OK;
}
/*
 * 若初始化发现硬件i2c总线挂死软件模拟补救（开机自检），中途出错直接停机报错即可
 * 初始化无误条件，SDA 、SCL 上拉到高电位，BUSY标志位清零，HAL_I2C_GetState(bus)为READY；
 * 先ODR再初始化是，高转低再转高的过程中 不会同步进行拉，会出现间隔导致可能会误以为出现Start或者Stop
 * 先释放开漏引脚，最多给 9 个 SCL 时钟让被中断的从机释放 SDA（一字节），
 * 再生成相应跳变、恢复 I2C 外设，并重新确认线路高电平及 BUSY 标志。
 */
static HAL_StatusTypeDef prepare_bus(I2C_HandleTypeDef *bus)
{
    GPIO_InitTypeDef pins = {0};                                                        //后续soft模拟引脚初始化
    HAL_StatusTypeDef status = HAL_OK, init_status;
    unsigned pulses = 0U;
    uint8_t scl, sda, busy;
    if (bus != &hi2c2) return HAL_OK;
    scl = (uint8_t)(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET);
    sda = (uint8_t)(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET);
    busy = (uint8_t)(__HAL_I2C_GET_FLAG(bus, I2C_FLAG_BUSY) != RESET);                  //获取I2C：BUSY的标志位，BUSY为1（SET）则是总线别占用
    /*判断总线是否卡死*/
    if (scl && sda && !busy && HAL_I2C_GetState(bus) == HAL_I2C_STATE_READY) return HAL_OK;
    UART_Printf("I2C2 总线恢复 START SCL=%u SDA=%u BUSY=%u\r\n",
                (unsigned)scl, (unsigned)sda, (unsigned)busy);
    status = HAL_I2C_DeInit(bus);                                                       //关闭I2C的通道
    if (status != HAL_OK) {
        UART_Printf("I2C2 recover deinit 失败 HAL=%u\r\n", (unsigned)status);
        return status;
    }
    /*开始初始化soft——I2C引脚*/
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();
    /*初始化先拉高引脚，为了预输入ODR，ODR默认0，初始化前为高阻态，后续在拉高会出现 高转低再转高 的情况*/
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
    pins.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    pins.Mode = GPIO_MODE_OUTPUT_OD;
    pins.Pull = GPIO_NOPULL;
    pins.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &pins);
    HAL_Delay(1U);                                                                      //等待电位稳定，模拟的是2ms一周期
    status = wait_bus_pin(GPIO_PIN_10, GPIO_PIN_SET);
    if (status != HAL_OK) goto restore;
    /* 进行9次脉冲读数，走完一整个读数周期，为从机自动STOP，释放SDA做准备 */
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET && pulses < 9U) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_Delay(1U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        ++pulses;
        status = wait_bus_pin(GPIO_PIN_10, GPIO_PIN_SET);
        if (status != HAL_OK) goto restore;
        HAL_Delay(1U);
    }
    /*9个脉冲后理论SDA已经拉高，如果没拉高就进行restore*/
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET) {
        status = HAL_BUSY;
        goto restore;
    }
    /* SCL和SDA都拉低，准备STOP前工作，后面再SCL拉高后1ms等待再拉高SDA进行停止位模拟*/
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(1U);
    status = wait_bus_pin(GPIO_PIN_11, GPIO_PIN_RESET);
    if (status != HAL_OK) goto restore;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(1U);
    status = wait_bus_pin(GPIO_PIN_10, GPIO_PIN_RESET);
    if (status != HAL_OK) goto restore;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    status = wait_bus_pin(GPIO_PIN_10, GPIO_PIN_SET);
    if (status != HAL_OK) goto restore;
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    status = wait_bus_pin(GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(1U);
restore:
    /* 将soft模拟引脚换回复用开漏，初始化I2C，再进行一次判断SDA是否释放，返回问题 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
    pins.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOB, &pins);
    init_status = HAL_I2C_Init(bus);
    if (status == HAL_OK) status = init_status;
    HAL_Delay(1U);
    scl = (uint8_t)(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET);
    sda = (uint8_t)(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET);
    busy = (uint8_t)(__HAL_I2C_GET_FLAG(bus, I2C_FLAG_BUSY) != RESET);
    if (status == HAL_OK && (!scl || !sda || busy)) status = HAL_BUSY;
    UART_Printf("I2C2 recover %s HAL=%u SCL=%u SDA=%u BUSY=%u PULSES=%u\r\n",
                status == HAL_OK ? "OK" : "FAILED", (unsigned)status,
                (unsigned)scl, (unsigned)sda, (unsigned)busy, pulses);
    /*若最后还是没有抢救回来则返回问题*/
    if (status != HAL_OK)
        UART_Printf("I2C2 not ready; cycle sensor power and check SCL/SDA wiring/pull-ups\r\n");
    return status;
}
/*=====================写一个 8 位寄存器，失败时记录日志；再读一次log_failure=====================*/
static HAL_StatusTypeDef write_reg(I2C_HandleTypeDef *bus, uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(bus, MPU6050_ADDR, reg,
        I2C_MEMADD_SIZE_8BIT, &value, 1U, 20U);
    if (status != HAL_OK) log_failure(bus, "write", reg, status);
    return status;
}

/*
 * 读回配置后只比较 mask 指定的位。
 * expected 我输入REG的值，actual我读取REG里的值对比mask位
 */
static HAL_StatusTypeDef verify_reg(I2C_HandleTypeDef *bus, uint8_t reg,
                                    uint8_t expected, uint8_t mask)
{
    uint8_t actual = 0U;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(bus, MPU6050_ADDR, reg,
        I2C_MEMADD_SIZE_8BIT, &actual, 1U, 20U);
    if (status != HAL_OK) {
        log_failure(bus, "verify read", reg, status);
        return status;
    }
    if ((actual & mask) != (expected & mask)) {
        UART_Printf("MPU verify reg=0x%02X got=0x%02X expected=0x%02X mask=0x%02X\r\n",
                    (unsigned)reg, (unsigned)actual, (unsigned)expected, (unsigned)mask);
        return HAL_ERROR;
    }
    return HAL_OK;
}
/*
 * 启动顺序：等待上电 -> 检查/恢复总线 -> 读 WHO_AM_I -> 复位并唤醒芯片
 * -> 初始寄存器校验 -> 装载 DMP 固件并等待首帧四元数。
 * DMP 初始化将陀螺仪量程改为 ±2000°/s，加速度量程为 ±2g。
 * 初始化阶段调用者已关闭电机。
 */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *bus)
{
    uint8_t who = 0;
    HAL_StatusTypeDef status;
    HAL_Delay(100U);
    status = prepare_bus(bus);
    if (status != HAL_OK) return status;
    status = HAL_I2C_Mem_Read(bus, MPU6050_ADDR, MPU6050_WHO_AM_I,
                            I2C_MEMADD_SIZE_8BIT, &who, 1U, 20U);
    if (status != HAL_OK) {
        log_failure(bus, "read ID addr=0x68", MPU6050_WHO_AM_I, status);
        probe_alternate_address(bus);
        return status;
    }
    UART_Printf("MPU addr=0x68 WHO=0x%02X DMP requires WHO=0x68\r\n", (unsigned)who);
    if (who != MPU6050_ID) {
        UART_Printf("DMP requires MPU6050 WHO=0x68; initialization stopped\r\n");
        return HAL_ERROR;
    }
    // 向PWR_MGMT_1写入0x80，Bit7置1，执行MPU6050芯片软复位，所有寄存器恢复出厂默认值
    if ((status = write_reg(bus, MPU6050_PWR_MGMT_1, 0x80U)) != HAL_OK) return status;
    HAL_Delay(100U);                         // 等待芯片软复位完成，复位需要一定时间稳定

    // PWR_MGMT_1 = 0x01：退出SLEEP休眠模式；选择陀螺仪X轴作为芯片内部时钟源，唤醒传感器
    if ((status = write_reg(bus, MPU6050_PWR_MGMT_1, 0x01U)) != HAL_OK) return status;

    // PWR_MGMT_2 = 0x00：关闭所有轴的断电，加速度X/Y/Z、陀螺仪X/Y/Z 6个轴全部开启工作
    if ((status = write_reg(bus, MPU6050_PWR_MGMT_2, 0x00U)) != HAL_OK) return status;

    // CONFIG寄存器 0x03：配置DLPF数字低通滤波器，设置陀螺仪和加速度计滤波带宽
    if ((status = write_reg(bus, MPU6050_CONFIG, 0x03U)) != HAL_OK) return status;

    // SMPLRT_DIV=0x04 采样率分频；内部基准1kHz，输出采样率 = 1000/(1+4) = 200Hz
    if ((status = write_reg(bus, MPU6050_SMPLRT_DIV, 0x04U)) != HAL_OK) return status;

    // GYRO_CONFIG=0x00：陀螺仪量程设置 ±250°/s；关闭陀螺仪自测试
    if ((status = write_reg(bus, MPU6050_GYRO_CONFIG, 0x00U)) != HAL_OK) return status;

    // ACCEL_CONFIG=0x00：加速度计量程设置 ±2g；关闭加速度自测试
    if ((status = write_reg(bus, MPU6050_ACCEL_CONFIG, 0x00U)) != HAL_OK) return status;

    HAL_Delay(50U);                          // 等待陀螺仪上电、起振稳定

    // 校验PWR_MGMT_1寄存器：掩码0x7F，忽略Bit7复位位；确认休眠关闭、时钟源配置写入成功
    if ((status = verify_reg(bus, MPU6050_PWR_MGMT_1, 0x01U, 0x7FU)) != HAL_OK) return status;

    // 校验PWR_MGMT_2寄存器：掩码0x3F，校验6轴全部处于开启状态
    if ((status = verify_reg(bus, MPU6050_PWR_MGMT_2, 0x00U, 0x3FU)) != HAL_OK) return status;

    // 校验CONFIG寄存器：掩码0x07，只校验DLPF滤波配置的低3位
    if ((status = verify_reg(bus, MPU6050_CONFIG, 0x03U, 0x07U)) != HAL_OK) return status;

    // 校验SMPLRT_DIV，掩码0xFF全部位校验，确认采样分频设置生效
    if ((status = verify_reg(bus, MPU6050_SMPLRT_DIV, 0x04U, 0xFFU)) != HAL_OK) return status;

    // 校验GYRO_CONFIG全部位，确认陀螺仪量程配置写入成功
    if ((status = verify_reg(bus, MPU6050_GYRO_CONFIG, 0x00U, 0xFFU)) != HAL_OK) return status;

    // 校验ACCEL_CONFIG全部位，确认加速度计量程配置写入成功
    if ((status = verify_reg(bus, MPU6050_ACCEL_CONFIG, 0x00U, 0xFFU)) != HAL_OK) return status;

    // 初始化DMP：加载DMP固件，开启硬件四元数姿态解算
    if (DMP_AttitudeInit() != HAL_OK) return HAL_ERROR;

    UART_Printf("MPU6050 DMP configured addr=0x68 Fs=200Hz\r\n");
    return HAL_OK;
}

/*
 * 寄存器数据按高字节、低字节传来，先合成 16 位数，再转为有符号值。
 * 加速度/角速度有正负方向，因此不能把最终原始量当 uint16_t 使用。
 */
static int16_t signed_word(const uint8_t *bytes)
{
    return (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

/* DMP固定2000陀螺仪量程，我的是250量程，差了8倍，后面的角速度原始250量程函数，所以这里先乘8*/
static int16_t gyro_legacy_units(int16_t raw_2000)
{
    int32_t scaled = (int32_t)raw_2000 * 8;
    if (scaled > 32767) scaled = 32767;
    if (scaled < -32768) scaled = -32768;
    return (int16_t)scaled;
}
/*
 * 从 0x3B 开始一次读取：加速度 6 字节、温度 2 字节、角速度 6 字节。
 * 先放入临时 next，只有整次读取成功才更新 sample，失败时不把半包数据当成新样本。
 * 4ms 是 I2C 超时上限；读取结果是否成功由 balance.c 进一步决定是否停机。
 */
HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *bus, MPU6050_Raw_t *sample)
{
    uint8_t bytes[14];
    MPU6050_Raw_t next;
    HAL_StatusTypeDef status;
    last_read_stage = 0U;
    if (sample == NULL) { last_read_stage = 1U; return HAL_ERROR; }
    status = HAL_I2C_Mem_Read(bus, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H,
                             I2C_MEMADD_SIZE_8BIT, bytes, sizeof(bytes), 4U);
    if (status != HAL_OK) { last_read_stage = 1U; return status; }
    next.ax = signed_word(bytes); next.ay = signed_word(bytes + 2);
    next.az = signed_word(bytes + 4); next.temp = signed_word(bytes + 6);
    next.gx = gyro_legacy_units(signed_word(bytes + 8));
    next.gy = gyro_legacy_units(signed_word(bytes + 10));
    next.gz = gyro_legacy_units(signed_word(bytes + 12));
    status = DMP_AttitudePoll();
    if (status != HAL_OK) { last_read_stage = 2U; return status; }
    *sample = next;
    return HAL_OK;
}
/*
 * atan2(ay,az) 得到绕 X 轴的重力角，再乘 180/π=57.2957795 转成度。
 */
float MPU6050_AccelAngleX(const MPU6050_Raw_t *sample)
{
    return atan2f((float)sample->ay, (float)sample->az) * 57.2957795f;
}
/*
 * 设置 DMP 输出角度的软件偏移；校准后用于对齐机械零点。
 */
void MPU6050_ResetAngle(float angle_deg) { DMP_AttitudeResetAngle(angle_deg); }
/*
 * 有新 DMP 四元数时输出 X 轴角度；暂时没有新帧时用角速度预测 5ms。
 * 帧超时由读取函数报告故障；输出单位为度，每个控制周期调用一次。
 */
float MPU6050_GetAngleX(const MPU6050_Raw_t *sample, float gyro_deg_s)
{
    (void)sample;
    filtered_angle = DMP_AttitudeGetAngle(gyro_deg_s);
    return filtered_angle;
}
/*
 * 先扣除静止校准得到的零偏，再除以兼容接口的 131 计数/(°/s)。
 * 实际 DMP 量程是 ±2000°/s，采样值已按 8 倍换算并限幅。
 * 返回度/秒；这里的 bias_counts 不能直接传入角速度单位的数值。
 */
float MPU6050_GetGyroX(const MPU6050_Raw_t *sample, float bias_counts)
{
    return ((float)sample->gx - bias_counts) / 131.0f;
}
