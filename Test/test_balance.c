/* Builds the ACTUAL Core/user modules against deterministic peripheral mocks.
 * See Test/run_host_tests.py; hardware timing and physical signs require board tests. */
#include "balance.h"
#include "mpu6050.h"
#include "motor.h"
#include "encoder.h"
#include "uart_cmd.h"
#include "pid.h"
#include "tim.h"
#include "i2c.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#ifdef __TINYC__
/* Older Windows CRT lacks strtof; this shim is only linked into host tests. */
float strtof(const char *text,char **end) { return (float)strtod(text,end); }
#endif

MockTimer mock_tim1,mock_tim2,mock_tim3,mock_tim4;
GPIO_TypeDef mock_gpio_b;
TIM_HandleTypeDef htim1={&mock_tim1},htim2={&mock_tim2},htim3={&mock_tim3},htim4={&mock_tim4};
UART_HandleTypeDef huart1,huart2;
I2C_HandleTypeDef hi2c2;
static uint32_t mock_ms;
uint32_t test_i2c_busy_flag;
static HAL_I2C_StateTypeDef test_i2c_state=HAL_I2C_STATE_READY;
static uint32_t held_bus_pins, bus_pin_mode=GPIO_MODE_AF_OD;
static unsigned release_sda_after, bus_rising_edges, bus_deinit_calls, bus_init_calls;
static HAL_StatusTypeDef test_bus_reinit_status=HAL_OK;
static MPU6050_Raw_t sensor;
static HAL_StatusTypeDef sensor_status=HAL_OK,pwm_status=HAL_OK;
static uint8_t who=0x68,divider;
static uint8_t registers[128];
static uint16_t failed_write_reg=0xFFFFU, bad_readback_reg=0xFFFFU;
static unsigned accel_filter_writes, configuration_writes;
static uint8_t *receive_byte;
static uint8_t transmit_pending,inject_overrun;
static char transmitted[32768];
static size_t transmitted_count;
static unsigned checks;

#define CHECK(x) do { ++checks; if (!(x)) { printf("FAIL line %d: %s\n",__LINE__,#x);exit(1); } } while(0)
#define NEAR(x,y,tol) CHECK(fabsf((x)-(y))<(tol))
uint32_t HAL_GetTick(void) { return mock_ms; }
void HAL_Delay(uint32_t delay) { mock_ms+=delay; }
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *bus)
{ (void)bus;return sensor_status==HAL_OK?0U:4U; }
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *bus)
{ (void)bus;return test_i2c_state; }
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *bus)
{ (void)bus;++bus_deinit_calls;test_i2c_state=HAL_I2C_STATE_RESET;return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *bus)
{
    (void)bus;++bus_init_calls;
    test_i2c_state=test_bus_reinit_status==HAL_OK?HAL_I2C_STATE_READY:HAL_I2C_STATE_RESET;
    test_i2c_busy_flag=held_bus_pins!=0;
    return test_bus_reinit_status;
}
void HAL_GPIO_Init(GPIO_TypeDef *port,GPIO_InitTypeDef *pins)
{
    CHECK(port==GPIOB && pins->Pin==(GPIO_PIN_10|GPIO_PIN_11));
    CHECK(pins->Mode==GPIO_MODE_OUTPUT_OD || pins->Mode==GPIO_MODE_AF_OD);
    bus_pin_mode=pins->Mode;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *b,uint16_t a,uint16_t r,uint16_t m,uint8_t *v,uint16_t n,uint32_t t)
{
    (void)b;(void)a;(void)m;(void)n;(void)t;
    CHECK(a==MPU6050_ADDR && m==I2C_MEMADD_SIZE_8BIT && n==1);
    if(sensor_status!=HAL_OK)return sensor_status;
    if(r==failed_write_reg)return HAL_ERROR;
    ++configuration_writes;
    if(r==MPU6050_PWR_MGMT_1 && (*v & 0x80U))memset(registers,0,sizeof(registers));
    else registers[r]=*v;
    if(r==MPU6050_SMPLRT_DIV)divider=*v;
    if(r==MPU6500_ACCEL_CONFIG2)++accel_filter_writes;
    return HAL_OK;
}
static void encode(uint8_t *out,int16_t value)
{ out[0]=(uint8_t)((uint16_t)value>>8);out[1]=(uint8_t)value; }
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *b,uint16_t a,uint16_t r,uint16_t m,uint8_t *out,uint16_t n,uint32_t t)
{
    (void)b;(void)t;
    CHECK((a==MPU6050_ADDR || a==(0x69U<<1)) && m==I2C_MEMADD_SIZE_8BIT);
    if(a==(0x69U<<1))return HAL_ERROR;
    if(sensor_status!=HAL_OK)return sensor_status;
    if(r==MPU6050_WHO_AM_I){CHECK(n==1);*out=who;return HAL_OK;}
    if(n==1 && r<sizeof(registers)){
        *out=registers[r];
        if(r==bad_readback_reg)*out^=1U;
        return HAL_OK;
    }
    CHECK(r==0x3B && n==14);
    encode(out,sensor.ax);encode(out+2,sensor.ay);encode(out+4,sensor.az);
    encode(out+6,sensor.temp);encode(out+8,sensor.gx);
    encode(out+10,sensor.gy);encode(out+12,sensor.gz);
    if(inject_overrun){inject_overrun=0;Balance_OnTickISR();Balance_OnTickISR();}
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *t,uint32_t c)
{ CHECK((t==&htim2 || t==&htim3) && c==TIM_CHANNEL_ALL);return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *t,uint32_t c)
{ CHECK(t==&htim4 && (c==PWMA_CH || c==PWMB_CH));return pwm_status; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p,uint16_t pin,GPIO_PinState state)
{
    uint32_t previous=p->ODR;
    if(state)p->ODR|=pin;else p->ODR&=~pin;
    if(bus_pin_mode==GPIO_MODE_OUTPUT_OD && (pin&GPIO_PIN_10) && state
       && !(previous&GPIO_PIN_10) && !(held_bus_pins&GPIO_PIN_10)) {
        ++bus_rising_edges;
        if(release_sda_after && bus_rising_edges>=release_sda_after) held_bus_pins&=~GPIO_PIN_11;
    }
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p,uint16_t pin)
{ return ((p->ODR & ~held_bus_pins) & pin)?GPIO_PIN_SET:GPIO_PIN_RESET; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ CHECK(u==&huart1 && n==1);receive_byte=p;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{
    CHECK(u==&huart1 && n==1 && !transmit_pending);
    CHECK(transmitted_count<sizeof(transmitted)-1);
    transmitted[transmitted_count++]=(char)*p;transmitted[transmitted_count]=0;
    transmit_pending=1;return HAL_OK;
}
static void drain_tx(void)
{
    unsigned guard=0;
    while(transmit_pending){CHECK(++guard<20000);transmit_pending=0;HAL_UART_TxCpltCallback(&huart1);}
}
static void feed(const char *text)
{ while(*text){*receive_byte=(uint8_t)*text++;HAL_UART_RxCpltCallback(&huart1);} }
static void command(const char *text) { feed(text);UART_Poll();drain_tx(); }
static void step(void) { mock_ms+=5;Balance_OnTickISR();Balance_Poll();drain_tx(); }
static void key_set(uint8_t pressed)
{
    if(pressed)mock_gpio_b.ODR&=~GPIO_PIN_1;
    else mock_gpio_b.ODR|=GPIO_PIN_1;
    Balance_KeyPoll();drain_tx();
}
static void key_steps(unsigned count)
{
    for(unsigned i=0;i<count;i++){step();Balance_KeyPoll();drain_tx();}
}
static void level_sensor(void)
{
    memset(&sensor,0,sizeof(sensor));sensor.az=16384;who=0x68;sensor_status=HAL_OK;
    mock_gpio_b.ODR|=GPIO_PIN_10|GPIO_PIN_11;
    held_bus_pins=0;release_sda_after=bus_rising_edges=0;
    test_i2c_busy_flag=0;test_i2c_state=HAL_I2C_STATE_READY;
    test_bus_reinit_status=HAL_OK;
}
static void ready(void)
{
    level_sensor();pwm_status=HAL_OK;mock_gpio_b.ODR|=GPIO_PIN_1;
    CHECK(Balance_Init()==HAL_OK);
    CHECK(Balance_Arm(0));
    CHECK(!Balance_Arm(1));
    for(unsigned i=0;i<200;i++)step();
    CHECK(balance_state==BALANCE_READY && !(mock_gpio_b.ODR&STBY_PIN));
}
static void stopped(void)
{ CHECK(!(mock_gpio_b.ODR&STBY_PIN));CHECK(mock_tim4.CCR[0]==0 && mock_tim4.CCR[1]==0); }

static void fallen(void)
{
    ready();CHECK(Balance_Arm(1));
    target_speed=7.0f;target_turn=3.0f;
    sensor.ay=14189;sensor.az=8192;MPU6050_ResetAngle(60.0f);step();
    CHECK(balance_state==BALANCE_FAULT && balance_fault==FAULT_TILT);stopped();
    CHECK(target_speed==0.0f && target_turn==0.0f);
    level_sensor();MPU6050_ResetAngle(0.0f);
}

int main(void)
{
    CHECK(UART_Start(&huart1)==HAL_OK);

    level_sensor();
    sensor_status=HAL_TIMEOUT;
    CHECK(MPU6050_Init(&hi2c2)==HAL_TIMEOUT);drain_tx();
    CHECK(strstr(transmitted,"MPU read ID addr=0x68 reg=0x75 HAL=3 ERR=0x00000004")!=NULL);
    CHECK(strstr(transmitted,"MPU probe addr=0x69 HAL=1")!=NULL);
    level_sensor();who=0x71;
    unsigned writes_before=configuration_writes;
    CHECK(MPU6050_Init(&hi2c2)==HAL_ERROR);drain_tx();
    CHECK(configuration_writes==writes_before);
    CHECK(strstr(transmitted,"WHO=0x71 supported=0x68/0x70")!=NULL);
    CHECK(strstr(transmitted,"MPU ID mismatch")!=NULL);

    /* Reproduce the actual reported ID: initialization and calibration must work. */
    level_sensor();who=0x70;mock_gpio_b.ODR|=GPIO_PIN_1;
    unsigned filters_before=accel_filter_writes;
    CHECK(Balance_Init()==HAL_OK);CHECK(Balance_Arm(0));drain_tx();
    CHECK(accel_filter_writes==filters_before+1);
    CHECK(registers[MPU6500_ACCEL_CONFIG2]==3 && divider==4);
    CHECK(registers[MPU6050_PWR_MGMT_1]==1 && registers[MPU6050_PWR_MGMT_2]==0);
    CHECK(strstr(transmitted,"MPU init OK model=MPU6500")!=NULL);
    CHECK(!Balance_Arm(1));stopped();
    for(unsigned i=0;i<200;i++)step();
    CHECK(balance_state==BALANCE_READY);stopped();
    sensor.ay=8192;sensor.az=14189;sensor.gx=1310;step();
    CHECK(angle>0.5f && raw_gx>9.9f);stopped();

    /* A failed MPU6500-specific configuration must keep the motors off. */
    level_sensor();who=0x70;failed_write_reg=MPU6500_ACCEL_CONFIG2;
    CHECK(Balance_Init()==HAL_ERROR);drain_tx();
    CHECK(balance_state==BALANCE_FAULT && balance_fault==FAULT_IMU);stopped();
    failed_write_reg=0xFFFFU;bad_readback_reg=MPU6500_ACCEL_CONFIG2;
    CHECK(Balance_Init()==HAL_ERROR);drain_tx();
    CHECK(balance_state==BALANCE_FAULT && balance_fault==FAULT_IMU);stopped();
    bad_readback_reg=0xFFFFU;
    /* MPU6050 must never be written at the MPU6500-only register. */
    level_sensor();filters_before=accel_filter_writes;
    CHECK(MPU6050_Init(&hi2c2)==HAL_OK);drain_tx();
    CHECK(accel_filter_writes==filters_before);
    transmitted_count=0;transmitted[0]=0;
    level_sensor();sensor.ax=-32768;sensor.ay=-123;sensor.gx=-1310;sensor.temp=-222;
    CHECK(MPU6050_ReadAll(&hi2c2,raw)==HAL_OK);
    CHECK(raw->ax==-32768 && raw->ay==-123 && raw->gx==-1310 && raw->temp==-222);
    MPU6050_Raw_t before=*raw;
    sensor_status=HAL_ERROR;
    CHECK(MPU6050_ReadAll(&hi2c2,raw)==HAL_ERROR && !memcmp(raw,&before,sizeof(before)));
    level_sensor();CHECK(MPU6050_Init(&hi2c2)==HAL_OK && divider==4);
    who=0;CHECK(MPU6050_Init(&hi2c2)==HAL_ERROR);who=0x68;
    sensor.ay=8192;sensor.az=14189;
    float measured=MPU6050_AccelAngleX(&sensor);
    NEAR(measured,30.0f,0.01f);
    MPU6050_ResetAngle(measured);
    for(unsigned i=0;i<100;i++)NEAR(MPU6050_GetAngleX(&sensor,0),measured,0.01f);
    NEAR(MPU6050_GetAngleX(&sensor,10.0f),measured+0.0485f,0.001f);
    sensor.gx=1310;NEAR(MPU6050_GetGyroX(&sensor,0),10.0f,0.001f);

    mock_tim2.CNT=65530;mock_tim3.CNT=1000;
    CHECK(EncoderA_Init()==HAL_OK && EncoderB_Init()==HAL_OK);
    mock_tim2.CNT=4;mock_tim3.CNT=994;
    CHECK(EncoderA_GetSpeed()==10 && EncoderB_GetSpeed()==6);
    mock_tim2.CNT=65534;CHECK(EncoderA_GetSpeed()==-6);CHECK(EncoderB_GetSpeed()==0);

    CHECK(motor_Init()==HAL_OK);MotorA_SetSpeed(2000);stopped();
    Motor_Enable();MotorA_SetSpeed(99999);MotorB_SetSpeed(-99999);
    CHECK(mock_tim4.CCR[0]==PWM_MAX && mock_tim4.CCR[1]==PWM_MAX);
    CHECK((mock_gpio_b.ODR&AIN1_PIN) && !(mock_gpio_b.ODR&AIN2_PIN));
    CHECK(!(mock_gpio_b.ODR&BIN1_PIN) && (mock_gpio_b.ODR&BIN2_PIN));
    Motor_Disable();stopped();
    htim4.Instance=NULL;Motor_Disable();htim4.Instance=&mock_tim4;

    pid_t controller;
    PID_Init(&controller,0,0.2f,0,5,5);NEAR(pid_pi(&controller,0,1),0.2f,0.001f);
    PID_Init(&controller,10,1,0,5,5);NEAR(pid_pi(&controller,0,2),5.0f,0.001f);
    NEAR(controller.integral,0.0f,0.001f);

    /* Position hold uses signed wheel deltas, limits return speed, and reanchors
     * after a commanded movement or a stop/re-arm. */
    ready();CHECK(Balance_Arm(1));
    mock_tim2.CNT+=100;mock_tim3.CNT-=100;step();
    NEAR(position_counts,100.0f,0.001f);
    NEAR(position_speed_target,-0.5f,0.001f);
    mock_tim2.CNT+=500;mock_tim3.CNT-=500;step();
    NEAR(position_speed_target,-2.0f,0.001f);
    mock_tim2.CNT-=600;mock_tim3.CNT+=600;step();
    NEAR(position_counts,0.0f,0.001f);
    NEAR(position_speed_target,0.0f,0.001f);
    target_speed=3.0f;mock_tim2.CNT+=20;mock_tim3.CNT-=20;step();
    NEAR(position_counts,0.0f,0.001f);NEAR(position_speed_target,3.0f,0.001f);
    target_speed=0.0f;step();NEAR(position_speed_target,0.0f,0.001f);
    mock_tim2.CNT-=50;mock_tim3.CNT+=50;step();
    NEAR(position_speed_target,0.25f,0.001f);
    command("PKP 0.01\nPLIM 1\n");
    NEAR(pid_position.kp,0.01f,0.00001f);NEAR(pid_position.output_max,1.0f,0.001f);
    command("PKP -1\nPLIM -1\n");
    NEAR(pid_position.kp,0.01f,0.00001f);NEAR(pid_position.output_max,1.0f,0.001f);
    CHECK(Balance_Arm(0));NEAR(position_counts,0.0f,0.001f);
    CHECK(Balance_Arm(1));NEAR(position_speed_target,0.0f,0.001f);
    CHECK(Balance_Arm(0));

    ready();CHECK(Balance_Arm(1));sensor_status=HAL_ERROR;step();
    CHECK(balance_state==BALANCE_FAULT && balance_fault==FAULT_IMU);stopped();
    ready();CHECK(Balance_Arm(1));pid_turn.kp=100.0f;
    MPU6050_ResetAngle(60.0f);mock_tim2.CNT+=10;step();
    CHECK(balance_state==BALANCE_FAULT && balance_fault==FAULT_TILT);stopped();
    ready();CHECK(Balance_Arm(1));Balance_OnTickISR();Balance_OnTickISR();
    CHECK(balance_state==BALANCE_FAULT && balance_fault==FAULT_OVERRUN);stopped();
    ready();CHECK(Balance_Arm(1));inject_overrun=1;step();
    CHECK(balance_state==BALANCE_FAULT && balance_fault==FAULT_OVERRUN);stopped();
    ready();mock_ms+=25;CHECK(!Balance_Arm(1));step();
    mock_gpio_b.ODR&=~GPIO_PIN_1;CHECK(!Balance_Arm(1));mock_gpio_b.ODR|=GPIO_PIN_1;

    command("DKP 80\r\nDKD 2.5\nVKP 0.2\nVKI 0.01\n");
    NEAR(pid_balance.kp,80.0f,0.001f);NEAR(pid_balance.kd,2.5f,0.001f);
    NEAR(pid_speed.kp,0.2f,0.001f);NEAR(pid_speed.ki,0.01f,0.001f);
    command("DKP nan\nDKP 99junk\nDKP 90000\n");NEAR(pid_balance.kp,80.0f,0.001f);
    feed("DKP 33");for(unsigned i=0;i<150;i++)feed("A");command("\nDKP 42\n");
    NEAR(pid_balance.kp,42.0f,0.001f);
    for(unsigned i=0;i<300;i++)feed("B");UART_Poll();drain_tx();
    command("\nDKP 73\n");NEAR(pid_balance.kp,73.0f,0.001f);
    step();command("ARM 1\n");CHECK(balance_state==BALANCE_RUNNING);
    command("ARM 0\n");CHECK(balance_state==BALANCE_READY);stopped();

    /* Diagnostics distinguish the calibration blockers from an unarmed READY state. */
    level_sensor();CHECK(Balance_Init()==HAL_OK);CHECK(Balance_Arm(0));drain_tx();
    sensor.ay=14189;sensor.az=8192;sensor.gx=-3000;step();
    transmitted_count=0;transmitted[0]=0;
    command("STAT 0\n");
    CHECK(strstr(transmitted,"STATE=CALIBRATING(0)")!=NULL);
    CHECK(strstr(transmitted,"ANGLE_OK=0 GYRO_OK=0 GRAVITY_OK=1")!=NULL);
    CHECK(strstr(transmitted,"STBY=0 KEY_PRESSED=0")!=NULL);stopped();
    ready();transmitted_count=0;transmitted[0]=0;
    mock_tim2.CNT=(uint16_t)(mock_tim2.CNT+9);
    mock_tim3.CNT=(uint16_t)(mock_tim3.CNT-11);
    command("STAT 0\n");
    CHECK(strstr(transmitted,"STATE=READY(1)")!=NULL);
    CHECK(strstr(transmitted,"HINT READY: press PB1 to START")!=NULL);
    /* Reading diagnostics must not consume any counts. */
    CHECK(EncoderA_GetSpeed()==9 && EncoderB_GetSpeed()==11);stopped();
    motor_deadband=0.0f; /* This check isolates diagnostics from motor compensation. */
    CHECK(Balance_Arm(1));MotorA_SetSpeed(500);MotorB_SetSpeed(-600);
    transmitted_count=0;transmitted[0]=0;
    Balance_PrintDiagnostics();drain_tx();
    CHECK(strstr(transmitted,"STATE=RUNNING(2)")!=NULL);
    CHECK(strstr(transmitted,"PWM_A=500 PWM_B=600 STBY=1")!=NULL);
    CHECK(mock_tim4.CCR[0]==500 && mock_tim4.CCR[1]==600);
    CHECK(Balance_Arm(0));stopped();motor_deadband=65.0f;


    /* Regression: the observed -662-count DC bias must calibrate when stationary. */
    level_sensor();sensor.gx=-662;who=0x70;
    CHECK(Balance_Init()==HAL_OK);CHECK(Balance_Arm(0));drain_tx();
    for(unsigned i=0;i<199;i++)step();
    CHECK(balance_state==BALANCE_CALIBRATING);stopped();
    step();CHECK(balance_state==BALANCE_READY);NEAR(raw_gx,0.0f,0.001f);stopped();
    step();NEAR(raw_gx,0.0f,0.001f);
    /* Small noise about a larger constant offset is averaged, not rejected as rotation. */
    level_sensor();CHECK(Balance_Init()==HAL_OK);CHECK(Balance_Arm(0));drain_tx();
    for(unsigned i=0;i<200;i++){
        sensor.gx=-1000+(i%2?10:-10);sensor.gy=300;sensor.gz=-200;
        sensor.ay=(i%2?8:-8);step();
    }
    CHECK(balance_state==BALANCE_READY);NEAR(raw_gx,10.0f/131.0f,0.001f);stopped();
    /* Significant changing rate must never be accumulated as a DC offset. */
    level_sensor();CHECK(Balance_Init()==HAL_OK);CHECK(Balance_Arm(0));drain_tx();
    for(unsigned i=0;i<400;i++){sensor.gx=-662+(i%2?500:-500);step();}
    CHECK(balance_state==BALANCE_CALIBRATING);CHECK(!Balance_Arm(1));stopped();
    /* Slow X tilt still invalidates the window even with a constant gyro reading. */
    level_sensor();CHECK(Balance_Init()==HAL_OK);CHECK(Balance_Arm(0));drain_tx();sensor.gx=-662;
    for(unsigned i=0;i<200;i++){sensor.ay=(int16_t)(i*4);step();}
    CHECK(balance_state==BALANCE_CALIBRATING);stopped();
    /* Movement on another accelerometer axis is also detected. */
    level_sensor();CHECK(Balance_Init()==HAL_OK);CHECK(Balance_Arm(0));drain_tx();sensor.gx=-662;
    for(unsigned i=0;i<200;i++){sensor.ax=(int16_t)(i*8);step();}
    CHECK(balance_state==BALANCE_CALIBRATING);stopped();
    /* Large rates, wheel motion and lost sampling periods all block calibration. */
    level_sensor();CHECK(Balance_Init()==HAL_OK);CHECK(Balance_Arm(0));drain_tx();sensor.gx=-3000;
    for(unsigned i=0;i<200;i++)step();
    CHECK(balance_state==BALANCE_CALIBRATING);stopped();
    sensor.gx=-662;
    for(unsigned i=0;i<200;i++){++mock_tim2.CNT;step();}
    CHECK(balance_state==BALANCE_CALIBRATING);stopped();
    for(unsigned i=0;i<199;i++)step();
    Balance_OnTickISR();step();CHECK(balance_state==BALANCE_CALIBRATING);
    for(unsigned i=0;i<199;i++)step();CHECK(balance_state==BALANCE_CALIBRATING);
    step();CHECK(balance_state==BALANCE_READY);NEAR(raw_gx,0.0f,0.001f);stopped();

    /* Reported -8.52 degree pose and -529-count bias, with bounded all-axis noise.
     * The former peak-to-peak rule reset on this noise; statistics must complete. */
    level_sensor();mock_gpio_b.ODR|=GPIO_PIN_1;
    CHECK(Balance_Init()==HAL_OK);drain_tx();
    for(unsigned i=0;i<200;++i) {
        int noise=(i%2)?1:-1;
        sensor.ax=(int16_t)(noise*450);sensor.ay=(int16_t)(-2614+noise*150);sensor.az=17450;
        sensor.gx=(int16_t)(-529+noise*150);sensor.gy=(int16_t)(noise*150);sensor.gz=(int16_t)(noise*150);
        step();
        if(i<199){CHECK(balance_state==BALANCE_CALIBRATING);stopped();}
    }
    CHECK(balance_state==BALANCE_RUNNING && balance_fault==FAULT_NONE);
    NEAR(angle,-8.52f,0.05f);sensor.gx=-529;step();NEAR(raw_gx,0.0f,0.01f);
    CHECK(Balance_Arm(0));stopped();

    /* An isolated modest spike is averaged instead of resetting an entire second. */
    level_sensor();CHECK(Balance_Init()==HAL_OK);CHECK(Balance_Arm(0));drain_tx();
    for(unsigned i=0;i<200;++i) {
        level_sensor();sensor.gx=-529;
        if(i==130){sensor.ay=500;sensor.ax=1200;sensor.gx+=600;}
        step();
    }
    CHECK(balance_state==BALANCE_READY);stopped();NEAR(raw_gx,-3.0f/131.0f,0.01f);

    /* Large persistent gyro variation fails on every axis and identifies the cause. */
    for(unsigned axis=0;axis<3;++axis) {
        level_sensor();CHECK(Balance_Init()==HAL_OK);CHECK(Balance_Arm(0));drain_tx();
        for(unsigned i=0;i<200;++i) {
            int16_t value=(i%2)?500:-500;
            sensor.gx=axis==0?value:0;sensor.gy=axis==1?value:0;sensor.gz=axis==2?value:0;step();
        }
        CHECK(balance_state==BALANCE_CALIBRATING);stopped();
        transmitted_count=0;transmitted[0]=0;command("STAT 0\n");
        CHECK(strstr(transmitted,"LAST=GYRO_NOISE")!=NULL);
        CHECK(strstr(transmitted,"CAL WINDOW: SD_A=")!=NULL);
    }
    /* Slow angle, gyro and transverse-acceleration drifts must not become zero bias. */
    for(unsigned drift_axis=0;drift_axis<3;++drift_axis) {
        level_sensor();CHECK(Balance_Init()==HAL_OK);CHECK(Balance_Arm(0));drain_tx();
        for(unsigned i=0;i<200;++i) {
            sensor.ay=drift_axis==0?(int16_t)(i*4):0;
            sensor.gx=drift_axis==1?(int16_t)(i*2):0;
            sensor.ax=drift_axis==2?(int16_t)(i*8):0;step();
        }
        CHECK(balance_state==BALANCE_CALIBRATING);stopped();
        transmitted_count=0;transmitted[0]=0;command("STAT 0\n");
        CHECK(strstr(transmitted,drift_axis==0?"LAST=ANGLE_DRIFT":
                     (drift_axis==1?"LAST=GYRO_DRIFT":"LAST=ACCEL_DRIFT"))!=NULL);
        /* Once stationary again, one clean window must complete without a reset. */
        level_sensor();for(unsigned i=0;i<200;++i)step();
        CHECK(balance_state==BALANCE_READY);stopped();
    }
    /* Detect angle and acceleration vibration independently of the other channels. */
    for(unsigned noise_axis=0;noise_axis<2;++noise_axis) {
        level_sensor();CHECK(Balance_Init()==HAL_OK);CHECK(Balance_Arm(0));drain_tx();
        for(unsigned i=0;i<200;++i) {
            int noise=(i%2)?1:-1;
            sensor.ay=noise_axis==0?(int16_t)(noise*500):0;
            sensor.ax=noise_axis==1?(int16_t)(noise*550):0;step();
        }
        CHECK(balance_state==BALANCE_CALIBRATING);stopped();
        transmitted_count=0;transmitted[0]=0;command("STAT 0\n");
        CHECK(strstr(transmitted,noise_axis==0?"LAST=ANGLE_NOISE":"LAST=ACCEL_NOISE")!=NULL);
    }
    /* Timing and wheel restart causes remain visible even when raw sensor ranges pass. */
    level_sensor();CHECK(Balance_Init()==HAL_OK);CHECK(Balance_Arm(0));drain_tx();
    for(unsigned i=0;i<50;++i)step();Balance_OnTickISR();step();
    transmitted_count=0;transmitted[0]=0;command("STAT 0\n");
    CHECK(strstr(transmitted,"LAST=SAMPLE_GAP")!=NULL);stopped();
    ++mock_tim2.CNT;step();transmitted_count=0;transmitted[0]=0;command("STAT 0\n");
    CHECK(strstr(transmitted,"LAST=WHEEL_MOVED")!=NULL);stopped();

    /* Idle initialization must not pulse or reset a healthy bus. */
    level_sensor();unsigned deinit_before=bus_deinit_calls;
    CHECK(MPU6050_Init(&hi2c2)==HAL_OK);drain_tx();
    CHECK(bus_deinit_calls==deinit_before && bus_rising_edges==0);
    /* A slave interrupted mid-byte releases SDA after three recovery clocks. */
    level_sensor();who=0x70;held_bus_pins=GPIO_PIN_11;release_sda_after=3;test_i2c_busy_flag=1;
    transmitted_count=0;transmitted[0]=0;
    CHECK(Balance_Init()==HAL_OK);CHECK(Balance_Arm(0));drain_tx();
    CHECK(bus_rising_edges>=3 && bus_rising_edges<=10);
    CHECK(bus_pin_mode==GPIO_MODE_AF_OD && !test_i2c_busy_flag);
    CHECK(strstr(transmitted,"I2C2 recover OK")!=NULL);
    CHECK(strstr(transmitted,"PULSES=3")!=NULL);stopped();
    for(unsigned i=0;i<200;i++)step();CHECK(balance_state==BALANCE_READY);stopped();
    /* MCU filter BUSY with physical lines high follows the verified transition sequence. */
    level_sensor();test_i2c_busy_flag=1;
    CHECK(MPU6050_Init(&hi2c2)==HAL_OK);drain_tx();CHECK(!test_i2c_busy_flag);
    /* A permanently held SDA must return in bounded time and keep motors off. */
    level_sensor();held_bus_pins=GPIO_PIN_11;test_i2c_busy_flag=1;
    uint32_t failed_start=mock_ms;unsigned init_before=bus_init_calls;
    CHECK(Balance_Init()==HAL_ERROR);drain_tx();
    CHECK(mock_ms-failed_start<150U && bus_rising_edges==9);
    CHECK(bus_init_calls==init_before+1 && bus_pin_mode==GPIO_MODE_AF_OD);
    CHECK(balance_state==BALANCE_FAULT && balance_fault==FAULT_IMU);stopped();
    /* A permanently held SCL must time out without attempting clocks. */
    level_sensor();held_bus_pins=GPIO_PIN_10;test_i2c_busy_flag=1;failed_start=mock_ms;
    CHECK(Balance_Init()==HAL_ERROR);drain_tx();
    CHECK(mock_ms-failed_start<120U && bus_rising_edges==0);
    CHECK(bus_pin_mode==GPIO_MODE_AF_OD && balance_state==BALANCE_FAULT);stopped();
    /* A peripheral reinitialization failure must never be reported as recovery success. */
    level_sensor();test_i2c_busy_flag=1;test_bus_reinit_status=HAL_ERROR;
    CHECK(Balance_Init()==HAL_ERROR);drain_tx();
    CHECK(balance_state==BALANCE_FAULT && bus_pin_mode==GPIO_MODE_AF_OD);stopped();
    level_sensor();

    /* The exact binary command provided to the user must enable from READY. */
    ready();transmitted_count=0;transmitted[0]=0;
    command("\x0A\x41\x52\x4D\x20\x31\x0A");
    CHECK(balance_state==BALANCE_RUNNING);
    CHECK(strstr(transmitted,"ARM 1\r\n")!=NULL);
    command("\nARM 0\n");stopped();
    /* Distinguish ASCII hex digits and literal backslash-n from actual command bytes. */
    transmitted_count=0;transmitted[0]=0;
    command("0A 41 52 4D 20 31 0A\n");
    CHECK(strstr(transmitted,"ERR invalid number CMD='0A'")!=NULL);
    CHECK(strstr(transmitted,"RX FRAME LEN=20 HEX=30 41 20 34 31")!=NULL);stopped();
    transmitted_count=0;transmitted[0]=0;
    command("ARM 1\\n\n");
    CHECK(strstr(transmitted,"RX FRAME LEN=7 HEX=41 52 4D 20 31 5C 6E")!=NULL);stopped();
    /* A valid decoded ARM rejected by pose must report the received frame and state. */
    MPU6050_ResetAngle(30.0f);step();transmitted_count=0;transmitted[0]=0;
    command("ARM 1\n");
    CHECK(strstr(transmitted,"ERR ARM: require READY")!=NULL);
    CHECK(strstr(transmitted,"RX FRAME LEN=5 HEX=41 52 4D 20 31")!=NULL);
    CHECK(strstr(transmitted,"STATE=READY(1)")!=NULL);stopped();
    transmitted_count=0;transmitted[0]=0;command("ARM\n");
    CHECK(strstr(transmitted,"RX FRAME LEN=3 HEX=41 52 4D")!=NULL);stopped();

    /* Active-low key starts on one stable press and stops on the next. */
    ready();key_set(1);key_steps(5);
    CHECK(balance_state==BALANCE_READY);stopped();
    key_steps(1);CHECK(balance_state==BALANCE_RUNNING && (mock_gpio_b.ODR&STBY_PIN));
    key_steps(40);CHECK(balance_state==BALANCE_RUNNING);
    key_set(0);key_steps(6);CHECK(balance_state==BALANCE_RUNNING);
    key_set(1);key_steps(6);CHECK(balance_state==BALANCE_READY);stopped();
    key_steps(40);CHECK(balance_state==BALANCE_READY);stopped();
    key_set(0);key_steps(6);key_set(1);key_steps(6);
    CHECK(balance_state==BALANCE_RUNNING);CHECK(Balance_Arm(0));stopped();

    /* Contact bounce must not create an extra start/stop transition. */
    ready();key_set(1);key_steps(2);key_set(0);key_steps(2);
    key_set(1);key_steps(2);key_set(0);key_steps(2);
    CHECK(balance_state==BALANCE_READY);stopped();
    key_set(1);key_steps(6);CHECK(balance_state==BALANCE_RUNNING);
    key_set(0);key_steps(2);key_set(1);key_steps(2);
    CHECK(balance_state==BALANCE_RUNNING);
    key_set(0);key_steps(6);key_set(1);key_steps(6);
    CHECK(balance_state==BALANCE_READY);stopped();

    /* A key held at boot cannot start after calibration without release and re-press. */
    level_sensor();mock_gpio_b.ODR&=~GPIO_PIN_1;
    CHECK(Balance_Init()==HAL_OK);CHECK(Balance_Arm(0));drain_tx();key_steps(240);
    CHECK(balance_state==BALANCE_READY);stopped();
    key_set(0);key_steps(6);key_set(1);key_steps(6);
    CHECK(balance_state==BALANCE_RUNNING);CHECK(Balance_Arm(0));stopped();

    /* A press during calibration is consumed, never deferred into a later automatic start. */
    level_sensor();mock_gpio_b.ODR|=GPIO_PIN_1;
    CHECK(Balance_Init()==HAL_OK);CHECK(Balance_Arm(0));drain_tx();key_steps(20);
    key_set(1);key_steps(6);CHECK(balance_state==BALANCE_CALIBRATING);stopped();
    key_steps(220);CHECK(balance_state==BALANCE_READY);stopped();
    key_set(0);key_steps(6);key_set(1);key_steps(6);
    CHECK(balance_state==BALANCE_RUNNING);CHECK(Balance_Arm(0));stopped();

    /* A denied press at excessive tilt cannot become a start by merely holding the key. */
    ready();MPU6050_ResetAngle(30.0f);step();key_set(1);key_steps(6);
    CHECK(balance_state==BALANCE_READY);stopped();
    key_steps(60);CHECK(balance_state==BALANCE_READY);stopped();
    key_set(0);key_steps(6);key_set(1);key_steps(6);
    CHECK(balance_state==BALANCE_RUNNING);CHECK(Balance_Arm(0));stopped();

    /* Button presses never clear a latched sensor fault. */
    ready();CHECK(Balance_Arm(1));sensor_status=HAL_ERROR;step();
    key_set(1);key_steps(6);CHECK(balance_state==BALANCE_FAULT);stopped();
    key_set(0);key_steps(6);key_set(1);key_steps(6);
    CHECK(balance_state==BALANCE_FAULT);stopped();

    /* Debounce survives the millisecond clock wrapping through zero. */
    ready();mock_ms=0xFFFFFFF0U;key_set(1);key_steps(6);
    CHECK(balance_state==BALANCE_RUNNING);CHECK(Balance_Arm(0));stopped();
    level_sensor();

    /* New boot default: remain off through calibration, then automatically enter RUNNING. */
    level_sensor();mock_gpio_b.ODR|=GPIO_PIN_1;
    CHECK(Balance_Init()==HAL_OK);drain_tx();
    key_steps(199);CHECK(balance_state==BALANCE_CALIBRATING);stopped();
    transmitted_count=0;transmitted[0]=0;
    key_steps(1);CHECK(balance_state==BALANCE_RUNNING && (mock_gpio_b.ODR&STBY_PIN));
    CHECK(strstr(transmitted,"AUTO START: RUNNING")!=NULL);
    /* First press stops, second starts; staying stopped must not trigger boot auto-start again. */
    key_set(1);key_steps(6);CHECK(balance_state==BALANCE_READY);stopped();
    key_steps(220);CHECK(balance_state==BALANCE_READY);stopped();
    key_set(0);key_steps(6);key_set(1);key_steps(6);
    CHECK(balance_state==BALANCE_RUNNING);
    CHECK(Balance_Arm(0));key_set(0);key_steps(6);
    /* A reset/reinitialization restores the requested startup-enabled default. */
    CHECK(Balance_Init()==HAL_OK);drain_tx();key_steps(200);
    CHECK(balance_state==BALANCE_RUNNING);CHECK(Balance_Arm(0));stopped();

    /* A stop press while calibration is pending cancels automatic starting. */
    level_sensor();mock_gpio_b.ODR|=GPIO_PIN_1;
    CHECK(Balance_Init()==HAL_OK);drain_tx();key_steps(30);
    key_set(1);key_steps(6);key_set(0);key_steps(6);key_steps(200);
    CHECK(balance_state==BALANCE_READY);stopped();
    key_set(1);key_steps(6);CHECK(balance_state==BALANCE_RUNNING);
    CHECK(Balance_Arm(0));stopped();

    /* A key held at calibration completion blocks the one-shot auto-start. */
    level_sensor();mock_gpio_b.ODR&=~GPIO_PIN_1;
    CHECK(Balance_Init()==HAL_OK);drain_tx();key_steps(220);
    CHECK(balance_state==BALANCE_READY);stopped();
    key_set(0);key_steps(40);CHECK(balance_state==BALANCE_READY);stopped();
    key_set(1);key_steps(6);CHECK(balance_state==BALANCE_RUNNING);
    CHECK(Balance_Arm(0));stopped();

    /* Sensor fault cancels boot start, and an explicit CAL must not auto-restart the car. */
    level_sensor();mock_gpio_b.ODR|=GPIO_PIN_1;
    CHECK(Balance_Init()==HAL_OK);drain_tx();sensor_status=HAL_ERROR;step();
    CHECK(balance_state==BALANCE_FAULT);stopped();
    level_sensor();key_steps(220);CHECK(balance_state==BALANCE_FAULT);stopped();
    CHECK(Balance_Calibrate()==HAL_OK);drain_tx();key_steps(200);
    CHECK(balance_state==BALANCE_READY);stopped();
    key_set(1);key_steps(6);CHECK(balance_state==BALANCE_RUNNING);
    CHECK(Balance_Arm(0));stopped();
    level_sensor();
    /* A reset enables control even when zero angle error legitimately produces zero PWM. */
    for (unsigned reboot=0; reboot<3; ++reboot) {
        level_sensor();mock_gpio_b.ODR|=GPIO_PIN_1;
        CHECK(Balance_Init()==HAL_OK);drain_tx();key_steps(200);step();
        CHECK(balance_state==BALANCE_RUNNING && balance_fault==FAULT_NONE);
        CHECK(mock_gpio_b.ODR&STBY_PIN);
        CHECK(mock_tim4.CCR[0]==0 && mock_tim4.CCR[1]==0);
        sensor.ay=1000;sensor.az=16353;
        key_steps(20);
        CHECK(balance_state==BALANCE_RUNNING);
        CHECK(mock_tim4.CCR[0]>0 && mock_tim4.CCR[1]>0);
    }
    /* Both signs: 45 cuts both channels; only a continuous upright window resumes. */
    for (int sign=-1; sign<=1; sign+=2) {
        ready();CHECK(Balance_Arm(1));
        sensor.ay=(int16_t)(sign*11564);sensor.az=11605;
        MPU6050_ResetAngle(sign*44.9f);step();
        CHECK(balance_state==BALANCE_RUNNING);
        CHECK(mock_tim4.CCR[0]>0 && mock_tim4.CCR[1]>0);
        sensor.ay=(int16_t)(sign*11585);sensor.az=11585;
        MPU6050_ResetAngle(sign*45.0f);
        transmitted_count=0;transmitted[0]=0;step();
        CHECK(balance_state==BALANCE_FAULT && balance_fault==FAULT_TILT);stopped();
        CHECK(strstr(transmitted,"TILT STOP:")!=NULL);
        key_steps(220);CHECK(balance_state==BALANCE_FAULT);stopped();
        CHECK(!Balance_Arm(1));stopped();
        level_sensor();MPU6050_ResetAngle(0.0f);
        key_steps(39);CHECK(balance_state==BALANCE_FAULT);stopped();
        key_steps(1);CHECK(balance_state==BALANCE_RUNNING && balance_fault==FAULT_NONE);
        CHECK(mock_gpio_b.ODR&STBY_PIN);
        CHECK(strstr(transmitted,"TILT RESUME: RUNNING")!=NULL);
    }
    /* A nonzero calibration offset must never mask the absolute 45-degree cutout,
     * and the relative cutout must still protect the opposite direction. */
    for (int sign=-1; sign<=1; sign+=2) {
        for (unsigned relative=0;relative<2;++relative) {
            level_sensor();mock_gpio_b.ODR|=GPIO_PIN_1;
            sensor.ay=(int16_t)(sign*1428);sensor.az=16322; /* zero about +/-5 degrees */
            CHECK(Balance_Init()==HAL_OK);drain_tx();key_steps(200);
            CHECK(balance_state==BALANCE_RUNNING);
            int direction=relative ? -sign : sign;
            if(relative) {
                sensor.ay=(int16_t)(direction*10641);sensor.az=12456; /* +/-40.5 */
                MPU6050_ResetAngle(direction*40.5f);
            } else {
                sensor.ay=(int16_t)(direction*11585);sensor.az=11585;
                MPU6050_ResetAngle(direction*45.0f);
            }
            step();CHECK(balance_state==BALANCE_FAULT && balance_fault==FAULT_TILT);stopped();
            /* Recovery reuses this chassis zero instead of reinitializing the IMU. */
            sensor.ay=(int16_t)(sign*1428);sensor.az=16322;
            MPU6050_ResetAngle(sign*5.0f);key_steps(40);
            CHECK(balance_state==BALANCE_RUNNING);step();
            CHECK(mock_tim4.CCR[0]==0 && mock_tim4.CCR[1]==0);
        }
    }
    /* A tilted reset waits for calibration, then starts once an upright still window is valid. */
    level_sensor();mock_gpio_b.ODR|=GPIO_PIN_1;sensor.ay=14189;sensor.az=8192;
    CHECK(Balance_Init()==HAL_OK);drain_tx();key_steps(220);
    CHECK(balance_state==BALANCE_CALIBRATING);stopped();
    level_sensor();key_steps(200);CHECK(balance_state==BALANCE_RUNNING);
    CHECK(Balance_Arm(0));stopped();
    /* A posture excursion restarts the recovery window, with hysteresis at +/-10. */
    fallen();key_steps(25);stopped();
    sensor.ay=2860;sensor.az=16132;MPU6050_ResetAngle(10.1f);step();
    CHECK(balance_state==BALANCE_FAULT);stopped();
    level_sensor();MPU6050_ResetAngle(0.0f);key_steps(39);stopped();
    CHECK(balance_state==BALANCE_FAULT);key_steps(1);CHECK(balance_state==BALANCE_RUNNING);

    /* A lagging filtered angle near zero cannot restart when raw posture is still fallen. */
    fallen();sensor.ay=14189;sensor.az=8192;
    for(unsigned i=0;i<80;++i){MPU6050_ResetAngle(0.0f);step();stopped();}
    CHECK(balance_state==BALANCE_FAULT);
    level_sensor();MPU6050_ResetAngle(0.0f);key_steps(40);CHECK(balance_state==BALANCE_RUNNING);

    /* Rotation, implausible acceleration, wheel motion and missed ticks reset recovery. */
    fallen();key_steps(25);sensor.gx=21*131;step();stopped();
    level_sensor();MPU6050_ResetAngle(0.0f);key_steps(39);stopped();
    key_steps(1);CHECK(balance_state==BALANCE_RUNNING);
    fallen();key_steps(25);sensor.az=0;step();stopped();
    level_sensor();MPU6050_ResetAngle(0.0f);key_steps(39);stopped();
    key_steps(1);CHECK(balance_state==BALANCE_RUNNING);
    fallen();key_steps(25);++mock_tim2.CNT;step();stopped();
    key_steps(39);stopped();key_steps(1);CHECK(balance_state==BALANCE_RUNNING);
    fallen();key_steps(25);Balance_OnTickISR();step();stopped();
    key_steps(39);stopped();key_steps(1);CHECK(balance_state==BALANCE_RUNNING);
    /* Missed timer events during an I2C read cannot count as stable recovery either. */
    fallen();key_steps(39);inject_overrun=1;step();stopped();
    CHECK(balance_state==BALANCE_FAULT);step();stopped();
    key_steps(39);stopped();key_steps(1);CHECK(balance_state==BALANCE_RUNNING);

    /* PB1 or serial STOP cancels pending recovery permanently until another start. */
    fallen();key_steps(5);key_set(1);key_steps(6);
    CHECK(balance_state==BALANCE_READY && balance_fault==FAULT_NONE);stopped();
    key_set(0);key_steps(220);CHECK(balance_state==BALANCE_READY);stopped();
    key_set(1);key_steps(6);CHECK(balance_state==BALANCE_RUNNING);
    CHECK(Balance_Arm(0));key_set(0);key_steps(6);
    fallen();key_steps(25);command("ARM 0\n");
    CHECK(balance_state==BALANCE_READY);key_steps(220);stopped();
    CHECK(balance_state==BALANCE_READY);

    /* Physical press at the last recovery sample prevents enabling before debounce. */
    fallen();key_steps(39);key_set(1);step();stopped();
    CHECK(balance_state==BALANCE_FAULT);key_steps(5);
    CHECK(balance_state==BALANCE_READY);stopped();
    key_set(0);key_steps(40);CHECK(balance_state==BALANCE_READY);stopped();

    /* Failed samples override recoverable tilt with a latched IMU fault. */
    fallen();key_steps(39);sensor_status=HAL_ERROR;step();
    CHECK(balance_state==BALANCE_FAULT && balance_fault==FAULT_IMU);stopped();
    level_sensor();key_steps(220);CHECK(balance_state==BALANCE_FAULT);stopped();
    /* An overrun after resuming is still latched, never a tilt auto-recovery. */
    fallen();key_steps(40);CHECK(balance_state==BALANCE_RUNNING);
    Balance_OnTickISR();Balance_OnTickISR();
    CHECK(balance_state==BALANCE_FAULT && balance_fault==FAULT_OVERRUN);stopped();
    key_steps(220);CHECK(balance_state==BALANCE_FAULT);stopped();
    pwm_status=HAL_ERROR;CHECK(Balance_Init()==HAL_ERROR && balance_fault==FAULT_HARDWARE);
    CHECK(Balance_Calibrate()==HAL_ERROR);CHECK(!Balance_Arm(1));stopped();
    printf("PASS: %u assertions across sensor, filter, encoder, motor, PI, fault and UART tests\n",checks);
    return 0;
}
