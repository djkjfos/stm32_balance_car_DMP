/* Exercise the existing controller against a mocked DMP adapter. The real
 * DMP driver/port are tested separately by test_dmp.c. */
#define main original_test_main
#include "test_balance.c"
#undef main
#include "dmp_attitude.h"

static HAL_StatusTypeDef fake_dmp_init = HAL_OK;
static HAL_StatusTypeDef fake_dmp_poll = HAL_OK;
static float fake_dmp_angle;

HAL_StatusTypeDef DMP_AttitudeInit(void) { return fake_dmp_init; }
HAL_StatusTypeDef DMP_AttitudePoll(void) { return fake_dmp_poll; }
unsigned DMP_AttitudeLastError(void) { return fake_dmp_poll == HAL_OK ? 0U : 3U; }
void DMP_AttitudeResetAngle(float angle_deg) { fake_dmp_angle = angle_deg; }
float DMP_AttitudeGetAngle(float gyro_deg_s) { (void)gyro_deg_s; return fake_dmp_angle; }

int main(void)
{
    CHECK(UART_Start(&huart1)==HAL_OK);
    ready();
    CHECK(Balance_Arm(1));
    sensor.gx=164; step();
    NEAR(raw_gx, 164.0f * 8.0f / 131.0f, 0.001f);
    sensor.gx=32767; CHECK(MPU6050_ReadAll(&hi2c2,raw)==HAL_OK);
    CHECK(raw->gx==32767); /* saturation to original +/-250 dps units */
    sensor.gx=-32768; CHECK(MPU6050_ReadAll(&hi2c2,raw)==HAL_OK);
    CHECK(raw->gx==-32768);
    MPU6050_Raw_t previous=*raw;
    fake_dmp_poll=HAL_ERROR; sensor.gx=0;
    CHECK(MPU6050_ReadAll(&hi2c2,raw)==HAL_ERROR);
    CHECK(!memcmp(raw,&previous,sizeof(previous))); /* failed packet never replaces sample */
    step();CHECK(balance_state==BALANCE_FAULT && balance_fault==FAULT_IMU);stopped();

    fake_dmp_poll=HAL_OK; level_sensor(); who=0x70;
    CHECK(MPU6050_Init(&hi2c2)==HAL_ERROR); /* this image is MPU6050 only */
    level_sensor(); fake_dmp_init=HAL_ERROR; mock_gpio_b.ODR|=GPIO_PIN_1;
    CHECK(Balance_Init()==HAL_ERROR);
    CHECK(balance_state==BALANCE_FAULT && balance_fault==FAULT_IMU);stopped();

    fake_dmp_init=HAL_OK; ready();CHECK(Balance_Arm(1));
    fake_dmp_poll=HAL_TIMEOUT;step();
    CHECK(balance_state==BALANCE_FAULT && balance_fault==FAULT_IMU);stopped();
    printf("PASS: %u DMP controller calibration, scaling and fail-safe checks\n",checks);
    return 0;
}
