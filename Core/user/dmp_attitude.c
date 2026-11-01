#include "dmp_attitude.h"
#include "dmp_platform.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "uart_cmd.h"
#include <math.h>

#define DMP_STALE_MS 15U
#define DMP_FIRST_FRAME_TIMEOUT_MS 500U
#define RAD_TO_DEG 57.2957795f
static float measured_angle, output_angle, reference_offset;
static uint32_t last_frame_ms;
static uint8_t frame_available, new_frame;
static unsigned last_error;
unsigned DMP_AttitudeLastError(void) { return last_error; }

static float wrap_deg(float angle)
{
    if (angle > 180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}

static HAL_StatusTypeDef fail_init(unsigned stage)
{
    UART_Printf("DMP init FAIL step=%u; motors remain disabled\r\n", stage);
    return HAL_ERROR;
}

HAL_StatusTypeDef DMP_AttitudePoll(void)
{
    short gyro[3] = {0}, accel[3] = {0}, sensors = 0;
    long q[4] = {0};
    unsigned long timestamp;
    unsigned char more = 0;
    float q0, q1, q2, q3, norm;
    int result;
    unsigned packets = 0U;
    new_frame = 0U;
    last_error = 0U;
    DMP_PlatformClearError();
    result = dmp_read_fifo(gyro, accel, q, &timestamp, &sensors, &more);
    if (DMP_PlatformHasError()) { last_error = 1U; return HAL_ERROR; }
    if (result == -2) { last_error = 2U; return HAL_ERROR; }
    if (result != 0 || !(sensors & INV_WXYZ_QUAT)) {
        /* A missing packet is normal when the two 200Hz clocks differ slightly. */
        if (!frame_available || (uint32_t)(HAL_GetTick() - last_frame_ms) > DMP_STALE_MS) {
            last_error = 3U;
            return HAL_TIMEOUT;
        }
        return HAL_OK;
    }
    /* Drain up to three packets to follow small clock-rate differences.
     * Larger backlogs still fault instead of applying a stale attitude. */
    packets = 1U;
    while (more != 0U) {
        if (more > 2U || packets >= 3U) { last_error = 4U; return HAL_ERROR; }
        result = dmp_read_fifo(gyro, accel, q, &timestamp, &sensors, &more);
        ++packets;
        if (DMP_PlatformHasError()) { last_error = 1U; return HAL_ERROR; }
        if (result == -2) { last_error = 2U; return HAL_ERROR; }
        if (result != 0 || !(sensors & INV_WXYZ_QUAT)) {
            last_error = 5U; return HAL_ERROR;
        }
    }
    q0 = (float)q[0] / 1073741824.0f; q1 = (float)q[1] / 1073741824.0f;
    q2 = (float)q[2] / 1073741824.0f; q3 = (float)q[3] / 1073741824.0f;
    norm = q0*q0 + q1*q1 + q2*q2 + q3*q3;
    if (!isfinite(norm) || norm < 0.8f || norm > 1.2f) { last_error = 6U; return HAL_ERROR; }
    norm = sqrtf(norm); q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
    measured_angle = atan2f(2.0f * (q0*q1 + q2*q3),
                            1.0f - 2.0f * (q1*q1 + q2*q2)) * RAD_TO_DEG;
    if (!isfinite(measured_angle)) { last_error = 6U; return HAL_ERROR; }
    last_frame_ms = HAL_GetTick(); frame_available = new_frame = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef DMP_AttitudeInit(void)
{
    unsigned short gyro_fsr = 0;
    unsigned short accel_sensitivity = 0;
    unsigned char accel_fsr = 0;
    unsigned char fsr_registers[2] = {0};
    uint32_t started;
    frame_available = new_frame = 0U;
    last_error = 0U;
    measured_angle = output_angle = reference_offset = 0.0f;
    DMP_PlatformSetRunning(0);
    DMP_PlatformClearError();
#define DMP_INIT_STEP(n, call) do { if ((call) != 0 || DMP_PlatformHasError()) return fail_init(n); } while (0)
    DMP_INIT_STEP(1U, mpu_init(NULL));
    DMP_INIT_STEP(2U, mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL));
    /* This Motion Driver DMP image uses 2000 dps; the adapter converts raw
     * gyro readings back into the original controller's 131-count units. */
    DMP_INIT_STEP(3U, mpu_set_gyro_fsr(2000));
    DMP_INIT_STEP(4U, mpu_set_accel_fsr(2));
    DMP_INIT_STEP(5U, mpu_set_lpf(42));
    DMP_INIT_STEP(6U, mpu_set_sample_rate(200));
    DMP_INIT_STEP(7U, dmp_load_motion_driver_firmware());
    DMP_INIT_STEP(8U, dmp_set_orientation(0x88)); /* identity XYZ axes */
    DMP_INIT_STEP(9U, dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL));
    DMP_INIT_STEP(10U, dmp_set_fifo_rate(200));
    DMP_INIT_STEP(11U, mpu_set_dmp_state(1));
    DMP_INIT_STEP(12U, mpu_get_gyro_fsr(&gyro_fsr));
    DMP_INIT_STEP(13U, mpu_get_accel_fsr(&accel_fsr));
    DMP_INIT_STEP(14U, mpu_get_accel_sens(&accel_sensitivity));
    DMP_INIT_STEP(15U, i2c_read(0x68U, 0x1BU, 2U, fsr_registers));
    if (gyro_fsr != 2000U || accel_fsr != 2U || accel_sensitivity != 16384U
        || (fsr_registers[0] & 0x18U) != 0x18U || (fsr_registers[1] & 0x18U) != 0U)
        return fail_init(16U);
#undef DMP_INIT_STEP
    /* Wait for a valid hardware quaternion before allowing calibration. */
    started = HAL_GetTick();
    do {
        HAL_StatusTypeDef status = DMP_AttitudePoll();
        if (status == HAL_OK && frame_available) {
            DMP_PlatformSetRunning(1);
            UART_Printf("DMP init OK MPU6050 FIFO=200Hz gyro=2000dps\r\n");
            return HAL_OK;
        }
        if (status == HAL_ERROR) return fail_init(17U);
        HAL_Delay(1U);
    } while ((uint32_t)(HAL_GetTick() - started) < DMP_FIRST_FRAME_TIMEOUT_MS);
    return fail_init(18U);
}

void DMP_AttitudeResetAngle(float angle_deg)
{
    reference_offset = frame_available ? wrap_deg(angle_deg - measured_angle) : 0.0f;
    output_angle = angle_deg;
}

float DMP_AttitudeGetAngle(float gyro_deg_s)
{
    if (new_frame) output_angle = wrap_deg(measured_angle + reference_offset);
    else output_angle = wrap_deg(output_angle + gyro_deg_s * 0.005f);
    new_frame = 0U;
    return output_angle;
}
