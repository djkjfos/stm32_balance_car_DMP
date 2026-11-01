/* Simulate register and DMP memory access using the actual port and driver.
 * This does not simulate the MPU6050's DMP execution or I2C timing. */
#include "dmp_attitude.h"
#include "dmp_platform.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "i2c.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

I2C_HandleTypeDef hi2c2;
static uint32_t now_ms;
static uint8_t regs[256], dmp_mem[8192], fifo_data[16];
static unsigned bank, address, fw_writes;
static int fifo_count = 16, bad_read_reg = -1, bad_write_reg = -1, corrupt_verify;
static int drain_fifo;
static unsigned checks;
static unsigned startup_poll_count;
static void test_startup_poll(void) { ++startup_poll_count; }

#define CHECK(x) do { ++checks; if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); return 1; } } while (0)
uint32_t HAL_GetTick(void) { return now_ms; }
void HAL_Delay(uint32_t delay) { now_ms += delay; }
void UART_Printf(const char *fmt, ...) { (void)fmt; }

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus, uint16_t slave,
    uint16_t reg, uint16_t regsize, uint8_t *data, uint16_t len, uint32_t timeout)
{
    (void)bus; (void)regsize; (void)timeout;
    if (slave != 0xD0 || (int)reg == bad_write_reg) return HAL_ERROR;
    if (reg == 0x6D && len == 2) { bank = data[0]; address = data[1]; return HAL_OK; }
    if (reg == 0x6F) {
        unsigned offset = bank * 256U + address;
        if (offset + len > sizeof(dmp_mem)) return HAL_ERROR;
        memcpy(dmp_mem + offset, data, len); ++fw_writes; return HAL_OK;
    }
    if (reg + len > sizeof(regs)) return HAL_ERROR;
    memcpy(regs + reg, data, len);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus, uint16_t slave,
    uint16_t reg, uint16_t regsize, uint8_t *data, uint16_t len, uint32_t timeout)
{
    (void)bus; (void)regsize; (void)timeout;
    if (slave != 0xD0 || (int)reg == bad_read_reg) return HAL_ERROR;
    if (reg == 0x6F) {
        unsigned offset = bank * 256U + address;
        if (offset + len > sizeof(dmp_mem)) return HAL_ERROR;
        memcpy(data, dmp_mem + offset, len);
        if (corrupt_verify && len) data[0] ^= 1U;
        return HAL_OK;
    }
    if (reg == 0x72 && len == 2) {
        data[0] = (uint8_t)(fifo_count >> 8); data[1] = (uint8_t)fifo_count;
        return HAL_OK;
    }
    if (reg == 0x74 && len == 16) {
        memcpy(data, fifo_data, 16);
        if (drain_fifo) fifo_count -= 16;
        return HAL_OK;
    }
    if (reg + len > sizeof(regs)) return HAL_ERROR;
    memcpy(data, regs + reg, len);
    return HAL_OK;
}

static void set_quat(float angle_deg)
{
    float half = angle_deg * 0.01745329252f * 0.5f;
    int32_t q[4] = {(int32_t)(cosf(half) * 1073741824.0f),
                    (int32_t)(sinf(half) * 1073741824.0f), 0, 0};
    for (int i = 0; i < 4; ++i) {
        uint32_t value = (uint32_t)q[i];
        fifo_data[4*i] = (uint8_t)(value >> 24);
        fifo_data[4*i+1] = (uint8_t)(value >> 16);
        fifo_data[4*i+2] = (uint8_t)(value >> 8);
        fifo_data[4*i+3] = (uint8_t)value;
    }
}

static void reset_mock(void)
{
    memset(regs, 0, sizeof(regs)); memset(dmp_mem, 0, sizeof(dmp_mem));
    regs[0x09] = 1; /* MPU6050 revision 2: full accelerometer sensitivity. */
    fifo_count = 16; bad_read_reg = bad_write_reg = -1; corrupt_verify = 0;
    drain_fifo = 0;
    now_ms = fw_writes = bank = address = 0;
    set_quat(0.0f);
}

int main(void)
{
    reset_mock();
    DMP_PlatformSetStartupPoll(test_startup_poll);
    CHECK(DMP_AttitudeInit() == HAL_OK);
    CHECK(startup_poll_count > 0U);
    {
        unsigned after_init = startup_poll_count;
        CHECK(DMP_AttitudePoll() == HAL_OK);
        CHECK(startup_poll_count == after_init); /* no LCD work in control reads */
    }
    DMP_PlatformSetStartupPoll(NULL);
    CHECK(fw_writes > 190); /* Actual image written and read back in chunks. */
    CHECK((regs[0x1B] & 0x18U) == 0x18U && (regs[0x1C] & 0x18U) == 0U);
    DMP_AttitudeResetAngle(5.0f);
    CHECK(fabsf(DMP_AttitudeGetAngle(0.0f) - 5.0f) < 0.05f);
    set_quat(20.0f); now_ms += 5;
    CHECK(DMP_AttitudePoll() == HAL_OK);
    CHECK(fabsf(DMP_AttitudeGetAngle(0.0f) - 25.0f) < 0.1f);
    set_quat(-20.0f); now_ms += 5;
    CHECK(DMP_AttitudePoll() == HAL_OK);
    CHECK(fabsf(DMP_AttitudeGetAngle(0.0f) - (-15.0f)) < 0.1f);
    fifo_count = 0; now_ms += 5;
    CHECK(DMP_AttitudePoll() == HAL_OK);
    CHECK(fabsf(DMP_AttitudeGetAngle(10.0f) - (-14.95f)) < 0.1f);
    now_ms += 11;
    CHECK(DMP_AttitudePoll() == HAL_TIMEOUT);

    reset_mock(); CHECK(DMP_AttitudeInit() == HAL_OK);
    fifo_count = 48; drain_fifo = 1;
    CHECK(DMP_AttitudePoll() == HAL_OK); CHECK(fifo_count == 0); /* catch up 3 packets */
    fifo_count = 64; CHECK(DMP_AttitudePoll() == HAL_ERROR); /* excessive backlog */
    CHECK(DMP_AttitudeLastError() == 4U);
    fifo_count = 16; bad_read_reg = 0x72; CHECK(DMP_AttitudePoll() == HAL_ERROR);
    CHECK(DMP_AttitudeLastError() == 1U);

    reset_mock(); CHECK(DMP_AttitudeInit() == HAL_OK);
    memset(fifo_data, 0, sizeof(fifo_data));
    CHECK(DMP_AttitudePoll() == HAL_ERROR); /* corrupt quaternion */

    reset_mock(); corrupt_verify = 1;
    CHECK(DMP_AttitudeInit() == HAL_ERROR); /* firmware verification failed */
    reset_mock(); fifo_count = 0;
    CHECK(DMP_AttitudeInit() == HAL_ERROR); /* no first frame: never arm */
    reset_mock(); bad_write_reg = 0x6F;
    CHECK(DMP_AttitudeInit() == HAL_ERROR); /* bus error during firmware load */
    printf("PASS: %u DMP initialization, signed angles, FIFO, error and timeout checks\n", checks);
    return 0;
}
