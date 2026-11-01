#include "dmp_platform.h"
#include "i2c.h"

/* Initialization runs with motors disabled; runtime transactions have a
 * tighter timeout so the existing control-overrun protection can intervene. */
static int transfer_failed;
static int is_running;
static void (*startup_poll)(void);

void DMP_PlatformClearError(void) { transfer_failed = 0; }
int DMP_PlatformHasError(void) { return transfer_failed; }
void DMP_PlatformSetRunning(int running) { is_running = running; }
void DMP_PlatformSetStartupPoll(void (*poll)(void)) { startup_poll = poll; }
static void poll_startup_display(void)
{
    if (!is_running && startup_poll != NULL) startup_poll();
}

int i2c_write(unsigned char slave, unsigned char reg, unsigned char length,
              unsigned char const *data)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)slave << 1,
        reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, length, is_running ? 4U : 20U);
    poll_startup_display();
    if (status != HAL_OK) { transfer_failed = 1; return -1; }
    return 0;
}

int i2c_read(unsigned char slave, unsigned char reg, unsigned char length,
             unsigned char *data)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, (uint16_t)slave << 1,
        reg, I2C_MEMADD_SIZE_8BIT, data, length, is_running ? 4U : 20U);
    poll_startup_display();
    if (status != HAL_OK) { transfer_failed = 1; return -1; }
    return 0;
}

void delay_ms(unsigned long ms)
{
    if (is_running || startup_poll == NULL) { HAL_Delay((uint32_t)ms); return; }
    while (ms-- != 0UL) { poll_startup_display(); HAL_Delay(1U); }
}
void get_ms(unsigned long *count) { *count = HAL_GetTick(); }
