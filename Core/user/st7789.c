#include "st7789.h"
#include "spi.h"
#include "st7789_font.h"
#include <stdio.h>
#include <string.h>

/* CubeMX 的 spi.h 通常只导出 hspi1；DMA 句柄在 spi.c 中定义。 */
extern DMA_HandleTypeDef hdma_spi1_tx;

/* 每次最多 4 行、1920 字节；整个工程不分配全屏帧缓冲。 */
#define STRIP_ROWS 4U
#define TX_TIMEOUT_MS 10U
#define FRAME_INTERVAL_MS 200U
#define TEXT_ROWS 10U
#define BACKGROUND_COLOR 0x0000U /* 黑色，RGB565 */
typedef struct { uint8_t command, count, data[14]; uint16_t delay_ms; } InitCommand;
/* 沿用本机 st7789 示例的面板电压/伽马设置，RGB565，竖屏，反色开启。 */
static const InitCommand init_commands[] = {
    {0x11,0,{0},120}, {0x36,1,{ST7789_MADCTL},0}, {0x3A,1,{0x05},0},
    {0xB2,5,{0x0C,0x0C,0x00,0x33,0x33},0}, {0xB7,1,{0x35},0},
    {0xBB,1,{0x32},0}, {0xC2,1,{0x01},0}, {0xC3,1,{0x15},0},
    {0xC4,1,{0x20},0}, {0xC6,1,{0x0F},0}, {0xD0,2,{0xA4,0xA1},0},
    {0xE0,14,{0xD0,0x08,0x0E,0x09,0x09,0x05,0x31,0x33,0x48,0x17,0x14,0x15,0x31,0x34},0},
    {0xE1,14,{0xD0,0x08,0x0E,0x09,0x09,0x15,0x31,0x33,0x48,0x17,0x14,0x15,0x31,0x34},0},
    {0x21,0,{0},0}, {0x29,0,{0},20}
};
static const uint8_t cmd_column = 0x2A, cmd_row = 0x2B, cmd_memory = 0x2C;
static uint8_t pixels[ST7789_WIDTH * STRIP_ROWS * 2U];
static uint8_t column_data[4], row_data[4];
static volatile uint8_t dma_done, dma_error;
static uint8_t active, lcd_status, mode, init_index, init_phase, strip_phase;
static uint8_t text_row, text_y, drawing;
static uint16_t clear_y;
static uint32_t started_ms, wait_until, frame_ms;
static ST7789_Status latest, snapshot;
static char text[21];
static uint16_t foreground;

/* DMA IRQ 仅标记完成；SPI BSY 在主循环检查，IRQ 内没有等待与延时。 */
static void dma_finished(DMA_HandleTypeDef *dma)
{
    (void)dma;
    CLEAR_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);
    dma_done = 1U;
}
static void dma_failed(DMA_HandleTypeDef *dma)
{
    (void)dma;
    CLEAR_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);
    dma_error = 1U;
}
static void stop_display(void)
{
    CLEAR_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);
    __HAL_DMA_DISABLE(&hdma_spi1_tx);
    __HAL_DMA_DISABLE_IT(&hdma_spi1_tx, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);
    __HAL_SPI_DISABLE(&hspi1);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
    active = 0U;
    lcd_status = 2U;
}
static void send_dma(uint8_t is_data, const uint8_t *data, uint16_t count)
{
    dma_done = dma_error = 0U;
    started_ms = HAL_GetTick();
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, is_data ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    active = 1U;
    /* 本驱动独占 SPI1 TX DMA，不调用 HAL_SPI_Transmit_DMA：
     * HAL 的 SPI DMA 完成处理内部会等待 BSY；这里将等待改为主循环轮询。 */
    if (HAL_DMA_Start_IT(&hdma_spi1_tx, (uint32_t)(uintptr_t)data,
                         (uint32_t)(uintptr_t)&hspi1.Instance->DR, count) != HAL_OK) {
        stop_display();
        return;
    }
    __HAL_SPI_ENABLE(&hspi1);
    SET_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);
}

void ST7789_Init(void)
{
    /* 在 MX_DMA_Init/MX_SPI1_Init 后调用一次；重试需先确认旧传输已停止。 */
    lcd_status = active = dma_done = dma_error = 0U;
    mode = init_index = init_phase = strip_phase = drawing = 0U;
    clear_y = 0U;
    memset(&latest, 0, sizeof(latest));
    hdma_spi1_tx.XferCpltCallback = dma_finished;
    hdma_spi1_tx.XferErrorCallback = dma_failed;
    hdma_spi1_tx.XferHalfCpltCallback = NULL;
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_RESET);
    wait_until = HAL_GetTick() + 20U;
}
void ST7789_SetStatus(const ST7789_Status *status)
{
    if (status != NULL) latest = *status;
}
uint8_t ST7789_GetStatus(void) { return lcd_status; }

/* 为当前 4 行条带设置窗口；每次 Poll 最多启动一次 DMA。 */
static uint8_t push_strip(uint16_t y)
{
    uint16_t x0 = ST7789_X_OFFSET, x1 = ST7789_X_OFFSET + ST7789_WIDTH - 1U;
    uint16_t y0 = y + ST7789_Y_OFFSET, y1 = y0 + STRIP_ROWS - 1U;
    switch (strip_phase++) {
    case 0:
        column_data[0] = (uint8_t)(x0 >> 8); column_data[1] = (uint8_t)x0;
        column_data[2] = (uint8_t)(x1 >> 8); column_data[3] = (uint8_t)x1;
        row_data[0] = (uint8_t)(y0 >> 8); row_data[1] = (uint8_t)y0;
        row_data[2] = (uint8_t)(y1 >> 8); row_data[3] = (uint8_t)y1;
        send_dma(0, &cmd_column, 1); break;
    case 1: send_dma(1, column_data, 4); break;
    case 2: send_dma(0, &cmd_row, 1); break;
    case 3: send_dma(1, row_data, 4); break;
    case 4: send_dma(0, &cmd_memory, 1); break;
    case 5: send_dma(1, pixels, sizeof(pixels)); break;
    default: strip_phase = 0U; return 1U;
    }
    return 0U;
}
static void format_line(void)
{
    static const char *const names[] = {"CALIBRATING", "READY", "RUNNING", "FAULT"};
    foreground = 0xFFFFU;
    switch (text_row) {
    case 0: snprintf(text, sizeof(text), "BALANCE / ST7789"); foreground = 0x07FFU; break;
    case 1:
        snprintf(text, sizeof(text), "STATE %s", snapshot.state < 4U ? names[snapshot.state] : "UNKNOWN");
        foreground = snapshot.state == 2U ? 0x07E0U : (snapshot.state == 3U ? 0xF800U : 0xFFE0U); break;
    case 2: snprintf(text, sizeof(text), "FAULT %u", (unsigned)snapshot.fault); break;
    case 3: snprintf(text, sizeof(text), "ANGLE %8.2f", snapshot.angle); break;
    case 4: snprintf(text, sizeof(text), "GYRO  %8.2f", snapshot.gyro); break;
    case 5: snprintf(text, sizeof(text), "ENC A %8.0f", snapshot.encoder_a); break;
    case 6: snprintf(text, sizeof(text), "ENC B %8.0f", snapshot.encoder_b); break;
    case 7: snprintf(text, sizeof(text), "PWM A %5u", (unsigned)snapshot.pwm_a); break;
    case 8: snprintf(text, sizeof(text), "PWM B %5u", (unsigned)snapshot.pwm_b); break;
    default: snprintf(text, sizeof(text), "STBY %u KEY %u", (unsigned)snapshot.stby, (unsigned)snapshot.key); break;
    }
}
static void render_background_strip(void)
{
    for (uint16_t i = 0; i < sizeof(pixels); i += 2U) {
        pixels[i] = (uint8_t)(BACKGROUND_COLOR >> 8);
        pixels[i + 1U] = (uint8_t)BACKGROUND_COLOR;
    }
}
static void render_strip(void)
{
    size_t length = strlen(text);
    for (uint16_t y = 0; y < STRIP_ROWS; ++y) {
        uint8_t bit = (uint8_t)((text_y + y) / 2U);
        for (uint16_t x = 0; x < ST7789_WIDTH; ++x) {
            uint16_t color = BACKGROUND_COLOR;
            size_t character = x / 12U;
            if (character < length) {
                uint8_t c = (uint8_t)text[character];
                if (c >= 32U && c <= 126U && (st7789_font[c - 32U][(x % 12U) / 2U] & (1U << bit)))
                    color = foreground;
            }
            uint16_t index = (uint16_t)((y * ST7789_WIDTH + x) * 2U);
            pixels[index] = (uint8_t)(color >> 8); pixels[index + 1U] = (uint8_t)color;
        }
    }
}
void ST7789_Poll(void)
{
    uint32_t now = HAL_GetTick();
    if (lcd_status == 2U) return;
    if (active) {
        if (dma_error) { stop_display(); return; }
        if (dma_done && __HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY) == RESET) {
            __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
            HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
            active = 0U;
        } else {
            if ((uint32_t)(now - started_ms) >= TX_TIMEOUT_MS) stop_display();
            return;
        }
    }
    if ((int32_t)(now - wait_until) < 0) return;
    if (mode == 0U) {
        HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET);
        wait_until = now + 120U; mode = 1U; return;
    }
    if (mode == 1U) {
        if (init_index >= sizeof(init_commands) / sizeof(init_commands[0])) {
            render_background_strip(); mode = 2U; return;
        }
        const InitCommand *cmd = &init_commands[init_index];
        if (init_phase == 0U) { send_dma(0, &cmd->command, 1U); init_phase = 1U; return; }
        if (init_phase == 1U && cmd->count) { send_dma(1, cmd->data, cmd->count); init_phase = 2U; return; }
        wait_until = now + cmd->delay_ms; ++init_index; init_phase = 0U; return;
    }
    if (mode == 2U) {
        if (push_strip(clear_y)) {
            clear_y += STRIP_ROWS;
            if (clear_y >= ST7789_HEIGHT) { mode = 3U; lcd_status = 1U; frame_ms = now - FRAME_INTERVAL_MS; }
        }
        return;
    }
    if (!drawing) {
        if ((uint32_t)(now - frame_ms) < FRAME_INTERVAL_MS) return;
        snapshot = latest; frame_ms = now; drawing = 1U; text_row = text_y = 0U;
    }
    if (strip_phase == 0U) {
        if (text_y == 0U) format_line();
        render_strip();
    }
    if (push_strip((uint16_t)(8U + text_row * 22U + text_y))) {
        text_y += STRIP_ROWS;
        if (text_y >= 16U) { text_y = 0U; if (++text_row >= TEXT_ROWS) drawing = 0U; }
    }
}
