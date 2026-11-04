#pragma once
#include <stdint.h>
// Candidate pins, change to match your ESP32-S3 board and wiring.
#define ST7789_PIN_SCK 7
#define ST7789_PIN_MOSI 6
#define ST7789_PIN_CS 15
#define ST7789_PIN_DC 5
#define ST7789_PIN_RST 8
#define ST7789_PIN_BL (-1) // -1 means backlight wired separately; verify your module
#define ST7789_WIDTH 240U
#define ST7789_HEIGHT 240U
#define ST7789_X_OFFSET 0U
#define ST7789_Y_OFFSET 0U
#define ST7789_MADCTL 0x00U
struct ST7789_Status {
  float angle, gyro, encoder_a, encoder_b;
  uint16_t pwm_a, pwm_b;
  uint8_t state, fault, stby, key;
};
void ST7789_Init(void);
void ST7789_SetLinkOnline(bool online);
void ST7789_SetStatus(const ST7789_Status *status);
void ST7789_Poll(void);
uint8_t ST7789_GetStatus(void);
