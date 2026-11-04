#include "st7789.h"
#include "balance_link.h"
void setup() {
  ST7789_Init();
  ST7789_Status waiting = {};
  waiting.state = 255;
  ST7789_SetStatus(&waiting);
  BalanceLink_Init();
}
void loop() {
  BalanceLink_Poll();
  ST7789_Poll();
}
