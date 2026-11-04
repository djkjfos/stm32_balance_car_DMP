#pragma once
#include <stdint.h>
#include "st7789.h"

class BalanceFrameParser {
public:
  bool push(uint8_t byte, ST7789_Status &status);
  void reset();
private:
  uint8_t frame_[32] = {};
  uint8_t used_ = 0;
};
