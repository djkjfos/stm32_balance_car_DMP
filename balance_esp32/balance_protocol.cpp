#include "balance_protocol.h"
#include <string.h>
#include <stdint.h>

static uint16_t crc16(const uint8_t *p, unsigned len) {
  uint16_t crc = 0xffff;
  while (len--) {
    crc ^= (uint16_t)*p++ << 8;
    for (unsigned i=0; i<8; ++i) crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
  }
  return crc;
}
static uint16_t u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static int32_t i32(const uint8_t *p) {
  uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  return u <= INT32_MAX ? (int32_t)u : (int32_t)((int64_t)u - INT64_C(4294967296));
}
void BalanceFrameParser::reset() { used_=0; }
bool BalanceFrameParser::push(uint8_t byte, ST7789_Status &status) {
  if (used_ == 0 && byte != 0xA5) return false;
  if (used_ == 1 && byte != 0x5A) { used_ = byte == 0xA5 ? 1 : 0; return false; }
  frame_[used_++] = byte;
  if (used_ < 32) return false;
  const bool valid = frame_[2] == 1 && frame_[3] == 26 &&
      crc16(frame_ + 2, 28) == u16(frame_ + 30) &&
      frame_[26] <= 3 && frame_[28] <= 1 && frame_[29] <= 1;
  if (valid) {
    ST7789_Status next{};
    next.angle = i32(frame_+6) / 100.0f; next.gyro = i32(frame_+10) / 100.0f;
    next.encoder_a = i32(frame_+14) / 100.0f; next.encoder_b = i32(frame_+18) / 100.0f;
    next.pwm_a = u16(frame_+22); next.pwm_b = u16(frame_+24);
    next.state=frame_[26]; next.fault=frame_[27]; next.stby=frame_[28]; next.key=frame_[29];
    status=next; used_=0; return true;
  }
  // Preserve any possible sync header inside a damaged frame.
  for (uint8_t i=1; i<31; ++i) {
    if (frame_[i]==0xA5 && frame_[i+1]==0x5A) {
      used_=32-i; memmove(frame_,frame_+i,used_); return false;
    }
  }
  used_ = frame_[31]==0xA5 ? 1 : 0; if (used_) frame_[0]=0xA5;
  return false;
}
