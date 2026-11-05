#include "balance_link.h"
#include "balance_protocol.h"
#include <math.h>
static HardwareSerial BalanceSerial(1);  // ESP32的UART1串口对象
static BalanceFrameParser parser;         // 帧解析器实例
static bool online = false;               // STM32在线状态标记
static uint32_t last_frame = 0;           // 最后一帧收到的时间戳
void BalanceLink_Init() {
  BalanceSerial.begin(BALANCE_UART_BAUD, SERIAL_8N1, BALANCE_UART_RX, BALANCE_UART_TX);     // 启动串口
  ST7789_SetLinkOnline(false);
}
void BalanceLink_Poll() {
  ST7789_Status status{};
  unsigned budget=128;
  while (budget-- && BalanceSerial.available()) {
    if (parser.push((uint8_t)BalanceSerial.read(), status)) {
      ST7789_SetStatus(&status); ST7789_SetLinkOnline(true); online=true; last_frame=millis();
    }
  }
  if (online && (uint32_t)(millis()-last_frame)>=1000U) {
    online=false; parser.reset(); ST7789_Status blank{}; blank.state=255;
    ST7789_SetStatus(&blank); ST7789_SetLinkOnline(false);
  }
}
bool BalanceLink_Send(const uint8_t *data, size_t length) {
  if (!data || !length || length > 128 || BalanceSerial.availableForWrite() < (int)length) return false;
  return BalanceSerial.write(data,length)==length;
}
bool BalanceLink_SendTargets(float speed, float turn) {
  if (!isfinite(speed) || !isfinite(turn) || speed < -327.68f || speed > 327.67f ||
      turn < -327.68f || turn > 327.67f) return false;
  int32_t s=(int32_t)lroundf(speed*100.0f), t=(int32_t)lroundf(turn*100.0f);
  if (s < -32768 || s > 32767 || t < -32768 || t > 32767) return false;
  uint16_t a=(uint16_t)s, b=(uint16_t)t;
  uint8_t frame[10]={0x5A,0xA5,1,2,(uint8_t)a,(uint8_t)(a>>8),(uint8_t)b,(uint8_t)(b>>8),0,0};
  uint16_t crc=0xffffU;
  for (unsigned i=2; i<8; ++i) {
    crc ^= (uint16_t)frame[i]<<8;
    for (unsigned bit=0; bit<8; ++bit)
      crc=(crc&0x8000U)?(uint16_t)((crc<<1)^0x1021U):(uint16_t)(crc<<1);
  }
  frame[8]=(uint8_t)crc; frame[9]=(uint8_t)(crc>>8);
  return BalanceLink_Send(frame,sizeof(frame));
}
