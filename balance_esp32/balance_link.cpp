#include "balance_link.h"
#include "balance_protocol.h"
static HardwareSerial BalanceSerial(1);
static BalanceFrameParser parser;
static bool online=false;
static uint32_t last_frame=0;
void BalanceLink_Init() {
  BalanceSerial.begin(BALANCE_UART_BAUD, SERIAL_8N1, BALANCE_UART_RX, BALANCE_UART_TX);
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
