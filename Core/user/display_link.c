#include "display_link.h"
#include <stddef.h>
#include <limits.h>

#define RX_SIZE 128U
#define PAYLOAD_SIZE 26U
#define FRAME_SIZE 32U
static UART_HandleTypeDef *link_uart;
static uint8_t tx_frame[FRAME_SIZE];
static volatile uint8_t tx_busy;
static uint16_t sequence;
static uint8_t rx_byte, rx_data[RX_SIZE];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_error;

static void put16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void put32(uint8_t *p, int32_t v) {
    uint32_t u=(uint32_t)v;
    p[0]=(uint8_t)u; p[1]=(uint8_t)(u>>8); p[2]=(uint8_t)(u>>16); p[3]=(uint8_t)(u>>24);
}
// Fixed point in hundredths; clamp before conversion (including infinities).
static int32_t to100(float x) {
    if (x != x) return 0;
    if (x >= 20000000.0f) return INT32_C(2000000000);
    if (x <= -20000000.0f) return -INT32_C(2000000000);
    return (int32_t)(x * 100.0f);
}
// CRC-16/CCITT-FALSE: poly 0x1021, initial 0xffff, no reflection, xorout 0.
static uint16_t crc16(const uint8_t *p, unsigned len) {
    uint16_t crc=0xffffU;
    while(len--) {
        crc ^= (uint16_t)(*p++) << 8;
        for(unsigned i=0;i<8U;i++) crc=(crc&0x8000U)?(uint16_t)((crc<<1)^0x1021U):(uint16_t)(crc<<1);
    }
    return crc;
}
HAL_StatusTypeDef DisplayLink_Init(UART_HandleTypeDef *uart) {
    if (!uart) return HAL_ERROR;
    link_uart=uart; tx_busy=0U; sequence=0U;
    rx_head=rx_tail=0U; rx_error=0U;
    return HAL_UART_Receive_IT(link_uart, &rx_byte, 1U);
}
HAL_StatusTypeDef DisplayLink_SendStatus(const DisplayStatus *s) {
    if (!link_uart || !s) return HAL_ERROR;
    // Only main calls this function. The ISR can clear busy but never writes tx_frame.
    if (tx_busy) return HAL_BUSY;
    tx_frame[0]=0xA5U; tx_frame[1]=0x5AU; tx_frame[2]=1U; tx_frame[3]=PAYLOAD_SIZE;
    put16(&tx_frame[4], sequence++);
    put32(&tx_frame[6], to100(s->angle)); put32(&tx_frame[10], to100(s->gyro));
    put32(&tx_frame[14], to100(s->encoder_a)); put32(&tx_frame[18], to100(s->encoder_b));
    put16(&tx_frame[22],s->pwm_a); put16(&tx_frame[24],s->pwm_b);
    tx_frame[26]=s->state; tx_frame[27]=s->fault; tx_frame[28]=s->stby; tx_frame[29]=s->key;
    put16(&tx_frame[30],crc16(&tx_frame[2],28U));
    tx_busy=1U;
    HAL_StatusTypeDef result=HAL_UART_Transmit_IT(link_uart,tx_frame,FRAME_SIZE);
    if(result != HAL_OK) tx_busy=0U;
    return result;
}
void DisplayLink_OnTxComplete(UART_HandleTypeDef *uart) {
    if(uart==link_uart) tx_busy=0U;
}
void DisplayLink_OnRxComplete(UART_HandleTypeDef *uart) {
    if(uart!=link_uart) return;
    uint16_t next=(uint16_t)((rx_head+1U)%RX_SIZE);
    if(next==rx_tail) rx_error=1U;
    else { rx_data[rx_head]=rx_byte; rx_head=next; }
    if(HAL_UART_Receive_IT(link_uart,&rx_byte,1U)!=HAL_OK) rx_error=1U;
}
void DisplayLink_OnError(UART_HandleTypeDef *uart) {
    if(uart!=link_uart) return;
    rx_error=1U;
    // UART RX errors are recovered by rearming reception; TX buffer remains owned
    // by HAL until the TX completion callback, so it is never overwritten here.
    (void)HAL_UART_Receive_IT(link_uart,&rx_byte,1U);
}
uint8_t DisplayLink_ReadByte(uint8_t *out) {
    if(!out || rx_head==rx_tail) return 0U;
    *out=rx_data[rx_tail]; rx_tail=(uint16_t)((rx_tail+1U)%RX_SIZE); return 1U;
}
uint8_t DisplayLink_TakeRxError(void) {
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    uint8_t e=rx_error; rx_error=0U;
    if(e) rx_tail=rx_head;
    __set_PRIMASK(mask); return e;
}
