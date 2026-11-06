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

/* 拆分16/32bit转成单字节 */
static void put16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void put32(uint8_t *p, int32_t v) {
    uint32_t u=(uint32_t)v;
    p[0]=(uint8_t)u; p[1]=(uint8_t)(u>>8); p[2]=(uint8_t)(u>>16); p[3]=(uint8_t)(u>>24);
}
/* 将浮点数放大100倍成整形，解放Flash
 * x != x 因为NULL != NULL 排空处理
 */
static int32_t to100(float x) {
    if (x != x) return 0;
    if (x >= 20000000.0f) return INT32_C(2000000000);
    if (x <= -20000000.0f) return -INT32_C(2000000000);
    return (int32_t)(x * 100.0f);
}
// CRC校验
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
//设置为小端传输模式
HAL_StatusTypeDef DisplayLink_SendStatus(const DisplayStatus *s) {
    if (!link_uart || !s) return HAL_ERROR;
    // 打包数据到设置发送出去，接收方拼接
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
// 发送完成标准复位，0是完成了
void DisplayLink_OnTxComplete(UART_HandleTypeDef *uart) {
    if(uart==link_uart) tx_busy=0U;
}
// 环形缓冲区进行接收数据
void DisplayLink_OnRxComplete(UART_HandleTypeDef *uart) {
    if(uart!=link_uart) return;
    uint16_t next=(uint16_t)((rx_head+1U)%RX_SIZE);
    if(next==rx_tail) rx_error=1U;
    else { rx_data[rx_head]=rx_byte; rx_head=next; }
    if(HAL_UART_Receive_IT(link_uart,&rx_byte,1U)!=HAL_OK) rx_error=1U;
}
// 出错返回接收
void DisplayLink_OnError(UART_HandleTypeDef *uart) {
    if(uart!=link_uart) return;
    rx_error=1U;
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

// 接收ESP32发来的控制命令，小端拼接
// Control frame: 5A A5, version 01, type 02, two little-endian signed
// hundredths (speed, turn), CRC-16/CCITT-FALSE over bytes 2..7, LE.
uint8_t DisplayLink_PollCommand(float *speed, float *turn) {
    static uint8_t frame[10], used;
    uint8_t byte, valid=0U;
    if (!speed || !turn) return 0U;
    if (DisplayLink_TakeRxError()) used=0U;
    while (DisplayLink_ReadByte(&byte)) {
        if (used==0U && byte!=0x5AU) continue;
        if (used==1U && byte!=0xA5U) { used=(byte==0x5AU)?1U:0U; continue; }
        frame[used++]=byte;
        if (used==sizeof(frame)) {
            uint8_t frame_valid=0U;
            uint16_t crc=(uint16_t)(frame[8] | ((uint16_t)frame[9]<<8));
            if (frame[2]==1U && frame[3]==2U && crc16(&frame[2],6U)==crc) {
                uint16_t a=(uint16_t)(frame[4] | ((uint16_t)frame[5]<<8));
                uint16_t b=(uint16_t)(frame[6] | ((uint16_t)frame[7]<<8));
                int32_t sa=a<=32767U ? (int32_t)a : (int32_t)a-65536;
                int32_t sb=b<=32767U ? (int32_t)b : (int32_t)b-65536;
                *speed=(float)sa/100.0f; *turn=(float)sb/100.0f;
                valid=frame_valid=1U;
            }
            // Recover a potential header already present in a damaged frame.
            uint8_t start=1U;
            while (start<9U && !(frame[start]==0x5AU && frame[start+1U]==0xA5U)) ++start;
            if (!frame_valid && start<9U) {
                used=(uint8_t)(10U-start);
                for (uint8_t i=0U; i<used; ++i) frame[i]=frame[start+i];
            } else if (!frame_valid && frame[9]==0x5AU) { frame[0]=0x5AU; used=1U; }
            else used=0U;
        }
    }
    return valid;
}
