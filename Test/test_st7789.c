/* 模拟异步 DMA 与 LCD 命令解码，测试的是实际 st7789.c，不涉及电机硬件。 */
#include "st7789.h"
#include "spi.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static SPI_TypeDef spi;
SPI_HandleTypeDef hspi1 = {&spi};
DMA_HandleTypeDef hdma_spi1_tx;
static uint32_t now, assertions, tx_size, transfers, pixel_count;
static const uint8_t *tx_data;
static uint8_t pending, dc, cs, start_error, command, sent[1920];
static uint16_t framebuffer[240*240], x0,x1,y0,y1,x,y;
#define EXPECTED_BACKGROUND 0x0000U
static uint32_t reset_low_ms, reset_high_ms, sleep_out_ms, first_window_ms;
#define CHECK(c) do { ++assertions; if (!(c)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c); exit(1); } } while(0)
uint32_t HAL_GetTick(void) { return now; }
void test_disable_dma(void) { pending=0; }
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState state)
{
    (void)port;
    if(pin==LCD_CS_Pin) cs=(uint8_t)state;
    if(pin==LCD_DC_Pin) dc=(uint8_t)state;
    if(pin==LCD_RES_Pin) {
        if(state==GPIO_PIN_RESET) reset_low_ms=now;
        else reset_high_ms=now;
    }
}
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *dma, uint32_t from, uint32_t to, uint32_t count)
{
    uintptr_t address = ((uintptr_t)&spi & ~(uintptr_t)0xFFFFFFFFU) | from;
    (void)dma; (void)to;
    if(start_error) return HAL_ERROR;
    CHECK(!pending); CHECK(!cs); CHECK(count>0 && count<=1920);
    tx_data=(const uint8_t *)address; tx_size=count;
    memcpy(sent,tx_data,count);
    pending=1; spi.flags=SPI_FLAG_BSY; ++transfers;
    return HAL_OK;
}
static void complete(void)
{
    CHECK(pending); CHECK(!cs); CHECK(memcmp(sent,tx_data,tx_size)==0);
    if(!dc) {
        CHECK(tx_size==1); command=sent[0];
        if(command==0x11) sleep_out_ms=now;
        if(command==0x2A && !first_window_ms) first_window_ms=now;
        if(command==0x2C) { x=x0; y=y0; }
    } else if(command==0x2A || command==0x2B) {
        CHECK(tx_size==4);
        uint16_t a=(uint16_t)(sent[0]*256U+sent[1]), b=(uint16_t)(sent[2]*256U+sent[3]);
        CHECK(a<=b && b<240);
        if(command==0x2A) { x0=a; x1=b; } else { y0=a; y1=b; }
    } else if(command==0x2C) {
        CHECK(tx_size==1920);
        for(uint32_t i=0;i<tx_size;i+=2) {
            CHECK(x<=x1 && y<=y1);
            framebuffer[y*240U+x]=(uint16_t)(sent[i]*256U+sent[i+1]);
            ++pixel_count;
            if(++x>x1) { x=x0; ++y; }
        }
        CHECK(y==y1+1U);
    }
    pending=0; spi.flags=0;
    hdma_spi1_tx.XferCpltCallback(&hdma_spi1_tx);
}
static void reset_test(uint32_t tick)
{
    memset(&spi,0,sizeof(spi)); memset(&hdma_spi1_tx,0,sizeof(hdma_spi1_tx));
    memset(framebuffer,0xFF,sizeof(framebuffer));
    pending=start_error=0; cs=1; now=tick; transfers=pixel_count=0;
    reset_low_ms=reset_high_ms=sleep_out_ms=first_window_ms=0;
    ST7789_Init();
}
static void step(void)
{
    ST7789_Poll();
    if(pending) complete();
}
int main(void)
{
    ST7789_Status sample = {-12.34f, 0.5f, 2,-3,78,90,2,0,1,0};
    reset_test(0); ST7789_SetStatus(&sample);
    for(now=0;now<20;++now) { ST7789_Poll(); CHECK(transfers==0); }
    ST7789_Poll(); CHECK(reset_high_ms-reset_low_ms>=20);
    for(now=21;now<140;++now) { ST7789_Poll(); CHECK(transfers==0); }
    for(unsigned i=0;i<10000 && pixel_count<96000U;++i) { step(); ++now; }
    CHECK(ST7789_GetStatus()==1); CHECK(pixel_count==96000U);
    CHECK(first_window_ms-sleep_out_ms>=120);
    CHECK(framebuffer[0]==EXPECTED_BACKGROUND);
    CHECK(framebuffer[239*240+239]==EXPECTED_BACKGROUND);
    unsigned nonzero=0;
    for(unsigned i=0;i<240*240;++i) if(framebuffer[i]) ++nonzero;
    CHECK(nonzero>1000);
    unsigned white=0;
    for(unsigned row=74;row<90;++row)
        for(unsigned col=0;col<240;++col) if(framebuffer[row*240+col]==0xFFFFU) ++white;
    CHECK(white>0);
    /* READY 比 RUNNING 短，整行背景必须覆盖旧字符。 */
    sample.state=1; ST7789_SetStatus(&sample);
    uint32_t before=pixel_count;
    for(unsigned i=0;i<10000 && pixel_count<before+38400U;++i) { step(); ++now; }
    CHECK(pixel_count==before+38400U);
    for(unsigned row=30;row<46;++row)
        for(unsigned col=11*12;col<240;++col)
            CHECK(framebuffer[row*240+col]==EXPECTED_BACKGROUND);
    /* 时间回绕期间复位延时仍成立。 */
    reset_test(0xFFFFFFF0U);
    for(unsigned i=0;i<20;++i) { ST7789_Poll(); CHECK(transfers==0); ++now; }
    ST7789_Poll(); CHECK(reset_high_ms-reset_low_ms==20U);
    /* DMA 无完成中断、DMA 错误、SPI BSY 卡住、启动失败均只停用屏幕。 */
    reset_test(0); now=20; ST7789_Poll(); now=140; ST7789_Poll(); CHECK(pending);
    now=149; ST7789_Poll(); CHECK(ST7789_GetStatus()==0);
    now=150; ST7789_Poll(); CHECK(ST7789_GetStatus()==2 && cs && !spi.enabled && !pending);
    before=transfers; for(unsigned i=0;i<100;++i) ST7789_Poll(); CHECK(transfers==before);
    reset_test(0); now=20; ST7789_Poll(); now=140; ST7789_Poll();
    hdma_spi1_tx.XferErrorCallback(&hdma_spi1_tx); ST7789_Poll(); CHECK(ST7789_GetStatus()==2);
    reset_test(0); now=20; ST7789_Poll(); now=140; ST7789_Poll();
    hdma_spi1_tx.XferCpltCallback(&hdma_spi1_tx); now=150; ST7789_Poll(); CHECK(ST7789_GetStatus()==2);
    reset_test(0); now=20; ST7789_Poll(); start_error=1; now=140; ST7789_Poll(); CHECK(ST7789_GetStatus()==2);
    printf("PASS: %u LCD assertions (DMA, init delays, pixel bounds, redraw, tick wrap, timeouts/errors)\n",(unsigned)assertions);
    return 0;
}
