#include "LED.h"
#include "main.h"
#include "i2c.h"
#include "stdio.h"
#include "font.h"

#define OLED_ADDR (0x3CU << 1)

// 写命令
void OLED_WCmd(uint8_t cmd)
{
    uint8_t dat[2] = {0x00, cmd};
    HAL_I2C_Master_Transmit(&hi2c2, OLED_ADDR, dat, 2, 100);
}

// 写数据
void OLED_WData(uint8_t data)
{
    uint8_t dat[2] = {0x40, data};
    HAL_I2C_Master_Transmit(&hi2c2, OLED_ADDR, dat, 2, 100);
}

// 坐标设置
void OLED_SetPos(uint8_t x, uint8_t y)
{
    OLED_WCmd(0xb0 + y);
    OLED_WCmd(((x & 0xf0) >> 4) | 0x10);
    OLED_WCmd(x & 0x0f);
}

// 清屏
void OLED_Clear(void)
{
    uint8_t y,x;
    for(y=0;y<8;y++)
    {
        OLED_WCmd(0xb0+y);
        OLED_WCmd(0x00);
        OLED_WCmd(0x10);
        for(x=0;x<128;x++) OLED_WData(0x00);
    }
}

// 初始化1.54寸OLED
void OLED_Init(void)
{
    HAL_Delay(100);
    OLED_WCmd(0xAE);
    OLED_WCmd(0xD5);OLED_WCmd(0x80);
    OLED_WCmd(0xA8);OLED_WCmd(0x3F);
    OLED_WCmd(0xD3);OLED_WCmd(0x00);
    OLED_WCmd(0x40);
    OLED_WCmd(0x8D);OLED_WCmd(0x14);
    OLED_WCmd(0x20);OLED_WCmd(0x02);
    OLED_WCmd(0xA1);
    OLED_WCmd(0xC8);
    OLED_WCmd(0xDA);OLED_WCmd(0x12);
    OLED_WCmd(0x81);OLED_WCmd(0xCF);
    OLED_WCmd(0xD9);OLED_WCmd(0xF1);
    OLED_WCmd(0xDB);OLED_WCmd(0x30);
    OLED_WCmd(0xA4);
    OLED_WCmd(0xA6);
    OLED_WCmd(0xAF);
    OLED_Clear();
}

// 显示8*16字符
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr)
{
    uint8_t c=0,i=0;
    c=chr-' ';
    if(c>=sizeof(afont16x8.chars)/16) c=0;
    OLED_SetPos(x,y);
    for(i=0;i<8;i++) OLED_WData(afont16x8.chars[c*16+i]);
    OLED_SetPos(x,y+1);
    for(i=0;i<8;i++) OLED_WData(afont16x8.chars[c*16+i+8]);
}

// 显示字符串
void OLED_ShowStr(uint8_t x,uint8_t y,char *str)
{
    while(*str)
    {
        OLED_ShowChar(x,y,*str);
        x+=8;
        if(x>120) x=0,y+=2;
        str++;
    }
}
