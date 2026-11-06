#include "st7789.h"
#include "st7789_font.h"
#include "image_rgb565.h"
#include <Arduino.h>
#include <SPI.h>
#include <stdio.h>
#include <string.h>
#define STRIP_ROWS 4U
#define TEXT_ROWS 10U
#define BACKGROUND_COLOR 0xFFFFU
struct InitCommand { uint8_t command, count, data[14]; uint16_t delay_ms; };
static const InitCommand init_commands[] = {
 {0x11,0,{0},120}, {0x36,1,{ST7789_MADCTL},0}, {0x3A,1,{0x05},0},
 {0xB2,5,{0x0C,0x0C,0x00,0x33,0x33},0}, {0xB7,1,{0x35},0},
 {0xBB,1,{0x32},0}, {0xC2,1,{0x01},0}, {0xC3,1,{0x15},0},
 {0xC4,1,{0x20},0}, {0xC6,1,{0x0F},0}, {0xD0,2,{0xA4,0xA1},0},
 {0xE0,14,{0xD0,0x08,0x0E,0x09,0x09,0x05,0x31,0x33,0x48,0x17,0x14,0x15,0x31,0x34},0},
 {0xE1,14,{0xD0,0x08,0x0E,0x09,0x09,0x15,0x31,0x33,0x48,0x17,0x14,0x15,0x31,0x34},0},
 {0x21,0,{0},0}, {0x29,0,{0},20}
};
static SPIClass displaySPI(FSPI);
static SPISettings settings(40000000, MSBFIRST, SPI_MODE0);
static uint8_t pixels[ST7789_WIDTH * STRIP_ROWS * 2U];
static ST7789_Status latest{}, snapshot{};
static bool link_online=false, frame_online=false;
static uint8_t lcd_status=0, text_row=0, drawing=0, init_phase=0, init_index=0;
static uint8_t animation_frame=0;
static uint16_t draw_y=0, draw_end=ST7789_HEIGHT, foreground=0;
static uint32_t wait_until=0, frame_ms=0, animation_epoch=0, last_full_ms=0;
static char text[21];
static char frame_text[TEXT_ROWS][21];
static void cmd(uint8_t c, const uint8_t *data=nullptr, uint8_t n=0) {
 displaySPI.beginTransaction(settings); digitalWrite(ST7789_PIN_CS, LOW);
 digitalWrite(ST7789_PIN_DC, LOW); displaySPI.transfer(c);
 if(n) {digitalWrite(ST7789_PIN_DC,HIGH); for(uint8_t i=0;i<n;++i) displaySPI.transfer(data[i]);}
 digitalWrite(ST7789_PIN_CS,HIGH); displaySPI.endTransaction();
}
static void push_strip(uint16_t y) {
 uint16_t x0=ST7789_X_OFFSET, x1=x0+ST7789_WIDTH-1, y0=y+ST7789_Y_OFFSET, y1=y0+STRIP_ROWS-1;
 uint8_t col[]={uint8_t(x0>>8),uint8_t(x0),uint8_t(x1>>8),uint8_t(x1)};
 uint8_t row[]={uint8_t(y0>>8),uint8_t(y0),uint8_t(y1>>8),uint8_t(y1)};
 cmd(0x2A,col,4); cmd(0x2B,row,4);
 displaySPI.beginTransaction(settings); digitalWrite(ST7789_PIN_CS,LOW);
 digitalWrite(ST7789_PIN_DC,LOW); displaySPI.transfer(0x2C);
 digitalWrite(ST7789_PIN_DC,HIGH);
 displaySPI.writeBytes(pixels,sizeof(pixels));
 digitalWrite(ST7789_PIN_CS,HIGH); displaySPI.endTransaction();
}
void ST7789_Init(void) {
 pinMode(ST7789_PIN_CS,OUTPUT); pinMode(ST7789_PIN_DC,OUTPUT); pinMode(ST7789_PIN_RST,OUTPUT);
 if(ST7789_PIN_BL>=0) { pinMode(ST7789_PIN_BL,OUTPUT); digitalWrite(ST7789_PIN_BL,HIGH); }
 digitalWrite(ST7789_PIN_CS,HIGH); displaySPI.begin(ST7789_PIN_SCK,-1,ST7789_PIN_MOSI,-1);
 lcd_status=0; init_phase=0; init_index=0; draw_y=0; drawing=0; animation_frame=0; latest={};
 digitalWrite(ST7789_PIN_RST,LOW); wait_until=millis()+20U;
}
void ST7789_SetLinkOnline(bool online) { link_online=online; }
void ST7789_SetStatus(const ST7789_Status *status) { if(status) latest=*status; }
uint8_t ST7789_GetStatus(void) { return lcd_status; }
static void format_line(void)
{
    static const char *const names[] = {"CALIBRATING", "READY", "RUNNING", "FAULT"};
    foreground = 0xFFFFU;
    switch (text_row) {
    case 0: snprintf(text, sizeof(text), frame_online ? "BALANCE / ONLINE" : "LINK DISCONNECTED"); foreground = frame_online ? 0x07FFU : 0xF800U; break;
    case 1:
        snprintf(text, sizeof(text), "STATE %s", snapshot.state < 4U ? names[snapshot.state] : "UNKNOWN");
        foreground = snapshot.state == 2U ? 0x07E0U : (snapshot.state == 3U ? 0xF800U : 0xFFE0U); break;
    case 2: snprintf(text, sizeof(text), "FAULT %u", (unsigned)snapshot.fault); break;
    case 3: snprintf(text, sizeof(text), "ANGLE %8.2f", snapshot.angle); break;
    case 4: snprintf(text, sizeof(text), "GYRO  %8.2f", snapshot.gyro); break;
    case 5: snprintf(text, sizeof(text), "ENC A %8.0f", snapshot.encoder_a); break;
    case 6: snprintf(text, sizeof(text), "ENC B %8.0f", snapshot.encoder_b); break;
    case 7: snprintf(text, sizeof(text), "PWM A %5u", (unsigned)snapshot.pwm_a); break;
    case 8: snprintf(text, sizeof(text), "PWM B %5u", (unsigned)snapshot.pwm_b); break;
    default: snprintf(text, sizeof(text), "STBY %u KEY %u", (unsigned)snapshot.stby, (unsigned)snapshot.key); break;
    }
}
static uint16_t background_pixel(uint16_t x, uint16_t y) {
    if (x >= ANIMATION_WIDTH || y < ANIMATION_Y_OFFSET ||
        y >= ANIMATION_Y_OFFSET + ANIMATION_HEIGHT)
        return BACKGROUND_COLOR;
    uint32_t offset=(uint32_t)(y-ANIMATION_Y_OFFSET)*ANIMATION_WIDTH+x;
    uint8_t index=animation_pixels[animation_frame][offset];
    return animation_palette[animation_frame][index];
}
static bool text_pixel(int x, int y, size_t length) {
    if (x<0 || y<0 || x>=(int)ST7789_WIDTH || y>=16) return false;
    size_t character=(size_t)x/12U;
    if (character>=length) return false;
    uint8_t c=(uint8_t)text[character];
    return c>=32U && c<=126U &&
        (st7789_font[c-32U][((unsigned)x%12U)/2U] & (1U<<((unsigned)y/2U)));
}
static void render_strip(uint16_t top)
{
    for (uint16_t y = 0; y < STRIP_ROWS; ++y) {
        uint16_t screen_y=top+y;
        int line=(screen_y>=8U) ? (int)((screen_y-8U)/22U) : -1;
        int ty=(screen_y>=8U) ? (int)((screen_y-8U)%22U) : -1;
        bool text_area=line>=0 && line<(int)TEXT_ROWS && ty<16;
        size_t length=0;
        if (text_area) { memcpy(text,frame_text[line],sizeof(text)); length=strlen(text); }
        for (uint16_t x = 0; x < ST7789_WIDTH; ++x) {
            uint16_t color=background_pixel(x,screen_y);
            if (text_area) {
                bool glyph=text_pixel(x,ty,length);
                bool outline=text_pixel((int)x-1,ty,length) || text_pixel((int)x+1,ty,length)
                    || text_pixel(x,ty-1,length) || text_pixel(x,ty+1,length);
                if (outline) color=0xFFFFU;
                if (glyph) color=0x07E0U;
            }
            uint16_t index = (uint16_t)((y * ST7789_WIDTH + x) * 2U);
            pixels[index] = (uint8_t)(color >> 8); pixels[index + 1U] = (uint8_t)color;
        }
    }
}
void ST7789_Poll(void) {
 const uint32_t now=millis();
 if ((int32_t)(now-wait_until)<0) return;
 if(init_phase==0) { digitalWrite(ST7789_PIN_RST,HIGH); wait_until=now+120U; init_phase=1; return; }
 if(init_phase==1) {
  if(init_index<sizeof(init_commands)/sizeof(init_commands[0])) {
   const InitCommand &c=init_commands[init_index++]; cmd(c.command,c.data,c.count); wait_until=now+c.delay_ms; return;
  }
  init_phase=2; frame_ms=now-ANIMATION_INTERVAL_MS;
  animation_epoch=now; last_full_ms=now-200U; return;
 }
 if(!drawing) {
  if((uint32_t)(now-frame_ms)<ANIMATION_INTERVAL_MS) return;
  snapshot=latest; frame_online=link_online; frame_ms=now; drawing=1;
  animation_frame=(uint8_t)((((uint64_t)(uint32_t)(now-animation_epoch)*ANIMATION_FPS)/1000U)%ANIMATION_FRAME_COUNT);
  if (!lcd_status || (uint32_t)(now-last_full_ms)>=200U) {
   draw_y=0; draw_end=ST7789_HEIGHT; last_full_ms=now;
  } else {
   draw_y=ANIMATION_Y_OFFSET;
   draw_end=(uint16_t)((ANIMATION_Y_OFFSET+ANIMATION_HEIGHT+STRIP_ROWS-1U)/STRIP_ROWS*STRIP_ROWS);
  }
  for (text_row=0; text_row<TEXT_ROWS; ++text_row) {
   format_line(); memcpy(frame_text[text_row],text,sizeof(text));
  }
 }
 render_strip(draw_y); push_strip(draw_y); draw_y+=STRIP_ROWS;
 if(draw_y>=draw_end) {
  drawing=0; lcd_status=1;
 }
}
