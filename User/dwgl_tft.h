//dwgl_tft.h

#ifndef __dwgl_tft_H
#define __dwgl_tft_H

//#include "dwgl_tft.h"
#include "stm32f10x.h"


extern u32 g32_temp;
extern u16 g16_temp;
extern unsigned char AnsiChardata[32];
extern unsigned char HZ_XZWM[9];
extern unsigned char HZ_SRFY[9];

//const unsigned char gImage_320LOGO24BIT[2048];
//extern const unsigned char gImage7_128_128_1bitC[2048];
//extern const unsigned char gImage9_128_128_1bitC[2048];
 extern const unsigned char gImage10_16_288_1bitC[640];
//extern const unsigned char gImage1_240_171_16bitC[82088];
//extern const unsigned char gImage1_171_240_16bitC[82088];
//extern const unsigned char gImage1_320_228_16bitC[145928];
//extern const unsigned char gImage4_240_102_16bitC[48960];
//extern const unsigned char gImage5_240_80_16bitC[38400];
//extern const unsigned char gImage11_240_100_16bitC[48066];
extern const unsigned char gImage12_128_128_1bitC[2110];
//extern const unsigned char gImage6_200_24_1bitC[600];

extern void tft_Clear(u16 x,u16 y,u16 x_offset,u16 y_offset,u16 Color,u8 cs);
extern void tft_1bitdeep_array (u16 x,u16 y,u16 x_offset,u16 y_offset,const u8 *array);
// extern void tft_1bitdeep_BMP (u16 x,u16 y,const u8 *array,u16 PenColor, u16 BackColor);
extern void display_flash_BMP (u16 x,u16 y,unsigned int addr,u8 cs);//从FLASH按地址调图片
extern void display_flash_BMPE (u16 x,u16 y,u8 area,u8 num,u8 cs);//从FLASH按协议调图片
extern void display_IC_BMP (u16 x,u16 y,const unsigned char *p,u8 cs);  //从单片机ROM调图片

extern void tft_1bitdeep_BMP (u16 x,u16 y,const u8 *array,u16 PenColor, u16 BackColor,u8 cs);
extern void tft_1bitdeep_EWM (u16 x,u16 y, u8 *array,u16 PenColor, u16 BackColor,u8 cs);
extern void tft_1bitdeep_TXT (u16 x,u16 y, u8 *array,u16 PenColor, u16 BackColor,u8 cs);
extern void tft_16bitdeep_BMP (u16 x,u16 y,const u8 *array);
extern void tft_16bitdeep_array (u16 x,u16 y,u16 x_offset,u16 y_offset,const u8 *array);
extern void tft_ShowHz(u16 x, u16 y, u8 *pcStr, u16 PenColor, u16 BackColor);
extern void tft_ShowHzE(u16 x, u16 y, u8 *pcStr, u16 PenColor, u16 BackColor);
extern void tft_ShowChar(unsigned int x, unsigned int y, unsigned char num,u16 PenColor, u16 BackColor);
extern void tft_ShowCharE(unsigned int x, unsigned int y, unsigned char num,u16 PenColor, u16 BackColor);
// extern void tft_DisplayStr(unsigned int x, unsigned int y, unsigned char *s,u16 PenColor, u16 BackColor);
extern void tft_DisplayStr(unsigned int x, unsigned int y, unsigned char *s,u16 PenColor, u16 BackColor,u8 cs);
extern void tft_cs_enable(u8 cs);
extern void tft_cs_disable(u8 cs);

#endif
