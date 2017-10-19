// #include "stm32f10x_lib.h"
// #include "ili9320.h"
#include "stm32f10x.h"
//二维码北京给的
const unsigned char gImage9_128_128_1bitC[2048] = { /* 0X00,0X01,0X80,0X00,0X80,0X00, */
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X3F,0XFF,0XFE,0X00,0X00,0X00,0X00,0XE0,0X70,0X38,0XE0,0X70,0X38,0XFF,0XFF,0XF8,
0X3F,0XFF,0XFE,0X00,0X00,0X00,0X00,0XE0,0X70,0X38,0XE0,0X70,0X38,0XFF,0XFF,0XF8,
0X3F,0XFF,0XFE,0X00,0X00,0X00,0X00,0XE0,0X70,0X38,0XE0,0X70,0X38,0XFF,0XFF,0XF8,
0X38,0X00,0X0E,0X07,0X03,0X81,0XC7,0X1C,0X01,0XF8,0X03,0X8E,0X38,0XE0,0X00,0X38,
0X38,0X00,0X0E,0X07,0X03,0X81,0XC7,0X1C,0X01,0XF8,0X03,0X8E,0X38,0XE0,0X00,0X38,
0X38,0X00,0X0E,0X07,0X03,0X81,0XC7,0X1C,0X01,0XF8,0X03,0X8E,0X38,0XE0,0X00,0X38,
0X38,0XFF,0X8E,0X00,0X00,0X01,0XC7,0X1F,0XFF,0XF8,0XFC,0X0E,0X00,0XE3,0XFE,0X38,
0X38,0XFF,0X8E,0X00,0X00,0X01,0XC7,0X1F,0XFF,0XF8,0XFC,0X0E,0X00,0XE3,0XFE,0X38,
0X38,0XFF,0X8E,0X00,0X00,0X01,0XC7,0X1F,0XFF,0XF8,0XFC,0X0E,0X00,0XE3,0XFE,0X38,
0X38,0XFF,0X8E,0X07,0X00,0X0F,0XFF,0X03,0XFF,0XFF,0X00,0X7F,0XF8,0XE3,0XFE,0X38,
0X38,0XFF,0X8E,0X07,0X00,0X0F,0XFF,0X03,0XFF,0XFF,0X00,0X7F,0XF8,0XE3,0XFE,0X38,
0X38,0XFF,0X8E,0X07,0X00,0X0F,0XFF,0X03,0XFF,0XFF,0X00,0X7F,0XF8,0XE3,0XFE,0X38,
0X38,0XFF,0X8E,0X00,0X00,0X00,0X07,0X03,0XF1,0XC7,0X1F,0X8E,0X38,0XE3,0XFE,0X38,
0X38,0XFF,0X8E,0X00,0X00,0X00,0X07,0X03,0XF1,0XC7,0X1F,0X8E,0X38,0XE3,0XFE,0X38,
0X38,0XFF,0X8E,0X00,0X00,0X00,0X07,0X03,0XF1,0XC7,0X1F,0X8E,0X38,0XE3,0XFE,0X38,
0X38,0X00,0X0E,0X3F,0XFF,0XF0,0X3F,0XFF,0X8E,0X3F,0XFC,0X7F,0XF8,0XE0,0X00,0X38,
0X38,0X00,0X0E,0X3F,0XFF,0XF0,0X3F,0XFF,0X8E,0X3F,0XFC,0X7F,0XF8,0XE0,0X00,0X38,
0X38,0X00,0X0E,0X3F,0XFF,0XF0,0X3F,0XFF,0X8E,0X3F,0XFC,0X7F,0XF8,0XE0,0X00,0X38,
0X3F,0XFF,0XFE,0X38,0XE3,0X8E,0X38,0XE3,0X8E,0X38,0XE3,0X8E,0X38,0XFF,0XFF,0XF8,
0X3F,0XFF,0XFE,0X38,0XE3,0X8E,0X38,0XE3,0X8E,0X38,0XE3,0X8E,0X38,0XFF,0XFF,0XF8,
0X3F,0XFF,0XFE,0X38,0XE3,0X8E,0X38,0XE3,0X8E,0X38,0XE3,0X8E,0X38,0XFF,0XFF,0XF8,
0X00,0X00,0X00,0X07,0X1F,0X8E,0X07,0XE0,0X71,0XF8,0XFC,0X7E,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X07,0X1F,0X8E,0X07,0XE0,0X71,0XF8,0XFC,0X7E,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X07,0X1F,0X8E,0X07,0XE0,0X71,0XF8,0XFC,0X7E,0X00,0X00,0X00,0X00,
0X38,0X1C,0X7E,0X3F,0XE0,0X0F,0XC0,0X00,0X71,0XFF,0XE3,0X80,0X3F,0X1C,0X00,0X00,
0X38,0X1C,0X7E,0X3F,0XE0,0X0F,0XC0,0X00,0X71,0XFF,0XE3,0X80,0X3F,0X1C,0X00,0X00,
0X38,0X1C,0X7E,0X3F,0XE0,0X0F,0XC0,0X00,0X71,0XFF,0XE3,0X80,0X3F,0X1C,0X00,0X00,
0X38,0XE3,0X81,0XC7,0X1C,0X7F,0XFF,0X03,0XF0,0X38,0X03,0X8E,0X38,0X1C,0X7F,0XC0,
0X38,0XE3,0X81,0XC7,0X1C,0X7F,0XFF,0X03,0XF0,0X38,0X03,0X8E,0X38,0X1C,0X7F,0XC0,
0X38,0XE3,0X81,0XC7,0X1C,0X7F,0XFF,0X03,0XF0,0X38,0X03,0X8E,0X38,0X1C,0X7F,0XC0,
0X38,0XE0,0X7E,0X07,0X1C,0X7F,0XC0,0XFC,0X70,0X38,0X00,0X71,0XC7,0XFF,0XFF,0XF8,
0X38,0XE0,0X7E,0X07,0X1C,0X7F,0XC0,0XFC,0X70,0X38,0X00,0X71,0XC7,0XFF,0XFF,0XF8,
0X38,0XE0,0X7E,0X07,0X1C,0X7F,0XC0,0XFC,0X70,0X38,0X00,0X71,0XC7,0XFF,0XFF,0XF8,
0X3F,0XE3,0X81,0XFF,0X1C,0X70,0X00,0X03,0XF0,0X07,0XFC,0X0E,0X07,0XE0,0X0F,0XF8,
0X3F,0XE3,0X81,0XFF,0X1C,0X70,0X00,0X03,0XF0,0X07,0XFC,0X0E,0X07,0XE0,0X0F,0XF8,
0X3F,0XE3,0X81,0XFF,0X1C,0X70,0X00,0X03,0XF0,0X07,0XFC,0X0E,0X07,0XE0,0X0F,0XF8,
0X00,0X00,0X7E,0X3F,0X1F,0X80,0X07,0XE3,0XFF,0XC7,0X1F,0XF0,0X07,0X00,0X71,0XC0,
0X00,0X00,0X7E,0X3F,0X1F,0X80,0X07,0XE3,0XFF,0XC7,0X1F,0XF0,0X07,0X00,0X71,0XC0,
0X00,0X00,0X7E,0X3F,0X1F,0X80,0X07,0XE3,0XFF,0XC7,0X1F,0XF0,0X07,0X00,0X71,0XC0,
0X3F,0XE3,0XF1,0XFF,0XFC,0X00,0X07,0XE3,0X81,0XC0,0XFC,0X0E,0X07,0XFC,0X7E,0X00,
0X3F,0XE3,0XF1,0XFF,0XFC,0X00,0X07,0XE3,0X81,0XC0,0XFC,0X0E,0X07,0XFC,0X7E,0X00,
0X3F,0XE3,0XF1,0XFF,0XFC,0X00,0X07,0XE3,0X81,0XC0,0XFC,0X0E,0X07,0XFC,0X7E,0X00,
0X00,0X1C,0X7E,0X07,0X1C,0X70,0X07,0XFC,0X71,0XC0,0X1F,0X80,0X38,0X1C,0X7E,0X38,
0X00,0X1C,0X7E,0X07,0X1C,0X70,0X07,0XFC,0X71,0XC0,0X1F,0X80,0X38,0X1C,0X7E,0X38,
0X00,0X1C,0X7E,0X07,0X1C,0X70,0X07,0XFC,0X71,0XC0,0X1F,0X80,0X38,0X1C,0X7E,0X38,
0X38,0XFF,0X81,0XC7,0XE0,0X71,0XFF,0XE3,0X81,0XC7,0XE3,0X81,0XC7,0XE3,0X81,0XC0,
0X38,0XFF,0X81,0XC7,0XE0,0X71,0XFF,0XE3,0X81,0XC7,0XE3,0X81,0XC7,0XE3,0X81,0XC0,
0X38,0XFF,0X81,0XC7,0XE0,0X71,0XFF,0XE3,0X81,0XC7,0XE3,0X81,0XC7,0XE3,0X81,0XC0,
0X3F,0XE0,0X7F,0XC0,0X1C,0X0E,0X38,0XE0,0X0E,0X3F,0XFF,0X81,0XF8,0X1C,0X70,0X00,
0X3F,0XE0,0X7F,0XC0,0X1C,0X0E,0X38,0XE0,0X0E,0X3F,0XFF,0X81,0XF8,0X1C,0X70,0X00,
0X3F,0XE0,0X7F,0XC0,0X1C,0X0E,0X38,0XE0,0X0E,0X3F,0XFF,0X81,0XF8,0X1C,0X70,0X00,
0X38,0XE3,0XF0,0X07,0X1F,0X8F,0XFF,0XFF,0XF0,0X38,0X1C,0X01,0XC7,0XE0,0X7F,0XF8,
0X38,0XE3,0XF0,0X07,0X1F,0X8F,0XFF,0XFF,0XF0,0X38,0X1C,0X01,0XC7,0XE0,0X7F,0XF8,
0X38,0XE3,0XF0,0X07,0X1F,0X8F,0XFF,0XFF,0XF0,0X38,0X1C,0X01,0XC7,0XE0,0X7F,0XF8,
0X38,0X03,0X8F,0XC7,0X00,0X7E,0X07,0X00,0X0F,0XFF,0X00,0X0F,0XFF,0X03,0X8F,0XF8,
0X38,0X03,0X8F,0XC7,0X00,0X7E,0X07,0X00,0X0F,0XFF,0X00,0X0F,0XFF,0X03,0X8F,0XF8,
0X38,0X03,0X8F,0XC7,0X00,0X7E,0X07,0X00,0X0F,0XFF,0X00,0X0F,0XFF,0X03,0X8F,0XF8,
0X3F,0X1F,0X80,0X07,0XFF,0XF1,0XC0,0X00,0X70,0X3F,0X1F,0XFF,0XFF,0X1C,0X01,0XF8,
0X3F,0X1F,0X80,0X07,0XFF,0XF1,0XC0,0X00,0X70,0X3F,0X1F,0XFF,0XFF,0X1C,0X01,0XF8,
0X3F,0X1F,0X80,0X07,0XFF,0XF1,0XC0,0X00,0X70,0X3F,0X1F,0XFF,0XFF,0X1C,0X01,0XF8,
0X07,0XE0,0X7E,0X07,0X00,0X01,0XC7,0X03,0X8E,0X38,0XE0,0X7F,0XFF,0XE3,0XFF,0XC0,
0X07,0XE0,0X7E,0X07,0X00,0X01,0XC7,0X03,0X8E,0X38,0XE0,0X7F,0XFF,0XE3,0XFF,0XC0,
0X07,0XE0,0X7E,0X07,0X00,0X01,0XC7,0X03,0X8E,0X38,0XE0,0X7F,0XFF,0XE3,0XFF,0XC0,
0X38,0XFF,0X80,0X00,0X03,0XF0,0X38,0X00,0X0E,0X38,0X03,0XFE,0X3F,0XFC,0X71,0XF8,
0X38,0XFF,0X80,0X00,0X03,0XF0,0X38,0X00,0X0E,0X38,0X03,0XFE,0X3F,0XFC,0X71,0XF8,
0X38,0XFF,0X80,0X00,0X03,0XF0,0X38,0X00,0X0E,0X38,0X03,0XFE,0X3F,0XFC,0X71,0XF8,
0X00,0X1C,0X0E,0X00,0X1C,0X7F,0XC0,0X1F,0XFE,0X38,0X00,0X71,0XC7,0XFF,0X8F,0XC0,
0X00,0X1C,0X0E,0X00,0X1C,0X7F,0XC0,0X1F,0XFE,0X38,0X00,0X71,0XC7,0XFF,0X8F,0XC0,
0X00,0X1C,0X0E,0X00,0X1C,0X7F,0XC0,0X1F,0XFE,0X38,0X00,0X71,0XC7,0XFF,0X8F,0XC0,
0X00,0X03,0XF1,0XC7,0XE0,0X7E,0X00,0XFF,0XF1,0XC7,0XE0,0X0E,0X3F,0XE0,0X7F,0XC0,
0X00,0X03,0XF1,0XC7,0XE0,0X7E,0X00,0XFF,0XF1,0XC7,0XE0,0X0E,0X3F,0XE0,0X7F,0XC0,
0X00,0X03,0XF1,0XC7,0XE0,0X7E,0X00,0XFF,0XF1,0XC7,0XE0,0X0E,0X3F,0XE0,0X7F,0XC0,
0X00,0XFC,0X7E,0X3F,0X1C,0X70,0X3F,0XE3,0XF0,0X3F,0X03,0XFE,0X07,0X03,0XFE,0X00,
0X00,0XFC,0X7E,0X3F,0X1C,0X70,0X3F,0XE3,0XF0,0X3F,0X03,0XFE,0X07,0X03,0XFE,0X00,
0X00,0XFC,0X7E,0X3F,0X1C,0X70,0X3F,0XE3,0XF0,0X3F,0X03,0XFE,0X07,0X03,0XFE,0X00,
0X38,0XFC,0X01,0XC0,0X1F,0XF0,0X07,0X1F,0X8E,0X38,0X00,0X71,0XFF,0XE3,0XF0,0X38,
0X38,0XFC,0X01,0XC0,0X1F,0XF0,0X07,0X1F,0X8E,0X38,0X00,0X71,0XFF,0XE3,0XF0,0X38,
0X38,0XFC,0X01,0XC0,0X1F,0XF0,0X07,0X1F,0X8E,0X38,0X00,0X71,0XFF,0XE3,0XF0,0X38,
0X07,0X03,0XFF,0XFF,0XFC,0X7F,0XFF,0XE0,0X01,0XC0,0XE0,0X7E,0X38,0XFC,0X00,0X38,
0X07,0X03,0XFF,0XFF,0XFC,0X7F,0XFF,0XE0,0X01,0XC0,0XE0,0X7E,0X38,0XFC,0X00,0X38,
0X07,0X03,0XFF,0XFF,0XFC,0X7F,0XFF,0XE0,0X01,0XC0,0XE0,0X7E,0X38,0XFC,0X00,0X38,
0X3F,0XFC,0X00,0X00,0XFC,0X7F,0XC7,0X03,0XFE,0X07,0XE0,0X70,0X38,0X03,0X80,0X00,
0X3F,0XFC,0X00,0X00,0XFC,0X7F,0XC7,0X03,0XFE,0X07,0XE0,0X70,0X38,0X03,0X80,0X00,
0X3F,0XFC,0X00,0X00,0XFC,0X7F,0XC7,0X03,0XFE,0X07,0XE0,0X70,0X38,0X03,0X80,0X00,
0X00,0X1F,0XFE,0X38,0XFF,0XFF,0XC7,0XE0,0X70,0X38,0X03,0XF0,0X38,0X00,0X00,0X00,
0X00,0X1F,0XFE,0X38,0XFF,0XFF,0XC7,0XE0,0X70,0X38,0X03,0XF0,0X38,0X00,0X00,0X00,
0X00,0X1F,0XFE,0X38,0XFF,0XFF,0XC7,0XE0,0X70,0X38,0X03,0XF0,0X38,0X00,0X00,0X00,
0X38,0X1C,0X00,0X07,0X03,0X81,0XC7,0XFF,0XFE,0X38,0XFC,0X01,0XC0,0X1C,0X0E,0X38,
0X38,0X1C,0X00,0X07,0X03,0X81,0XC7,0XFF,0XFE,0X38,0XFC,0X01,0XC0,0X1C,0X0E,0X38,
0X38,0X1C,0X00,0X07,0X03,0X81,0XC7,0XFF,0XFE,0X38,0XFC,0X01,0XC0,0X1C,0X0E,0X38,
0X07,0X03,0XFF,0XFF,0XE0,0X70,0X3F,0X00,0X0F,0XFF,0X00,0X0F,0XFF,0X1F,0XF0,0X38,
0X07,0X03,0XFF,0XFF,0XE0,0X70,0X3F,0X00,0X0F,0XFF,0X00,0X0F,0XFF,0X1F,0XF0,0X38,
0X07,0X03,0XFF,0XFF,0XE0,0X70,0X3F,0X00,0X0F,0XFF,0X00,0X0F,0XFF,0X1F,0XF0,0X38,
0X3F,0XFC,0X71,0XFF,0X03,0X80,0X38,0X00,0X70,0X3F,0X1F,0XFF,0XF8,0X1F,0XF1,0XF8,
0X3F,0XFC,0X71,0XFF,0X03,0X80,0X38,0X00,0X70,0X3F,0X1F,0XFF,0XF8,0X1F,0XF1,0XF8,
0X3F,0XFC,0X71,0XFF,0X03,0X80,0X38,0X00,0X70,0X3F,0X1F,0XFF,0XF8,0X1F,0XF1,0XF8,
0X07,0X1C,0X7E,0X38,0X1C,0X7F,0XFF,0X00,0X0E,0X00,0XE3,0XFF,0XFF,0XFF,0X8E,0X38,
0X07,0X1C,0X7E,0X38,0X1C,0X7F,0XFF,0X00,0X0E,0X00,0XE3,0XFF,0XFF,0XFF,0X8E,0X38,
0X07,0X1C,0X7E,0X38,0X1C,0X7F,0XFF,0X00,0X0E,0X00,0XE3,0XFF,0XFF,0XFF,0X8E,0X38,
0X00,0X00,0X00,0X38,0XE3,0X81,0XFF,0X1F,0X8F,0XC7,0X1F,0X8E,0X38,0X03,0XF1,0XF8,
0X00,0X00,0X00,0X38,0XE3,0X81,0XFF,0X1F,0X8F,0XC7,0X1F,0X8E,0X38,0X03,0XF1,0XF8,
0X00,0X00,0X00,0X38,0XE3,0X81,0XFF,0X1F,0X8F,0XC7,0X1F,0X8E,0X38,0X03,0XF1,0XF8,
0X3F,0XFF,0XFE,0X00,0XFC,0X71,0XC0,0XE3,0XFE,0X07,0XFC,0X71,0XF8,0XE3,0XFF,0XC0,
0X3F,0XFF,0XFE,0X00,0XFC,0X71,0XC0,0XE3,0XFE,0X07,0XFC,0X71,0XF8,0XE3,0XFF,0XC0,
0X3F,0XFF,0XFE,0X00,0XFC,0X71,0XC0,0XE3,0XFE,0X07,0XFC,0X71,0XF8,0XE3,0XFF,0XC0,
0X38,0X00,0X0E,0X38,0X1C,0X71,0XC0,0XE0,0X00,0X07,0X03,0XF1,0XF8,0X03,0XFE,0X38,
0X38,0X00,0X0E,0X38,0X1C,0X71,0XC0,0XE0,0X00,0X07,0X03,0XF1,0XF8,0X03,0XFE,0X38,
0X38,0X00,0X0E,0X38,0X1C,0X71,0XC0,0XE0,0X00,0X07,0X03,0XF1,0XF8,0X03,0XFE,0X38,
0X38,0XFF,0X8E,0X00,0X1F,0XF0,0X38,0XFC,0X00,0X38,0XFC,0X00,0X3F,0XFF,0X8F,0XC0,
0X38,0XFF,0X8E,0X00,0X1F,0XF0,0X38,0XFC,0X00,0X38,0XFC,0X00,0X3F,0XFF,0X8F,0XC0,
0X38,0XFF,0X8E,0X00,0X1F,0XF0,0X38,0XFC,0X00,0X38,0XFC,0X00,0X3F,0XFF,0X8F,0XC0,
0X38,0XFF,0X8E,0X38,0X03,0XFE,0X00,0XFC,0X7E,0X38,0XE0,0X01,0XC7,0XE3,0XFE,0X00,
0X38,0XFF,0X8E,0X38,0X03,0XFE,0X00,0XFC,0X7E,0X38,0XE0,0X01,0XC7,0XE3,0XFE,0X00,
0X38,0XFF,0X8E,0X38,0X03,0XFE,0X00,0XFC,0X7E,0X38,0XE0,0X01,0XC7,0XE3,0XFE,0X00,
0X38,0XFF,0X8E,0X00,0XE3,0X8F,0XC0,0X03,0XF1,0XC7,0XE0,0X00,0X00,0X03,0X8E,0X38,
0X38,0XFF,0X8E,0X00,0XE3,0X8F,0XC0,0X03,0XF1,0XC7,0XE0,0X00,0X00,0X03,0X8E,0X38,
0X38,0XFF,0X8E,0X00,0XE3,0X8F,0XC0,0X03,0XF1,0XC7,0XE0,0X00,0X00,0X03,0X8E,0X38,
0X38,0X00,0X0E,0X07,0XFF,0XF0,0X3F,0XFC,0X7F,0XFF,0XE3,0XF0,0X3F,0XFC,0X0F,0XC0,
0X38,0X00,0X0E,0X07,0XFF,0XF0,0X3F,0XFC,0X7F,0XFF,0XE3,0XF0,0X3F,0XFC,0X0F,0XC0,
0X38,0X00,0X0E,0X07,0XFF,0XF0,0X3F,0XFC,0X7F,0XFF,0XE3,0XF0,0X3F,0XFC,0X0F,0XC0,
0X3F,0XFF,0XFE,0X38,0X1F,0XFE,0X07,0X1F,0XF1,0XF8,0X1C,0X70,0X38,0XFF,0XFF,0XC0,
0X3F,0XFF,0XFE,0X38,0X1F,0XFE,0X07,0X1F,0XF1,0XF8,0X1C,0X70,0X38,0XFF,0XFF,0XC0,
0X3F,0XFF,0XFE,0X38,0X1F,0XFE,0X07,0X1F,0XF1,0XF8,0X1C,0X70,0X38,0XFF,0XFF,0XC0,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00
};
