#ifndef __WB_LCD_H
#define __WB_LCD_H

#include "stm32f10x_conf.h"
#include "stdio.h"
#include "stdlib.h"

//#define LCD_RAM    *(__IO uint16_t *) ((uint32_t)0x6C000002)    	//地址寄存器
//#define LCD_REG    *(__IO uint16_t *) ((uint32_t)0x6C000000)	 	  //指令寄存器

//#define LCD_RAM    *(__IO uint16_t *) ((uint32_t)0x6C020000)    	//地址寄存器  //dwgl
//#define LCD_REG    *(__IO uint16_t *) ((uint32_t)0x6C000000)	 	  //指令寄存器

#define LCD_RAM    *(__IO uint16_t *) ((uint32_t)0x60020000)    	//地址寄存器  //dwgl
#define LCD_REG    *(__IO uint16_t *) ((uint32_t)0x60000000)	 	  //指令寄存器


#define LCD_WR_REG(LCD_Reg)  LCD_REG=(LCD_Reg)              //写命令
#define LCD_WE_RAM(RGB_Code) LCD_RAM=(RGB_Code)             //写数据


//#define LCD_W   800
//#define LCD_H   480
#define LCD_W   320
#define LCD_H   240

//#define SET_XY   1    //横1
//#define SET_XY   2	//横2
//#define SET_XY   3	//竖1
#define SET_XY   4		//竖2


//#define SET_XY1   1    //横1
//#define SET_XY1   2	//横2
//#define SET_XY1   3	//竖1
#define SET_XY1   4		//竖2

//#define SET_XY2   1    //横1
//#define SET_XY2   2	//横2
//#define SET_XY2   3	//竖1
#define SET_XY2   4		//竖2


//#define   HDP	  799
//#define   HT	  1000
//#define   HPS	  51
//#define   LPS	  3
//#define   HPW	  8

#define   HDP	  319
#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
#define   HT	  421
#else
#define   HT	  423//天马屏
#endif

//#define   HPS	  68
#define   HPS	  70   //天马屏
//#define   LPS	  33
#define   LPS	  0
#define   HPW	  1

//#define   VDP	  479
//#define   VT	  530
//#define   VPS	  24
//#define   FPS	  23
//#define   VPW	  3

#define   VDP	  239
#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
#define   VT	  268  
#else
#define   VT	  263   //天马屏
#endif

//#define   VPS	  18
#define   VPS	  13   //天马屏
//#define   FPS	  10
#define   FPS	  0
#define   VPW	  1

//////////////////////////////////////////////
//////////////////////////////////////////////
// #define  HDP  319 //Hsync Display period 
// #define  HT   408 //Horizontal total period = (HT + 1) pixels // HSYNC period time 
// #define  HPS  108  //Horizontal Sync Pulse Start Position = (HPS + 1) pixels  //HSYNC back porch //30
// #define  LPS  40  //Horizontal Display Period Start Position = LPS pixels   // HSYNC front porch//20
// #define  HPW  10 //Horizontal Sync Pulse Width = (HPW + 1) pixels //HSYNC width //22

// #define  VDP  239  //Vsync Display period 
// #define  VT   262 //Vertical Total = (VT + 1) lines // SET VSYNC period time 
// #define  VPS  23   //Vertical Sync Pulse Start Position = VPS lines  // VSYNC back porch  //20
// #define  FPS  5  //Vertical Display Period Start Position = FPS lines. // SET VSYNC front porch //40
// #define  VPW  3    //Vertical Sync Pulse Width = (VPW + 1) lines //set VSYNC width //40
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
#define MAX_HZ_POSX    784
#define MAX_HZ_POSY    464

#define MAX_CHAR_POSX  312
#define MAX_CHAR_POSY  234

#define Horizontal     0x00
#define Vertical       0x01

/*Private define-------------------------*/
//#define POINT_COLOR WHITE
#define POINT_COLOR 0x0000

#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
#define BACK_COLOR	BLUE
#else
#define BACK_COLOR	WHITE
#endif

/*--------16位颜色值---------------*/
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  

#define RED           	 0xF800
#define GREEN         	 0x07E0
#define BLUE         	   0x001F  
#define MAGENTA       	 0xF81F
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			     0XBC40 //棕色
#define BRRED 			     0XFC07 //棕红色
#define GRAY  			     0X8430 //灰色
#define LGRAY 			     0XC618 //浅灰色
#define GBLUE			 			 0X07FF
//GUI
#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
//PANEL
#define LIGHTGREEN     	 0X841F //浅绿色
#define LGRAY 			 		 0XC618 //浅灰色(PANNEL)窗体背景色
#define LGRAYBLUE        0XA651 //浅灰蓝色，（中间层色）
#define LBBLUE           0X2B12 //浅棕蓝色，选择条目的反色


/*----------函数声明------------*/
/* 刷屏 */
// void LCD_WR_REG(uint16_t reg);
// void LCD_WE_RAM(uint16_t data);
extern uint16_t Read_LCDReg(uint16_t LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_DrawPoint(uint16_t xsta, uint16_t ysta);

void LCD_Clear(uint16_t Color);
void LCD_Clear_S(uint16_t Color);
void LCD_SetCursor(u16 Xpos, u16 Ypos);
u16  LCD_GetPoint(u16 x,u16 y);
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t colour);
void LCD_DrawLine(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend);
void LCD_DrawHVLine(uint16_t xsta, uint16_t ysta, uint16_t Length,uint16_t Color,uint8_t Direction);
void LCD_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r);
void LCD_DrawRectangle(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend);
void LCD_WindowMax (unsigned int x,unsigned int y,unsigned int x_end,unsigned int y_end);
void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len, u16 PenColor, u16 BackColor);//显示数字
void LCD_DrawPicture(u16 StartX,u16 StartY,u16 Xend,u16 Yend,u8 *pic);     //显示图片 gImage取模 bmp格式

void LCD_IOConfig(void);
void FSMC_LCDInit(void);
void LCD_Init(void);
void LCD_Init1(void);
void LCD_DisplayStr(unsigned int x, unsigned int y, unsigned char *s,u16 PenColor, u16 BackColor);
void LCD_WriteReg(uint16_t LCD_Reg ,uint16_t LCD_RegValue);
#endif
