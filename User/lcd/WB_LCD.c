/**
  * @file    WB_LCD.c 
  * @author  WB R&D Team - openmcu666
  * @version V1.0
  * @date    2016.06.10
  * @brief   LCD Driver
	* @description 红牛7.0寸LCD屏的驱动(驱动IC为SSD1963)
  */
#include "WB_LCD.h"
#include "stm32f10x.h"

#define  Font_Size 16                 		                       // 默认字体为16*8 
#define  XY        1											                       //文字支持横竖屏切换,竖屏:0;横屏:1

                                               
/*********************************************************
** Function name:       void Delay_10ms (u32 nCount)
** Descriptions:        LCD延时函数 10MS
** input parameters:    nCount
** output parameters:   无
** Returned value:      无
*********************************************************/
void Delay_10ms (u32 nCount)
{
	volatile int i;	 	
	for (i=0;i<nCount*800;i++);
}

/*********************************************************************************
** Function name:     void LCD_WriteReg(uint16_t LCD_Reg ,uint16_t LCD_RegValue)
** Descriptions:      配置寄存器
** input parameters:  寄存器地址:LCD_Reg   数据:RegValue
** output parameters: 无
** Returned value:    无
**********************************************************************************/
void LCD_WriteReg(uint16_t LCD_Reg ,uint16_t LCD_RegValue)
{
	LCD_WR_REG(LCD_Reg);                    //写寄存器
  LCD_WE_RAM(LCD_RegValue);	            //写数据
			    
}

/*********************************************************
** Function name:       void LCD_WriteRAM_Prepare(void)
** Descriptions:        开始写GRAM
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************/
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(0x2C);
}

/*********************************************************
** Function name:       uint16_t Read_LCDReg(uint16_t LCD_Reg)
** Descriptions:        读寄存器
** input parameters:    LCD_Reg:寄存器
** output parameters:   无
** Returned value:      返回寄存器值
*********************************************************/
uint16_t Read_LCDReg(uint16_t LCD_Reg)
{
	uint16_t temp;

	LCD_REG= LCD_Reg;
 
	temp= LCD_RAM; 
  
  return temp;		
}

/*********************************************************
** Function name:       u16 LCD_ReadRAM(void)
** Descriptions:        读RAM
** input parameters:    None
** output parameters:   None
** Returned value:      返回寄存器值
*********************************************************/
u16 LCD_ReadRAM(void)
{
  u16 dummy;
 
  LCD_REG = 0x002e; // Select GRAM Reg 
 
  dummy = LCD_RAM;  // Read 16-bit Reg 
  return dummy;
}
/**********************************************************************************
** Function name:       void LCD_Configuration(void)
** Descriptions:        LCD相关引脚配置
** input parameters:  	无
** output parameters:   无
** Returned value:      无
************************************************************************************/
void LCD_IOConfig(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure;


  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE |
                         RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG |
                         RCC_APB2Periph_AFIO, ENABLE);

  /* Set PD.00(D2), PD.01(D3), PD.04(NOE), PD.05(NWE), PD.08(D13), PD.09(D14),
     PD.10(D15), PD.14(D0), PD.15(D1) as alternate 
     function push pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
                                GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 | 
                                GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Set PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10),
     PE.14(D11), PE.15(D12) as alternate function push pull */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | 
                                GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                GPIO_Pin_15;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  GPIO_WriteBit(GPIOE, GPIO_Pin_6, Bit_SET);
  /* Set PF.00(A0 (RS)) as alternate function push pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOF, &GPIO_InitStructure);

  /* Set PG.12(NE4 (LCD/CS)) as alternate function push pull - CE3(LCD /CS) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
}

/**********************************************************************************
** Function name:       void FSMC_LCDInit(void)
** Descriptions:        FSMC时序配置
** input parameters:  	无
** output parameters:   无
** Returned value:      无
************************************************************************************/
void FSMC_LCDInit(void)
{
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;	   	//定义一个NOR初始化结构体
	FSMC_NORSRAMTimingInitTypeDef  w,r;						          //定义一个NOR时序初始化结构体

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);						//使能FSMC总线时钟

  w.FSMC_AddressSetupTime = 0x00;							                  //地址建立时间：0x1
  w.FSMC_AddressHoldTime = 0x00;							                  //地址保持时间：0x00
  w.FSMC_DataSetupTime = 0x01;							                    //数据建立时间：0x2
  w.FSMC_BusTurnAroundDuration = 0x00;
  w.FSMC_CLKDivision = 0x00;
  w.FSMC_DataLatency = 0x00;
  w.FSMC_AccessMode = FSMC_AccessMode_B;

	r.FSMC_AddressSetupTime = 0x00;							                   //地址建立时间：0x1
  r.FSMC_AddressHoldTime = 0x00;							                   //地址保持时间：0x00
  r.FSMC_DataSetupTime = 0x07;							                     //数据建立时间：0x2
  r.FSMC_BusTurnAroundDuration = 0x00;
  r.FSMC_CLKDivision = 0x00;
  r.FSMC_DataLatency = 0x0f;
  r.FSMC_AccessMode = FSMC_AccessMode_B;

  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;	   	//扩展NOR BANK 的第1个子BANK
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;//不使用总线复用
																	                                            
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_NOR;//扩展类型为NOR FLASH
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;//扩展总线宽度为16位
																	                                          
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
																	
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
																	
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &r;	  	
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &w;	  
 
  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 							//根据指定参数初始化结构体
  										
  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE); 							//使能FSMC_Bank1_NORSRAM4 内存

}
/**********************************************************************************
** Function name:       void LCD_Init(void)
** Descriptions:        LCD初始化
** input parameters:  	无
** output parameters:   无
** Returned value:      无
************************************************************************************/
void LCD_Init(void)
{
	LCD_IOConfig();
  FSMC_LCDInit();

  LCD_WR_REG(0x00E2);	
	LCD_WE_RAM(0x0023);
	// Set PLL with OSC = 10MHz (hardware)
  // Multiplier N = 35, VCO (>250MHz)= OSC*(N+1), VCO = 360MHz	   
	LCD_WE_RAM(0x0002);
	// Divider M = 2, PLL = 360/(M+1) = 120MHz
	LCD_WE_RAM(0x0004);
	// Validate M and N values

	LCD_WR_REG(0x00E0);                             // PLL enable
	LCD_WE_RAM(0x0001);
	Delay_10ms(5);
	
	LCD_WR_REG(0x00E0);
	LCD_WE_RAM(0x0003);
	Delay_10ms(5);
	
	LCD_WR_REG(0x0001);                             // software reset
	Delay_10ms(5);
	LCD_WR_REG(0x00E6);
	LCD_WE_RAM(0x0004);
	LCD_WE_RAM(0x0093);
	LCD_WE_RAM(0x00e0);

	LCD_WR_REG(0x00B0);	                             //LCD SPECIFICATION
	LCD_WE_RAM(0x0000);
	LCD_WE_RAM(0x0000);
	LCD_WE_RAM((HDP>>8)&0x00FF);                    //Set HDP
	LCD_WE_RAM(HDP&0X00FF);
  LCD_WE_RAM((VDP>>8)&0x00FF);                    //Set VDP
	LCD_WE_RAM(VDP&0X00FF);
  LCD_WE_RAM(0x0000);

	LCD_WR_REG(0x00B4);	                             //HSYNC
	LCD_WE_RAM((HT>>8)&0x00FF);                     //Set HT
	LCD_WE_RAM(HT&0x00FF);
	LCD_WE_RAM((HPS>>8)&0x00FF);                     //Set HPS
	LCD_WE_RAM(HPS&0x00FF);
	LCD_WE_RAM(HPW);			                            //Set HPW
	LCD_WE_RAM((LPS>>8)&0x00FF);                      //Set HPS
	LCD_WE_RAM(LPS&0x00FF);
	LCD_WE_RAM(0x0000);

	LCD_WR_REG(0x00B6);	                              //VSYNC
	LCD_WE_RAM((VT>>8)&0x00FF);                       //Set VT
	LCD_WE_RAM(VT&0x00FF);
	LCD_WE_RAM((VPS>>8)&0x00FF);                       //Set VPS
	LCD_WE_RAM(VPS&0x00FF);
	LCD_WE_RAM(VPW);			                            //Set VPW
	LCD_WE_RAM((FPS>>8)&0x00FF);                      //Set FPS
	LCD_WE_RAM(FPS&0X00FF);

	LCD_WR_REG(0x00BA);
	LCD_WE_RAM(0x0005);                               //GPIO[3:0] out 1

	LCD_WR_REG(0x00B8);
	LCD_WE_RAM(0x0007);                               //GPIO3=input, GPIO[2:0]=output
	LCD_WE_RAM(0x0001);                               //GPIO0 normal

	LCD_WR_REG(0x0036);                                //rotation
	LCD_WE_RAM(0x0008);

	LCD_WR_REG(0x00F0);                               //pixel data interface
	LCD_WE_RAM(0x0003);


	Delay_10ms(5);
	LCD_WR_REG(0x0029);                                //display on

	LCD_WR_REG(0x00BE);                                //set PWM for B/L
	LCD_WE_RAM(0x0006);
	LCD_WE_RAM(0x0080);
	
	LCD_WE_RAM(0x0001);
	LCD_WE_RAM(0x00f0);
	LCD_WE_RAM(0x0000);
	LCD_WE_RAM(0x0000);

	LCD_WR_REG(0x00d0);                               //设置动态背光控制配置 
	LCD_WE_RAM(0x000d);

	LCD_Clear(BLACK);

	Delay_10ms(5);	                                   //延时50ms
	LCD_Clear(BLACK);
}

/************************************************************************
** Function name:       void LCD_SetCursor(u16 Xpos, u16 Ypos)
** Descriptions:        设置光标位置
** input parameters:    x轴坐标：uint16_t Xpos ;  y轴坐标：uint16_t Ypos
** output parameters:   无
** Returned value:      无
**************************************************************************/
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{		
	LCD_WR_REG(0x002A);	
	LCD_WE_RAM(Xpos>>8);	    
	LCD_WE_RAM(Xpos&0x00ff);
	LCD_WE_RAM(HDP>>8);	    
	LCD_WE_RAM(HDP&0x00ff);
	
  LCD_WR_REG(0x002b);	
	LCD_WE_RAM(Ypos>>8);	    
	LCD_WE_RAM(Ypos&0x00ff);
	LCD_WE_RAM(VDP>>8);	    
	LCD_WE_RAM(VDP&0x00ff);
}

/****************************************************************************
* 名    称：u16 LCD_GetPoint(u16 x,u16 y)
* 功    能：获取指定座标的颜色值
* 入口参数：x      行座标
*           y      列座标
* 出口参数：当前座标颜色值
* 说    明：
* 调用方法：CurrentColor = LCD_GetPoint(10,10);
****************************************************************************/
u16 LCD_GetPoint(u16 x,u16 y)
{
	u16 temp;
	
	LCD_SetCursor(x,y);
	LCD_WR_REG(0x2C);
	LCD_WR_REG(0x2E);

	temp = LCD_ReadRAM();
	return temp;	
}

/***********************************************************************
** Function name:       void LCD_DrawPoint(uint16_t xsta, uint16_t ysta)
** Descriptions:        打点
** input parameters:    xsta X起始坐标 0~480;ysta	Y起始坐标 0~272
** output parameters:   无
** Returned value:      无
***********************************************************************/
void LCD_DrawPoint(uint16_t xsta, uint16_t ysta)
{
	LCD_SetCursor(xsta,ysta);           //设置光标位置  
	LCD_WR_REG(0x2C);                   //开始写入GRAM 
	LCD_WE_RAM(POINT_COLOR); 
}

/*******************************************************************************************************
** Function name:       void LCD_WindowMax (unsigned int x,unsigned int y,unsigned int x_end,unsigned int y_end) 
** Descriptions:        设置窗口最大化480*272
** input parameters:    xsta X起始坐标 0~480;ysta	Y起始坐标 0~272
** output parameters:   无
** Returned value:      无
******************************************************************************************************/
void LCD_WindowMax (unsigned int x,unsigned int y,unsigned int x_end,unsigned int y_end) 
{
	LCD_WR_REG(0x002A);
	LCD_WE_RAM(x>>8);
	LCD_WE_RAM(x&0x00FF);
	LCD_WE_RAM(x_end>>8);
	LCD_WE_RAM(x_end&0x00FF);
		
	LCD_WR_REG(0x002B);	
	LCD_WE_RAM(y>>8);	
	LCD_WE_RAM(y&0x00FF);
	LCD_WE_RAM(y_end>>8);	
	LCD_WE_RAM(y_end&0x00FF);			
 
}

/***********************************************************
** Function name:       void LCD_Clear(uint16_t Color)
** Descriptions:        刷屏函数
** input parameters:    颜色值
** output parameters:   无
** Returned value:      无
************************************************************/
void LCD_Clear(uint16_t Color)
{
	uint16_t index=0; 
  u16 length=LCD_H;	
	LCD_WindowMax(0,0,HDP,VDP);          //设置显示窗口
	LCD_WriteRAM_Prepare();              //开始写入GRAM	  
	while(length--)
	{
		for(index=0;index<LCD_W;index++)
	  {
		  LCD_WE_RAM(Color);	
	  }
	}
}

/*************************************************************************************************************
** Function name:       void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t colour)
** Descriptions:        在指定矩形区域填充指定颜色，区域大小(xend-xsta)*(yend-ysta)
** input parameters:    xsta 起始X坐标
		                    ysta 起始Y坐标
		                    xend 结束X坐标
		                    yend 结束Y坐标
		                    color 待填充颜色
** output parameters:   无
** Returned value:      无
**************************************************************************************************************/
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t colour)
{                    	
	u32 n;
	LCD_WindowMax (xsta, ysta, xend, yend);       //设置窗口	
	LCD_WriteRAM_Prepare();                       //开始写入GRAM*6 	   	   
	n=(u32)(yend-ysta+1)*(xend-xsta+1);     
  while(n--){LCD_WE_RAM(colour);}               //显示所填充的颜色
	 
	LCD_WindowMax (0, 0, LCD_W, LCD_H);      	    //恢复窗口
}  


/****************************************************************************************************
** Function name:       void LCD_DrawLine(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend)
** Descriptions:        画线
** input parameters:    xsta X起始坐标
	   	                  ysta Y起始坐标
		                    xend X终点坐标
		                    yend Y终点坐标	
** output parameters:   无
** Returned value:      无
****************************************************************************************************/
void LCD_DrawLine(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend)
{
  u16 x, y, t;
	if((xsta==xend)&&(ysta==yend))LCD_DrawPoint(xsta, ysta);
	else if(abs(yend-ysta)>abs(xend-xsta))                    //斜率大于1 
	{
		if(ysta>yend) 
		{
			t=ysta;
			ysta=yend;
			yend=t; 
			t=xsta;
			xsta=xend;
			xend=t; 
		}
		for(y=ysta;y<yend;y++)                                   //以y轴为基准 
		{
			x=(u32)(y-ysta)*(xend-xsta)/(yend-ysta)+xsta;
			LCD_DrawPoint(x, y);  
		}
	}
	else                                                       //斜率小于等于1 
	{
		if(xsta>xend)
		{
			t=ysta;
			ysta=yend;
			yend=t;
			t=xsta;
			xsta=xend;
			xend=t;
		}   
		for(x=xsta;x<=xend;x++)                                   //以x轴为基准
		{
			y =(u32)(x-xsta)*(yend-ysta)/(xend-xsta)+ysta;
			LCD_DrawPoint(x,y); 
		}
	} 
} 

/****************************************************************************************************
** Function name:       void LCD_DrawHVLine(uint16_t xsta, uint16_t ysta, uint16_t Length,uint16_t Color,uint8_t Direction)
** Descriptions:        绘制水平或垂直的线条
** input parameters:    xsta X起始坐标
	   	                  ysta Y起始坐标
		                    Length 线条长度
		                    Color 线条颜色
                        Direction: Horizontal 水平; Vertical 垂直
** output parameters:   无
** Returned value:      无
****************************************************************************************************/
void LCD_DrawHVLine(uint16_t xsta, uint16_t ysta, uint16_t Length,uint16_t Color,uint8_t Direction)
{
  uint32_t i = 0;
  
  LCD_SetCursor(xsta, ysta);

  if(Direction == Horizontal)
  {
    LCD_WriteRAM_Prepare();                                //Prepare to write GRAM 
    for(i = 0; i < Length; i++)
    {
      LCD_WE_RAM(Color);
    }
  }
  else
  {
    for(i = 0; i < Length; i++)
    {
			
      LCD_WriteRAM_Prepare();                               //Prepare to write GRAM 
      LCD_WE_RAM(Color);
      xsta++;
      LCD_SetCursor(xsta, ysta);
    }
  }
}

/*****************************************************************************
** Function name:       void LCD_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r)
** Descriptions:        画圆
** input parameters:    x0 中心点横坐标
	                      y0 中心点纵坐标
		                    r  半径
** output parameters:   无
** Returned value:      无
******************************************************************************/
void LCD_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);                                        //判断下个点位置的标志
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a);                         //3           
		LCD_DrawPoint(x0+b,y0-a);                         //0           
		LCD_DrawPoint(x0-a,y0+b);                         //1       
		LCD_DrawPoint(x0-b,y0-a);                         //7           
		LCD_DrawPoint(x0-a,y0-b);                         //2             
		LCD_DrawPoint(x0+b,y0+a);                         //4               
		LCD_DrawPoint(x0+a,y0-b);                         //5
		LCD_DrawPoint(x0+a,y0+b);                         //6 
		LCD_DrawPoint(x0-b,y0+a);             
		a++;

		/*使用Bresenham算法画圆*/     
		if(di<0)di +=4*a+6;	  
		else
		{
			di+=10+4*(a-b);   
			b--;
		} 
		LCD_DrawPoint(x0+a,y0+b);
	}
} 

/*****************************************************************************
** Function name:       void Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r)
** Descriptions:        画矩形
** input parameters:    xsta X起始坐标
	                      ysta Y起始坐标
		                    xend X结束坐标
		                    yend Y结束坐标
** output parameters:   无
** Returned value:      无
******************************************************************************/
void LCD_DrawRectangle(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend)
{
	LCD_DrawLine(xsta,ysta,xend,ysta);
	LCD_DrawLine(xsta,ysta,xsta,yend);
	LCD_DrawLine(xsta,yend,xend,yend);
	LCD_DrawLine(xend,ysta,xend,yend);
} 

/***************************************************************************************************
** Function name:       void LCD_ShowChar(u16 x, u16 y, u8 num, u8 size, u16 PenColor, u16 BackColor)
** Descriptions:        显示一个字符
** input parameters:    x，y      起始坐标（x:0~480 y:0~272）
		                    num       字符ASCII码值
		                    size      字符大小，使用默认8*16
		                    PenColor  字体颜色
		                    BackColor 字体背景颜色
** output parameters:   无
** Returned value:      无
****************************************************************************************************/
void LCD_ShowChar(u16 x, u16 y, u8 num, u8 size, u16 PenColor, u16 BackColor)
{       
  u8 temp;
  u8 pos,t;
  if(x>MAX_CHAR_POSX||y>MAX_CHAR_POSY)return;		 
  
	if(XY)                                             //横屏显示
 	 {	
 		 LCD_WindowMax(x,y,x+size/2-1,y+size-1);	        //设置窗口
 	 }
   else                                                  //竖屏显示
 	 {
		  LCD_WR_REG(0x0036); //rotation
	    LCD_WE_RAM(0x00AB);//AC
			LCD_WindowMax(x,y-size/2,x+size-1,y-1);	           //设置窗口		
 	 }  
	
	LCD_WriteRAM_Prepare();                                //开始写入GRAM   
	num=num-' ';                                            //得到偏移后的值 
	for(pos=0;pos<size;pos++)
	{
		if(size==12)temp=asc2_1206[num][pos];                  //调用1206字体		
		else temp=asc2_1608[num][pos];		                     //调用1608字体
			
		for(t=0;t<size/2;t++)
	  {                 
	    if(temp&0x01)			                                   //从低位开始
			 {
				 LCD_WE_RAM(PenColor);                             //画字体颜色 一个点
			 }
			 else 
			 {
				 LCD_WE_RAM(BackColor);	                           //画背景颜色 一个点      
			 }
			temp>>=1; 
	  }
	}			
	LCD_WindowMax(0x0000,0x0000,LCD_W,LCD_H);	                //恢复窗体大小
	
}

/**********************************************************************************
** Function name:       unsigned int mypow(unsigned int m,unsigned int n)
** Descriptions:        m的n次方 用于取出数值每一位
** input parameters:  	m进制：unsigned int m
**											n次方：unsigned int n						
** output parameters:   m^n
** Returned value:      无
************************************************************************************/
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}			 
 
/*********************************************************************************************
** Function name:       void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len, u16 PenColor, u16 BackColor)
** Descriptions:        显示数字(0~4294967295);
** input parameters:  	x起始：     u16 x
  											y起始：     u16 y
  											数值:       num
    										有效长度：  len		
                        字体颜色:   PenColor
                        背景颜色：  BackColor
** output parameters:   无
** Returned value:      无
************************************************************************************************/
void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len, u16 PenColor, u16 BackColor)
{    
	u8 size = 16;                                          // 这里使用默认的16*8       	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+(size/2)*t,y,' ',size, PenColor, BackColor);
				LCD_ShowChar(x+(size/2)*t,y,'0',size, PenColor, BackColor);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size, PenColor, BackColor); 
	}
} 
  

/*********************************************************************************************
** Function name:       void LCD_ShowCharString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t PenColor, uint16_t BackColor)
** Descriptions:        显示字符(此函数不能单独调用)
** input parameters:  	x，y      起始坐标
	                      p         指向字符串起始地址
		                    PenColor  字符颜色
		                    BackColor 背景颜色
** output parameters:   无
** Returned value:      无
************************************************************************************************/
void LCD_ShowCharString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t PenColor, uint16_t BackColor)
{   
	uint8_t size = 16;                                    //字符大小默认16*8
	 
    if(x>MAX_CHAR_POSX){x=0;y+=size;}			              //超出X轴字体最小单位，换行
    if(y>MAX_CHAR_POSY){y=x=0;LCD_Clear(WHITE);}	      //超出Y轴字体最小单位，回到原点，并且清屏
    LCD_ShowChar(x, y, *p, size, PenColor, BackColor);	//0表示非叠加方式
 
}


/**********************************************************************************
** Function name:       u16 findHzIndex(u8 *hz) 
** Descriptions:        索引汉字存储的内存地址
** input parameters:  	hz				
** output parameters:   无
** Returned value:      无
************************************************************************************/
u16 findHzIndex(u8 *hz)                               // 在自定义汉字库在查找所要显示 
                                                      // 的汉字的位置 
{
	u16 i=0;
	FNT_GB16 *ptGb16 = (FNT_GB16 *)GBHZ_16;		          //ptGb16指向GBHZ_16
	while(ptGb16[i].Index[0] > 0x80)
	{
	  if ((*hz == ptGb16[i].Index[0]) && (*(hz+1) == ptGb16[i].Index[1])) //汉字用两位来表示地址码
		{
	    return i;
	  }
	  i++;
	  if(i > (sizeof((FNT_GB16 *)GBHZ_16) / sizeof(FNT_GB16) - 1))  //搜索下标约束 
	  {
		  break;
	  }
	}
	return 0;
}

/************************************************************************************************
** Function name:       void WriteOneHz(u16 x0, u16 y0, u8 *pucMsk, u16 PenColor, u16 BackColor)
** Descriptions:        显示一个汉字(此函数不能单独作为汉字字符显示)
** input parameters:  	x0,y0      起始坐标
		                    *pucMsk    指向字符串地址
		                    PenColor	 字符颜色
		                    BackColor  背景颜色			
** output parameters:   无
** Returned value:      无
************************************************************************************************/
void WriteOneHz(u16 x0, u16 y0, u8 *pucMsk, u16 PenColor, u16 BackColor)
{
   u16 i,j;
   u16 mod[16];                                      //当前字模 16*16 
   u16 *pusMsk;                                      //当前字库地址  
   u16 y;

	 u16 size = 16;                                      //汉字默认大小16*16

    pusMsk = (u16 *)pucMsk;

    for(i=0; i<16; i++)                                //保存当前汉字点阵式字模 
    {
      mod[i] = *pusMsk;                              //取得当前字模，半字对齐访问   
      mod[i] = ((mod[i] & 0xff00) >> 8) | ((mod[i] & 0x00ff) << 8);//字模交换高低字节
		  pusMsk = pusMsk+1;
    }
    y = x0;		  
		if(XY)                                            //横屏显示
 	  {
 		  LCD_WindowMax(x0,y0,x0+size-1,y0+size-1);	       //设置窗口
 		  
 	  }
     else                                                //竖屏显示
 	   {
 		 	LCD_WR_REG(0x0036); //rotation
	    LCD_WE_RAM(0x00AB);  //AC
			LCD_WindowMax(x0,y0-size,x0+size-1,y0-1);	       //设置窗口		
 	   }  
                               
	  LCD_WriteRAM_Prepare();                              //开始写入GRAM  
    for(i=0; i<16; i++)                                   //16行 
    {                                              
      for(j=0; j<16; j++)                                 //16列  
       {
		    if((mod[i] << j) & 0x8000)                     //显示第i行 共16个点 
         { 
			     LCD_WE_RAM(PenColor);
         } 
			   else 
			   {
           LCD_WE_RAM(BackColor);                   //用读方式跳过写空白点的像素
			   } 
       }
        y++;
    }
	LCD_WindowMax(0x0000,0x0000,LCD_W,LCD_H);  	             //恢复窗体大小
}

/**********************************************************************************
** Function name:       void LCD_ShowHzString(u16 x0, u16 y0, u8 *pcStr, u16 PenColor, u16 BackColor)
** Descriptions:        显示汉字字符串(不能单独调用)
** input parameters:  	x0，y0    起始坐标
		                    pcStr     指向字符串地址
		                    PenColor  字体颜色
		                    BackColor 字体背景				
** output parameters:   无
** Returned value:      无
************************************************************************************/
void LCD_ShowHzString(u16 x0, u16 y0, u8 *pcStr, u16 PenColor, u16 BackColor)
{

	u16 usIndex;
	u8 size = 16; 
	FNT_GB16 *ptGb16 = 0;    
  ptGb16 = (FNT_GB16 *)GBHZ_16; 

	if(x0>MAX_HZ_POSX){x0=0;y0+=size;}			            //超出X轴字体最小单位，换行
  // if(y0>MAX_HZ_POSY){y0=x0=0;LCD_Clear(WHITE);}	    //超出Y轴字体最小单位，回到原点，并且清屏

	usIndex = findHzIndex(pcStr);
	WriteOneHz(x0, y0, (u8 *)&(ptGb16[usIndex].Msk[0]),  PenColor, BackColor); //显示一个汉字
}

/**********************************************************************************
** Function name:       void LCD_DisplayStr(unsigned int x, unsigned int y, unsigned char *s,u16 PenColor, u16 BackColor)
** Descriptions:        显示字符串(可以和汉字混合显示,支持横竖屏切换)
** input parameters:  	x，y    起始坐标
		                    *s     指向字符串地址
		                    PenColor  字体颜色
		                    BackColor 字体背景				
** output parameters:   无
** Returned value:      无
************************************************************************************/
void LCD_DisplayStr(unsigned int x, unsigned int y, unsigned char *s,u16 PenColor, u16 BackColor) 
{
	if(XY==0)
	{
		u16 change=0;
	  change=x;
	  x=y;
	  y=change;
	}
  while (*s) 
	{  	
		if(*s>0x80)                                            //显示汉字
		{
			if(XY)
		  {	
				LCD_ShowHzString(x, y, s, PenColor, BackColor);		
			  x += 16;
			}
			else	
			{
				LCD_ShowHzString(x, LCD_H-y, s, PenColor, BackColor);

			  y += 16;
			}
		  s += 2;
		}
  	else                                                 //非汉字
		{		
		 if(XY)                                               //横屏显示
		 {
		   LCD_ShowChar(x, y, *s++, 16, PenColor, BackColor);
			 x+=Font_Size/2;
			 if(x>=MAX_CHAR_POSX)
			 {
				x=0;
				y+=Font_Size;
				if(y>=MAX_CHAR_POSY)y=0;
			 }
		 }
		 else                                                  //竖屏显示
		 {
			 LCD_ShowChar(x, LCD_H-y, *s++, 16, PenColor, BackColor);
			 y+=Font_Size/2;
			 if(y>=MAX_CHAR_POSX)
			 {
				 y=0;
				 x+=Font_Size;
				 if(x>=MAX_CHAR_POSY)y=0;
			 }
		}
  }
  }
}

/**********************************************************************************
** Function name:       u16 LCD_RGBtoBGR(u16 Color)
** Descriptions:        RRRRRGGGGGGBBBBB 改为 BBBBBGGGGGGRRRRR 格式
** input parameters:  	Color      RGB 颜色值		
** output parameters:   BGR 颜色值
** Returned value:      无
************************************************************************************/
u16 LCD_RGBtoBGR(u16 Color)
{						   
  u16  r, g, b, bgr;

  b = (Color>>0)  & 0x1f;	                    //提取B 
  g = (Color>>5)  & 0x3f;	                    //中间六位
  r = (Color>>11) & 0x1f;	                    //提取R  
  
  bgr =  (b<<11) + (g<<5) + (r<<0);

  return( bgr );
}


/**********************************************************************************
** Function name:       void LCD_DrawPicture(u16 StartX,u16 StartY,u16 Xend,u16 Yend,u8 *pic)
** Descriptions:        图片取模格式为水平扫描，16位颜色模式
** input parameters:  	StartX     行起始座标
                        StartY     列起始座标
                        EndX       行结束座标
                        EndY       列结束座标
                        pic        图片头指针	
** output parameters:   无
** Returned value:      无
************************************************************************************/
void LCD_DrawPicture(u16 StartX,u16 StartY,u16 Xend,u16 Yend,u8 *pic)
{
	static	u16 i=0,j=0;
	u16 *bitmap = (u16 *)pic;

	LCD_WindowMax(StartX, StartY, Xend-1, Yend-1);	//设置图片显示窗口大小
	 
	LCD_WriteRAM_Prepare();
	for(j=0; j<Yend-StartY; j++)
	{
		for(i=0; i<Xend-StartX; i++) LCD_WE_RAM(*bitmap++); 	
	}
	
	LCD_WindowMax(0, 0, LCD_W, LCD_H);              //恢复窗口
}

