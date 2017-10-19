//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*******************************************
//#include "dwgl_dma.h"
#ifndef __DWGL_DMA_H
#define	__DWGL_DMA_H
#include "stm32f10x.h"
//#define
#define ADC1_DR_Address    ((u32)0x40012400+0x4c)
#define SPI1_DR_Address    ((u32)SPI1_BASE+0x0c)

//变量
 extern  u8  *DMA1_Channel1_Pointer;
 extern  u8   DMA1_Channel1_State;
 extern  u8   DMA1_Channel1_Error;
 extern  u32  DMA1_Channel1_Buffer;

 extern  u8  *DMA2_Channel1_Pointer;
 extern  u8   DMA2_Channel1_State;
 extern  u8   DMA2_Channel1_Error;
 extern  u32  DMA2_Channel1_Buffer;

//子程序
extern void DMA1_Channel1_Enable(void);
extern void DMA1_Channel1_Disable(void);
extern void DMA1_Channel1_Init(void);
extern void DMA1_Channel1_Set(u8 baud,u8 work_mode);

extern void DMA1_Channel2_Enable(void);
extern void DMA1_Channel2_Disable(void);
extern void DMA1_Channel2_Init(unsigned char *buffer,unsigned int len);
extern void DMA1_Channel2_Set(u8 baud,u8 work_mode);

extern void DMA2_Channel1_Enable(void);
extern void DMA2_Channel1_Disable(void);
extern void DMA2_Channel1_Init(void);
extern void DMA2_Channel1_Set(u8 baud,u8 work_mode);

extern void DMA_Configuration(void);
extern void SPI1_DMARecive( u8 *buff, u32 len );


#endif
