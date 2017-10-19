//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*******************************************
#ifndef __DWGL_TIM_H
#define	__DWGL_TIM_H
//#include "dwgl_tim.h"
#include "stm32f10x.h"

//变量
 extern  u8  *TIM2_Pointer;
 extern  u8   TIM2_State;
 extern  u8   TIM2_Error;
 extern  u16  TIM2_Buffer;
 extern  u16  TIM2_Counter;

 extern  u8  *TIM3_Pointer;
 extern  u8   TIM3_State;
 extern  u8   TIM3_Error;
 extern  u16  TIM3_Buffer;
 extern  u16  TIM3_Counter;

//子程序
extern void TIM2_Enable(void);
extern void TIM2_Disable(void);
extern void TIM2_Init(void);
extern void TIM2_Set(u8 baud,u8 work_mode);

extern void TIM3_Enable(void);
extern void TIM3_Disable(void);
extern void TIM3_Init(void);
extern void TIM3_Set(u8 baud,u8 work_mode);

extern void TIM_Configuration(void);
#endif
