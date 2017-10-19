//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*******************************************
#ifndef __DWGL_EXTI_H
#define	__DWGL_EXTI_H
//#include "dwgl_exti.h"
#include "stm32f10x.h"

//变量
 extern  u8  *EXTI1_Pointer;
 extern  u8   EXTI1_State;
 extern  u8   EXTI1_Error;
 extern  u16  EXTI1_Counter;

 extern  u8  *EXTI15_10_Pointer;
 extern  u8   EXTI15_10_State;
 extern  u8   EXTI15_10_Error;
 extern  u16  EXTI15_10_Counter;

//子程序
extern void EXTI1_Enable(void);
extern void EXTI1_Disable(void);
extern void EXTI1_Init(void);
extern void EXTI1_Set(u8 baud,u8 work_mode);

extern void EXTI15_10_Enable(void);
extern void EXTI15_10_Disable(void);
extern void EXTI15_10_Init(void);
extern void EXTI15_10_Set(u8 baud,u8 work_mode);

extern void EXTI_Configuration(void);

#endif
