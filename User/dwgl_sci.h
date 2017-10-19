//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*******************************************
#ifndef  __DWGL_SCI_H
#define  __DWGL_SCI_H
#include "stm32f10x.h"

//define
#define  UART1_ISR_Choose     2  //??????
//#define  UART0_ISR_Choose     1  //??????
//#define  UART0_ISR_Choose     0  //??????  ?????
//#define  UART1_ISR_Choose     2  //??????
//#define  UART1_ISR_Choose     1  //??????
#define  UART2_ISR_Choose     0  //??????  ?????

#define  UART_TX_Ready       1  //
#define  UART_RX_Ready       1  //

#define A9_USART1_TX_PIN			GPIO_Pin_9 		//GPIO_Mode_AF_PP
#define A10_USART1_RX_PIN			GPIO_Pin_10		//GPIO_Mode_IN_FLOATING

#define B10_RJ45_TX_PIN				GPIO_Pin_10		//GPIO_Mode_AF_PP
#define B11_RJ45_RX_PIN				GPIO_Pin_11		//GPIO_Mode_IN_FLOATING



//变量
 extern  u8  *UART1_Receive_Pointer;
 extern  u16  UART1_Receive_BufSize;
 extern  u16  UART1_Receive_Length;
 extern  u8  *UART1_Trans_Pointer;
 extern  u16  UART1_Trans_BufSize;
 extern  u16  UART1_Trans_Length;
 extern  u8   UART1_TX_State;
 extern  u8   UART1_RX_State;
 extern  u8   UART1_Error;

 extern  u8  *UART2_Receive_Pointer;
 extern  u16  UART2_Receive_BufSize;
 extern  u16  UART2_Receive_Length;
 extern  u8  *UART2_Trans_Pointer;
 extern  u16  UART2_Trans_BufSize;
 extern  u16  UART2_Trans_Length;
 extern  u8   UART2_TX_State;
 extern  u8   UART2_RX_State;
 extern  u8   UART2_Error;

 extern  u8  *UART3_Receive_Pointer;
 extern  u16  UART3_Receive_BufSize;
 extern  u16  UART3_Receive_Length;
 extern  u8  *UART3_Trans_Pointer;
 extern  u16  UART3_Trans_BufSize;
 extern  u16  UART3_Trans_Length;
 extern  u8   UART3_TX_State;
 extern  u8   UART3_RX_State;
 extern  u8   UART3_Error;

//子程序
extern void UART1_Enable(void);
extern void UART1_Disable(void);
//extern void UART1_Init(void);
extern void UART1_Set(u8 baud,u8 work_mode);
extern void UART1_Send_Data(u8 * p,u16 len);

extern void UART2_Enable(void);
extern void UART2_Disable(void);
extern void UART2_Init(void);
extern void UART2_Set(u8 baud,u8 work_mode);
extern void UART2_Send_Data(u8 * p,u16 len);

extern void UART3_Enable(void);
extern void UART3_Disable(void);
//extern void UART3_Init(void);
extern void UART3_Set(u8 baud,u8 work_mode);
extern void UART3_Send_Data(u8 * p,u16 len);

extern void UART_Configuration(void);

#endif




