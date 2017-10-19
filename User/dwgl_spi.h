//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*******************************************
#ifndef  __DWGL_SPI_H
#define  __DWGL_SPI_H
#include "stm32f10x.h"

//define 
//#define	SPI1_init_Normal//手动更新方式
#define	SPI1_init_DMA      // DMA方式

#ifndef A7_SPI1_MOSI_PIN
#define A7_SPI1_MOSI_PIN			GPIO_Pin_7 		//GPIO_Mode_Out_PP
#endif

#ifndef A6_SPI1_MISO_PIN
#define A6_SPI1_MISO_PIN			GPIO_Pin_6		//GPIO_Mode_IN_FLOATING
#endif

#ifndef A5_SPI1_CLK_PIN
#define A5_SPI1_CLK_PIN				GPIO_Pin_5 		//GPIO_Mode_Out_PP
#endif

#ifndef A4_SPI1_CS_PIN
#define A4_SPI1_CS_PIN				GPIO_Pin_4 		//GPIO_Mode_Out_PP
#endif


//变量
 extern  u8  *SPI1_Receive_Pointer;
 extern  u16  SPI1_Receive_BufSize;
 extern  u16  SPI1_Receive_Length;
 extern  u8  *SPI1_Trans_Pointer;
 extern  u16  SPI1_Trans_BufSize;
 extern  u16  SPI1_Trans_Length;
 extern  u8   SPI1_TX_State;
 extern  u8   SPI1_RX_State;
 extern  u8   SPI1_Error;

 extern  u8  *SPI2_Receive_Pointer;
 extern  u16  SPI2_Receive_BufSize;
 extern  u16  SPI2_Receive_Length;
 extern  u8  *SPI2_Trans_Pointer;
 extern  u16  SPI2_Trans_BufSize;
 extern  u16  SPI2_Trans_Length;
 extern  u8   SPI2_TX_State;
 extern  u8   SPI2_RX_State;
 extern  u8   SPI2_Error;

//子程序
extern void SPI1_Enable(void);
extern void SPI1_Disable(void);
//extern void SPI1_Init(void);
extern void SPI1_Set(u8 baud,u8 work_mode);
//extern void SPI1_Send_Data(u8 * p,u16 len);
extern void SPI1_Send_Data(u8 * txp,u8 * rxp,u16 len);
extern void SPI1_Send_Data1(u8 * txp,u8 * rxp,u16 len);

extern void SPI2_Enable(void);
extern void SPI2_Disable(void);
extern void SPI2_Init(void);
extern void SPI2_Set(u8 baud,u8 work_mode);
extern void SPI2_Send_Data(u8 * p,u16 len);

extern void GPIOSPI_Init(void);
extern void GPIOSPI_Send_BYTE(u8 data);
extern void GPIOSPI_Send1_BYTE(u8 data);
extern void GPIOSPI_Send_Data(u8 * p,u16 len);
extern void GPIOSPI_Send1_Data(u8 * p,u16 len);

extern u8 GPIOSPI_Read_BYTE(void);
extern void GPIOSPI_Read_Data(u8 * p,u16 len);

extern u8 GPIOSPI_SR_BYTE(u8 data);
extern void GPIOSPI_SR_Data(u8 * tx,u8 * rx,u16 len);

extern void GPIOSPI_ESend_Data(u8 * p,u16 len);
extern void GPIOSPI_FSend_Data(u8 * p,u16 len);
extern void GPIOSPI_EFSend_Data(u8 * p,u16 len);
void FLASH2_GPIOSPI_Read (unsigned int write_addr, unsigned char *buffer, unsigned int len);//FLASH-读出LEN个字节-
void FLASH2_GPIOSPI_Write(unsigned int write_addr, unsigned char *desbuffer,unsigned int len); // reentrant//FLASH-写入LEN个字节
void FLASH2_GPIOSPI_IsBusy(void); // 
void FLASH2_GPIOSPI_SER(unsigned int u32_addr);//按页擦除部分flash  每次擦擦4K
void FLASH2_GPIOSPI_SE(unsigned int u32_addr);//按块擦除部分flash  每次擦擦64K


extern void SPI_Configuration(void);

#endif

