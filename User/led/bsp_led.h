#ifndef __LED_H
#define	__LED_H

#include "stm32f10x.h"

/** the macro definition to trigger the led on or off 
  * 1 - off
  *0 - on
  */
#define ON  0
#define OFF 1

/* 带参宏，可以像内联函数一样使用 */
#define LED1(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_0);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_0)

#define LED2(a)	if (a)	\
					GPIO_SetBits(GPIOC,GPIO_Pin_4);\
					else		\
					GPIO_ResetBits(GPIOC,GPIO_Pin_4)

#define LED3(a)	if (a)	\
					GPIO_SetBits(GPIOC,GPIO_Pin_3);\
					else		\
					GPIO_ResetBits(GPIOC,GPIO_Pin_3)


/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)			{p->BSRR=i;}			//设置为高电平		
#define digitalLo(p,i)			{p->BRR=i;}				//输出低电平
#define digitalToggle(p,i)		{p->ODR ^=i;}			//输出反转状态


/* 定义控制IO的宏 */
#define LED1_TOGGLE		digitalToggle(GPIOB,GPIO_Pin_0)
#define LED1_OFF		digitalLo(LED1_PORT,LED1_PIN)
#define LED1_ON			digitalHi(LED1_PORT,LED1_PIN)

//BEGIN----------------------------------------
//EN_TPS54336 ON/OFF
#define E0_EN_TPS54336_4_PIN		GPIO_Pin_0 		//GPIO_Mode_Out_PP
#define E1_EN_HV5_PIN				GPIO_Pin_1 		//GPIO_Mode_Out_PP
#define E2_EN_TPS54336_5_PIN		GPIO_Pin_2 		//GPIO_Mode_Out_PP
#define E3_EN_HV6_PIN				GPIO_Pin_3 		//GPIO_Mode_Out_PP
#define E4_EN_TPS54336_6_PIN		GPIO_Pin_4 		//GPIO_Mode_Out_PP
#define E5_EN_HV3_PIN				GPIO_Pin_5 		//GPIO_Mode_Out_PP
#define E6_EN_TPS54336_3_PIN		GPIO_Pin_6 		//GPIO_Mode_Out_PP
#define F0_EN_HV2_PIN				GPIO_Pin_0 		//GPIO_Mode_Out_PP
#define F1_EN_TPS54336_2_PIN		GPIO_Pin_1 		//GPIO_Mode_Out_PP
#define F2_EN_HV1_PIN				GPIO_Pin_2 		//GPIO_Mode_Out_PP
#define F3_EN_TPS54336_1_PIN		GPIO_Pin_3 		//GPIO_Mode_Out_PP
#define B7_EN_HV4_PIN				GPIO_Pin_7		//GPIO_Mode_Out_PP

//MOS ON/OFF
#define SSB0_PIN					E2_EN_TPS54336_5_PIN
#define SSB0_PORT					GPIOE
#define SSB1_PIN					E0_EN_TPS54336_4_PIN
#define SSB1_PORT					GPIOE
#define SSB2_PIN					E4_EN_TPS54336_6_PIN
#define SSB2_PORT					GPIOE
#define SSC0_PIN					E6_EN_TPS54336_3_PIN
#define SSC0_PORT					GPIOE
#define SSC1_PIN					F3_EN_TPS54336_1_PIN
#define SSC1_PORT					GPIOF
#define SSC2_PIN					F1_EN_TPS54336_2_PIN
#define SSC2_PORT					GPIOF
//END----------------------------------------

//BEGIN----------------------------------------
#define F7_INPUT_AD_PIN				GPIO_Pin_7 		//GPIO_Mode_AIN
//输入电流监测
#define B0_AN1_PIN					GPIO_Pin_0 		//GPIO_Mode_AIN
#define A1_AN2_PIN					GPIO_Pin_1 		//GPIO_Mode_AIN
#define C3_AN3_PIN					GPIO_Pin_3 		//GPIO_Mode_AIN
#define F8_AN4_PIN					GPIO_Pin_8 		//GPIO_Mode_AIN
#define F10_AN5_PIN					GPIO_Pin_10		//GPIO_Mode_AIN
#define C1_AN6_PIN					GPIO_Pin_1 		//GPIO_Mode_AIN
//输入电压监测
#define C5_AN7_PIN					GPIO_Pin_5 		//GPIO_Mode_AIN
#define B1_AN8_PIN					GPIO_Pin_1 		//GPIO_Mode_AIN
#define A0_AN9_PIN					GPIO_Pin_0 		//GPIO_Mode_AIN
#define F9_AN10_PIN					GPIO_Pin_9 		//GPIO_Mode_AIN
#define C0_AN11_PIN					GPIO_Pin_0 		//GPIO_Mode_AIN
#define C2_AN12_PIN					GPIO_Pin_2 		//GPIO_Mode_AIN

#define AIN0_PIN					GPIO_Pin_0
#define AIN0_PORT					GPIOA
#define AIN1_PIN					GPIO_Pin_1
#define AIN1_PORT					GPIOA
#define AIN2_PIN					GPIO_Pin_2
#define AIN2_PORT					GPIOA
//END----------------------------------------

//BEGIN----------------------------------------
#define C9_485_TX_EN_PIN			GPIO_Pin_9 		//GPIO_Mode_Out_PP

#define EN_485_PIN					C9_485_TX_EN_PIN
#define EN_485_PORT					GPIOC
//END----------------------------------------

//BEGIN----------------------------------------
#define G2_L_LED_PIN				GPIO_Pin_2 		//GPIO_Mode_Out_PP
#define G3_R_LED_PIN				GPIO_Pin_3 		//GPIO_Mode_Out_PP
#define G4_LED_PIN					GPIO_Pin_4 		//GPIO_Mode_Out_PP
#define G13_KEY_PIN					GPIO_Pin_13		//GPIO_Mode_IPU

#define LED_PIN						G4_LED_PIN
#define LED_PORT					GPIOG
#define LED1_PIN					G3_R_LED_PIN
#define LED1_PORT					GPIOG
#define LED2_PIN					G2_L_LED_PIN
#define LED2_PORT					GPIOG
#define KEY_PIN						G13_KEY_PIN
#define KEY_PORT					GPIOG
//END----------------------------------------



//BEGIN----------------------------------------
//USB,3to1
#define B6_L_SEL1_PIN				GPIO_Pin_6 		//GPIO_Mode_Out_PP
#define B5_L_SEL0_PIN				GPIO_Pin_5 		//GPIO_Mode_Out_PP
#define B3_R_SEL1_PIN				GPIO_Pin_3 		//GPIO_Mode_Out_PP
#define G15_R_SEL0_PIN				GPIO_Pin_15		//GPIO_Mode_Out_PP

#define USB_DB0_PIN					G15_R_SEL0_PIN
#define USB_DB0_PORT				GPIOG
#define USB_DB1_PIN					B3_R_SEL1_PIN
#define USB_DB1_PORT				GPIOB
#define USB_DC0_PIN					B5_L_SEL0_PIN
#define USB_DC0_PORT				GPIOB
#define USB_DC1_PIN					B6_L_SEL1_PIN
#define USB_DC1_PORT				GPIOB
//END----------------------------------------

#define DPB1_PIN			   GPIO_Pin_12   //本地USB口
#define DPB1_PORT			   GPIOA
#define DPC3_PIN			   GPIO_Pin_11
#define DPC3_PORT			   GPIOA

//BEGIN----------------------------------------
#define A3_RJ45_IO_PIN				GPIO_Pin_3 		//GPIO_Mode_Out_PP

#define RJ45_IO1_PIN				A3_RJ45_IO_PIN
#define RJ45_IO1_PORT				GPIOA
//END----------------------------------------

//BEGIN----------------------------------------
#define A7_SPI1_MOSI_PIN			GPIO_Pin_7 		//GPIO_Mode_Out_PP
#define A6_SPI1_MISO_PIN			GPIO_Pin_6		//GPIO_Mode_IN_FLOATING
#define A5_SPI1_CLK_PIN				GPIO_Pin_5 		//GPIO_Mode_Out_PP
#define A4_SPI1_CS_PIN				GPIO_Pin_4 		//GPIO_Mode_Out_PP

#define SPI2_MOSI_PIN				A7_SPI1_MOSI_PIN
#define SPI2_MOSI_PORT				GPIOA
#define SPI2_MISO_PIN				A6_SPI1_MISO_PIN
#define SPI2_MISO_PORT				GPIOA
#define SPI2_CLK_PIN				A5_SPI1_CLK_PIN
#define SPI2_CLK_PORT				GPIOA
#define SPI2_CS_PIN					A4_SPI1_CS_PIN
#define SPI2_CS_PORT				GPIOA
//END----------------------------------------

//BEGIN----------------------------------------
#define C7_LCD_RST_PIN				GPIO_Pin_7 		//GPIO_Mode_Out_PP
#define D12_LCD_CS1_PIN				GPIO_Pin_12		//GPIO_Mode_Out_PP
#define D13_LCD_CS2_PIN				GPIO_Pin_13		//GPIO_Mode_Out_PP

#define LCD_CS1_PIN					D12_LCD_CS1_PIN
#define LCD_CS1_PORT				GPIOD
#define LCD_CS2_PIN					D13_LCD_CS2_PIN
#define LCD_CS2_PORT				GPIOD
#define LCD_RST_PIN					C7_LCD_RST_PIN
#define LCD_RST_PORT				GPIOC
//END----------------------------------------

// #define LCD_TE_PIN			  GPIO_Pin_8  //功能已取消
// #define LCD_TE_PORT			  GPIOC
// #define LCD_RS_PIN			  GPIO_Pin_6
// #define LCD_RS_PORT			  GPIOC
// #define LCD_WR_PIN			  GPIO_Pin_2
// #define LCD_WR_PORT			  GPIOD
// #define LCD_RD_PIN			  GPIO_Pin_12
// #define LCD_RD_PORT			  GPIOC

//BEGIN----------------------------------------
//快充切换
#define B4_L_SW_883_HUB_PIN			GPIO_Pin_4 		//GPIO_Mode_Out_PP
#define G14_R_SW_883_HUB_PIN		GPIO_Pin_14		//GPIO_Mode_Out_PP

#define EN_KC1_PIN					B4_L_SW_883_HUB_PIN
#define EN_KC1_PORT					GPIOB
#define EN_KC0_PIN					G14_R_SW_883_HUB_PIN
#define EN_KC0_PORT					GPIOG
//END----------------------------------------

//BEGIN----------------------------------------
#define B8_HUB1_REST_PIN			GPIO_Pin_8 		//GPIO_Mode_Out_PP
#define B9_HUB2_REST_PIN			GPIO_Pin_9 		//GPIO_Mode_Out_PP

#define HUB0_REST_PIN				B8_HUB1_REST_PIN
#define HUB0_REST_PORT				GPIOB
#define HUB1_REST_PIN				B9_HUB2_REST_PIN
#define HUB1_REST_PORT				GPIOB
//END----------------------------------------

//引脚定义(PAx~PGx)
/*******************************************************/
#define B2_BOOT1_PIN				GPIO_Pin_2		//GPIO_Mode_Out_PP

#define C8_FAN1_PIN					GPIO_Pin_8 		//

#define D0_LCD_PD0_PIN				GPIO_Pin_0 		//
#define D1_LCD_PD1_PIN				GPIO_Pin_1 		//
#define D4_LCD_PD4_PIN				GPIO_Pin_4 		//
#define D5_LCD_PD5_PIN				GPIO_Pin_5 		//
#define D7_LCD_PD7_PIN				GPIO_Pin_7 		//
#define D8_LCD_PD8_PIN				GPIO_Pin_8 		//
#define D9_LCD_PD9_PIN				GPIO_Pin_9 		//
#define D10_LCD_PD10_PIN			GPIO_Pin_10		//
#define D11_LCD_PD11_PIN			GPIO_Pin_11		//

#define D14_LCD_PD14_PIN			GPIO_Pin_14		//
#define D15_LCD_PD15_PIN			GPIO_Pin_15		//

#define E7_LCD_PE7_PIN				GPIO_Pin_7 		//
#define E8_LCD_PE8_PIN				GPIO_Pin_8 		//
#define E9_LCD_PE9_PIN				GPIO_Pin_9 		//
#define E10_LCD_PE10_PIN			GPIO_Pin_10		//
#define E11_LCD_PE11_PIN			GPIO_Pin_11		//
#define E12_LCD_PE12_PIN			GPIO_Pin_12		//
#define E13_LCD_PE13_PIN			GPIO_Pin_13		//
#define E14_LCD_PE14_PIN			GPIO_Pin_14		//
#define E15_LCD_PE15_PIN			GPIO_Pin_15		//

#define F11_EN_FAN_PIN				GPIO_Pin_11		//

#define G5_LCD_LED_SEG_PIN			GPIO_Pin_5 		//
#define G8_FAN2_PIN					GPIO_Pin_8 		//
/*******************************************************/

void LED_GPIO_Config(void);
void GPIO_NegationBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin); //dwgl  取反

#endif /* __LED_H */
