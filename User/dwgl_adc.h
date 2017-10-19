//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*******************************************
//#include "dwgl_adc.h"
#ifndef __DWGL_ADC_H
#define	__DWGL_ADC_H
#include "stm32f10x.h"

//#define	ADC1_init_Normal	//手动更新方式
//#define	ADC1_init_DMA		// DMA方式
//#define	ADC1_init_DMAE		// DMA 自动扫描

//电流 real_size = get_adc_value / 32
#define AN1_ADC1_CH8			ADC_Channel_8
#define AN2_ADC1_CH1			ADC_Channel_1
#define AN3_ADC1_CH13			ADC_Channel_13

#define AN4_ADC3_CH6			ADC_Channel_6
#define AN5_ADC3_CH8			ADC_Channel_8
#define AN6_ADC3_CH11			ADC_Channel_11

//电压 real_size = get_adc_value * 11
#define AN7_ADC1_CH15			ADC_Channel_15
#define AN8_ADC1_CH9			ADC_Channel_9
#define AN9_ADC1_CH0			ADC_Channel_0

#define AN10_ADC3_CH7			ADC_Channel_7
#define AN11_ADC3_CH10			ADC_Channel_10
#define AN12_ADC3_CH12			ADC_Channel_12

//电压 real_size = get_adc_value * 2
#define INPUT_AD_ADC3_CH5		ADC_Channel_5



//变量
extern  u16  *ADC1_Pointer;
extern  u16  ADC1_BufSize;
extern  u16  ADC1_Length ;
extern  u8   ADC1_State;
extern  u8   ADC1_Error;
extern  u16  ADC1_Buffer;

extern  u16  *ADC2_Pointer;
extern  u16  ADC2_BufSize;
extern  u16  ADC2_Length ;
extern  u8   ADC2_State;
extern  u8   ADC2_Error;
extern  u16  ADC2_Buffer;

extern  u16 * ADC3_Pointer;

#if 0
//子程序
extern void ADC1_Enable(void);
extern void ADC1_Disable(void);
extern void ADC1_Init(void);
extern void ADC1_Set(u8 baud,u8 work_mode);

extern void ADC2_Enable(void);
extern void ADC2_Disable(void);
extern void ADC2_Init(void);
extern void ADC2_Set(u8 baud,u8 work_mode);
#endif

#define ADC_SAMPLING_TIMES					(3) // max sampling times is 10

#define ADC1_ENABLE_CHANNEL_NUM				(6)
#define ADC3_ENABLE_CHANNEL_NUM				(7)
#define ADC1_3_ENABLE_CHANNEL_NUM			(ADC1_ENABLE_CHANNEL_NUM+ADC3_ENABLE_CHANNEL_NUM)

extern  u8 ADC1_channel[18];
extern  u8 ADC3_channel[18];

extern void ADC_Configuration(void);

#endif 
