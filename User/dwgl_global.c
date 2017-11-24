//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*******************************************
#include "stm32f10x.h"
struct  device_table device;
struct  file_table file_t;
struct  Addr_info2STR info2STR;
struct  LCDREG  LCDC;

struct cmd_recv_stru stru_cmd_recv;

u32 time_sys =0;
u16 step;
u16 time_s;
u16 testcmd1_time;
u16 testcmd3_time;
u32 time_s_temp;
u32 time_sys_temp1;
u32 time_sys_temp2;
u32 time_sys_temp3;
u32 time_sys_temp4;
u32 time_sys_temp5;
u8 global_u8temp;
u8 *global_u8p;
u16 global_u16temp;
u16 *global_u16p;

u16 global_u16BUFFER[128];
u8 global_u8BUFFER[128];

volatile unsigned char touch_flag;
u32 time_uart1;
u32 time_uart3;
u8 str_buffer[4100];
u8 str_buffer1[4100];

u8 LCD_TxtBuffer[2][2050];		//做显示的内存

u16 Uport_PowerSetTime[2];
u16 Uport_PowerUseTime[2];
u16 Uport_PowerShowTime[2];
u16 Dport_PowerSetTime[6];
u16 Dport_PowerUseTime[6];
u8 Dport_State[6];
u8 checking_portB;
u8 checking_portC;
u8 checking_port[2];
u8 LOW_portB;
u8 LOW_portC;

u32 file_area_addr_min[6] = {0x000000,0x080000,0x100000,0x400000,0xE00000,0xF00000};
u32 file_area_addr_max[6] = {0x01ffff,0x09FFFF,0x2FFFFF,0xBFFFFF,0xEFFFFF,0xF10000};

//u8  LCD1_CTRL8[8];			//做LCD1控制字
//u8  LCD2_CTRL8[8];			//做LCD2控制字


u32 file_addr;
u32 Rfile_addr;
u8 file_hook;
u8 file_wr;
u8 file_id;						//当前正写文件的ID
u32 NextFileAddr;
u32 check_time;
#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
u32 KEY_time;
#endif
//u32 display_time;
u8  UART_BUFFER[128];
u8  UART1_TXBUFFER[128];
u8  UART1_RXBUFFER[UART1_RXBUFFER_SIZE];
u16  UART1_RXBUFFE_HEAD =0;		//有效内容的第一个
u16  UART1_RXBUFFE_LAST =0;		//有效内容的最后一个
u8  UART2_TXBUFFER[128];
u8  UART2_RXBUFFER[128];
u8  UART3_TXBUFFER[128];
u8  UART3_RXBUFFER[UART3_RXBUFFER_SIZE];
u16  UART3_RXBUFFE_HEAD;		//有效内容的第一个
u16  UART3_RXBUFFE_LAST;		//有效内容的最后一个
u8  SPI_BUFFER[128];

u16 ADC_BUFFER[ADC_BUFFER_SIZE];

//u8 AINx_ADCch[18]={ADC_Channel_13,ADC_Channel_12,ADC_Channel_0,ADC_Channel_1,ADC_Channel_3,ADC_Channel_2,ADC_Channel_11,ADC_Channel_10,0};
u8 AINx_ADCch[18]={ADC_Channel_13,ADC_Channel_12,ADC_Channel_0,ADC_Channel_1,ADC_Channel_3,ADC_Channel_0,ADC_Channel_2,ADC_Channel_1,0};

u16 ADC_Base0[18];				//ADC静态值
u8 device_num[20];
#ifndef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
u16 AD_count[64];				//广告计数
u8 charge_speed[2];				//充电速度	
#endif
u8  SF_REG = 7;					//0-7. =7时，采样速度最慢 
u8  GAIN_REG;

/* 波形数据 ---------------------------------------------------------*/
uint16_t Sine12bit[32] = {
	2448,2832,3186,3496,3751,3940,4057,4095,4057,3940,
	3751,3496,3186,2832,2448,2048,1648,1264,910,600,345,
	156,39,0,39,156,345,600,910,1264,1648,2048
};
/* 坐标数据 ---------------------------------------------------------*/
uint16_t co_x[10] = {
	103,152,201,
	103,152,201,
	103,152,201,
	0
};
/* 坐标数据 ---------------------------------------------------------*/
uint16_t co_y[10] = {
	50,50,50,
	99,99,99,
	148,148,148,
	0
};
/* 九宫数据 ---------------------------------------------------------*/
uint16_t xy_3[3][3] = {
	0,1,2,
	3,4,5,
	6,7,8
};

//要管理中断的优先级分组
void NVIC_Configuration(void)
{
	////智嵌上的配置
	NVIC_InitTypeDef   NVIC_InitStructure;
	//设定向量表所处的位置，必须在分组前设置好，默认的是NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	// Set the Vector Table base location at 0x08000000
	// NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

	// 2 bit for pre-emption priority, 2 bits for subpriority 
	//复位后默认为分组0,工程中只设置一次
	//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 

	// Enable the USART1 global Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Enable the USART2 global Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 

	// Enable the USART3 global Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Enable the SPI1 global Interrupt 
	// NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;	  
	// NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	// NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// NVIC_Init(&NVIC_InitStructure);

	// Enable the ADC1_2 global Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Enable the ADC3 global Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = ADC3_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Enable the DMA1_Channel1 global Interrupt. ADC1 DMA 中断方式
	// NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;	  
	// NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	// NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
	// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// NVIC_Init(&NVIC_InitStructure); 

	// Enable the TIM3 global Interrupt 
	// NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;	  
	// NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	// NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// NVIC_Init(&NVIC_InitStructure);

	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	// 	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	// 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// 	NVIC_Init(&NVIC_InitStructure);

	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	// 	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	// 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// 	NVIC_Init(&NVIC_InitStructure);
}
