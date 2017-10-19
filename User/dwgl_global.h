//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*******************************************
#ifndef __DWGL_GLOBAL_H
#define	__DWGL_GLOBAL_H

#include "stm32f10x.h"

#if defined (STM32F10X_LD)
#error " target STM32F10x device used (STM32F10X_LD) (in dwgl_global.h file)"
#endif

#if defined (STM32F10X_LD_VL)
#error " target STM32F10x device used (STM32F10X_LD_VL) (in dwgl_global.h file)"
#endif
#if defined (STM32F10X_MD)
#error " target STM32F10x device used (STM32F10X_MD) (in dwgl_global.h file)"
#endif
#if defined (STM32F10X_MD_VL) 
#error " target STM32F10x device used (STM32F10X_MD_VL) (in dwgl_global.h file)"
#endif
#if !defined (STM32F10X_HD)
#error " target STM32F10x device used (STM32F10X_HD) (in dwgl_global.h file)"
#endif
#if defined (STM32F10X_HD_VL)
#error " target STM32F10x device used (STM32F10X_HD_VL) (in dwgl_global.h file)"
#endif
#if defined (STM32F10X_XL)
#error " target STM32F10x device used (STM32F10X_XL) (in dwgl_global.h file)"
#endif
#if defined (STM32F10X_CL)
#error " target STM32F10x device used (STM32F10X_CL) (in dwgl_global.h file)"
#endif
//cmd
#define frame_headerC				0x67
#define frame_headerD				0x68
#define frame_last					0x99
#define Version_FLAG1				('B')  //版本标志1
//#define port_num					0x02
#define cmd_set_addr				0xe2  //设充电控制板地址命令

#define Addr_01min					0x000000   //二维码地址始
#define Addr_01max					0x01ffff   //二维码地址末
#define Addr_02min					0x080000   //小图片地址始
#define Addr_02max					0x09FFFF   //小图片地址末
#define Addr_03min					0x100000   //大图片地址始
#define Addr_03max					0x2FFFFF   //大图片地址末
#define Addr_04min					0x400000   //大图片地址始
#define Addr_04max					0xBFFFFF   //大图片地址末
#define Addr_05min					0xE00000   //程序地址起
#define Addr_05max					0xEFFFFF   //程序地址末
#define Addr_info1					0xF00000   //信息地址1
#define Addr_info2					0xF10000   //信息地址2

#define GLOBLE_ADDR					0xE0
#define PC_ADDR						0xF0
#define Broadcast					0xFF

#define ADC_BUFFER_SIZE				320

#define ADC_LINE1					0x0c0  //低于些电压认为有手机;=0xc0波动为310mV,这个可能要做补偿
#define ADC_LINE2					0x080  //电流波动范围 ;=0x80 波动为20mA
#define ADC_LINE3					0x060  //电流波动范围 ;=0x60 波动为15mA  电流小于此值表示没有使用
//#define ADC_LINE2					0x0100  //电流波动范围 ;=0x100 波动为40mA

#define UART1_RXBUFFER_SIZE			0x200
#define UART1_RX_MAX				0x1FF
#define UART3_RXBUFFER_SIZE			0x80
#define UART3_RX_MAX				0x7f

struct  Addr_info2STR {
	u8 head[2];
	u8 temp[3];
	u8 item1[2];		//广告控制项与项字长 
	u8 item1_data[4]; 
	u8 item2[2];		//屏保控制项与项字长      
	u8 item2_data[4]; 
	u8 item3[2];		//图片标记控制项与项字长      
	u8 item3_data[2]; 
	u8 item11[2];		//图片版本控制项与项字长      
	u8 item11_data[16]; 
	u8 item12[2];		//图片版本控制项与项字长      
	u8 item12_data[16]; 
	u8 item13[2];		//图片版本控制项与项字长      
	u8 item13_data[16]; 
	u8 item14[2];		//图片版本控制项与项字长      
	u8 item14_data[16]; 
	u8 item15[2];		//图片版本控制项与项字长      
	u8 item15_data[16]; 
	u8 item21[2];		//程序更新设定项
	u8 item21_data[4];
	u8 item81[2];		//设备号项
	u8 item81_data[16]; 
	u8 item31[2];		//B类程序版本项与项字长 
	u8 item31_data[16];
	u8 item32[2];		//A类程序版本项与项字长 
	u8 item32_data[16];
};	

struct  device_table {
	u8 head;
	u8 addr;
	u16 Toolname;
	u8 	Version[12];
	u8 port_num; 
	u8 hub_id[8];
	u8 port_id[8];		//只用了前面两个保存端口号
	u8 use;				//使用情况 bit0=1连接断； bit1=1地址与二维码充突； bit2=1，0号无二维码 ； bit3=1，1号无二维码 ；bit4=1，二维码有更新
	u8 TASK_state;		//任务情况对程序
};	  

struct  file_table {
	u8 f_class;
	u8 f_unm;
	u8 f_name[8];
	u8 f_size[4];
	u8 f_addr[4];
};

//str_buffer[0]=	//头0X67
//str_buffer[?]=	//LEN
//str_buffer[?]=	//文件区u8
//str_buffer[?]=	//文件号u8
//str_buffer[?]=	//文件名u64
//str_buffer[?]=	//文件大小u32
//str_buffer[?]=	//文件起地址u32
//str_buffer[?]=	//check
//str_buffer[?]=	//尾0X99

#define LCD1_INDEX			(0)
#define LCD2_INDEX			(1)

struct  LCDREG {
	u8 Head;				//头
	//u8 LCD1POFFTime;		//断电时间        
	//u8 LCD2POFFTime;		//断电时间     
	u8 SPSwitch;			//屏保开关
	u16 LCDSPTimeSet;		//屏保间隔        
	//u16 LCD1SPTime;			//屏保间隔计数       
	//u16 LCD2SPTime;			//屏保间隔计数         
	//u8 LCD1SPPID;			//屏保ID      
	//u8 LCD2SPPID;			//屏保ID     
	u8 PSwitch;				//广告开关
	u8 PNum;				//广告个数
	u16 LCDPTimeSet;		//广告间隔        
	//u16 LCD1PTime;			//广告间隔计数         
	//u16 LCD2PTime;			//广告间隔计数         
	//u8 LCD1PID;				//广告图片ID        
	//u8 LCD2PID;				//广告图片ID

	u8 LCDPOFFTIME[2];		//断电时间
	u16 LCDSPTime[2];		//屏保间隔计数
	u8 LCDSPPID[2];			//屏保ID
	u16 LCDPTime[2];		//广告间隔计数
	u8 LCDPID[2];			//广告图片ID
};	 

extern struct device_table device;
extern struct file_table * file_p;
extern struct file_table file_t;
extern struct Addr_info2STR info2STR;
extern struct LCDREG LCDC;
extern u16 step;
extern u16 time_s;
extern u16 testcmd1_time;
extern u16 testcmd3_time;
extern u32 time_s_temp;
//extern u32 time_sys_temp1;
//extern u32 time_sys_temp2;
//extern u32 time_sys_temp3;
//extern u32 time_sys_temp4;
//extern u32 time_sys_temp5;
extern u32 time_sys;
extern u8 global_u8temp;
extern u8 *global_u8p;
extern u16 global_u16temp;
extern u16 *global_u16p;
extern u16 global_u16BUFFER[128];
extern u8 global_u8BUFFER[128];

extern volatile unsigned char touch_flag;
extern u32 time_uart1;
extern u32 time_uart3;
extern u8 str_buffer[4100];			//做显示的内存
extern u8 LCD_TxtBuffer[2][2050];	//做显示的内存
extern u16 Uport_PowerSetTime[2];
extern u16 Uport_PowerUseTime[2];
extern u16 Uport_PowerShowTime[2];
extern u16 Dport_PowerSetTime[6];
extern u16 Dport_PowerUseTime[6];
extern u8 Dport_State[6];
//extern u8 checking_portB;			//检测阶段
//extern u8 checking_portC;
extern u8 checking_port[2];
//extern u8 LOW_portB;				//低电流计数
//extern u8 LOW_portC;

//extern u8 LOW_portB;
//extern u8 LOW_portC;
//extern u8 LCD1_CTRL8[8];			//做LCD1控制字
//extern u8 LCD2_CTRL8[8];			//做LCD2控制字

extern u32 file_addr;				//写文件时地址
extern u32 Rfile_addr;				//读文件时地址
extern u8 file_hook;
extern u8 file_wr;
extern u8 file_id;					//当前正写文件的ID
extern u32 NextFileAddr;
extern u32 check_time;
//extern u32 display_time;
extern u8 UART_BUFFER[128];
extern u8 UART1_TXBUFFER[128];
extern u8 UART1_RXBUFFER[UART1_RXBUFFER_SIZE];
extern u16 UART1_RXBUFFE_HEAD;		//有效内容的第一个
extern u16 UART1_RXBUFFE_LAST;		//有效内容的最后一个
extern u8 UART2_TXBUFFER[128];
extern u8 UART2_RXBUFFER[128];
extern u8 UART3_TXBUFFER[128];
extern u8 UART3_RXBUFFER[UART3_RXBUFFER_SIZE];
extern u16 UART3_RXBUFFE_HEAD;		//有效内容的第一个
extern u16 UART3_RXBUFFE_LAST;		//有效内容的最后一个
extern u8 SPI_BUFFER[128];
extern u16 ADC_BUFFER[ADC_BUFFER_SIZE];
extern u8 AINx_ADCch[18];
extern u16 ADC_Base0[18];			//ADC静态值
extern u8 device_num[20];
extern u16 AD_count[64];			//广告计数
extern u8 charge_speed[2];			//充电速度

extern 	u8  SF_REG;
extern 	u8  GAIN_REG;

/* 波形数据 ---------------------------------------------------------*/
extern  uint16_t Sine12bit[32];
/* 坐标数据 ---------------------------------------------------------*/
extern  uint16_t co_x[10];
/* 坐标数据 ---------------------------------------------------------*/
extern  uint16_t co_y[10];
/* 九宫数据 ---------------------------------------------------------*/
extern  uint16_t xy_3[3][3];


#define USB_POWER_ON		(0)
#define USB_POWER_OFF		(1)
#define USB_ALL_INDEX		(0xff)


#define LED_TURN_ON			(0)
#define LED_TURN_OFF		(1)
#define LED_TURN_NEGATION	(2)	//make the LED state from ON to OFF, or from OFF to ON

#define LED_INDEX			(0)
#define LED1_INDEX			(1)
#define LED2_INDEX			(2)
#define LED_ALL_INDEX		(0xff)

extern void NVIC_Configuration(void);
#endif
