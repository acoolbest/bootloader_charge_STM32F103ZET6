#ifndef  PM25LQ512FLASH_H
#define  PM25LQ512FLASH_H
// #define FLASH_IC  PM25LV512   //512Kbit PMC公司
// #define FLASH_IC  PM25LQ512   //512Kbit ISSI公司
// #define FLASH_IC  W25Q128   //128Mbit   Win bond 华邦
// #define FLASH_IC  N25Q256   //256Mbit 美光(Micron)

#include "stm32f10x.h"
//define
//#define   PM25LV512   //512Kbit PMC公司
//#define   PM25LQ512   //512Kbit ISSI公司
#define   W25Q128   //128Mbit   Win bond 华邦
//#define   N25Q256   //256Mbit 美光(Micron)


//-----------------FLASH--PM25LQ512指令------------------------------
#if defined (PM25LQ512) 
	#define  FLASH_CMD_READ        0x03        //读取数据
	#define  FLASH_CMD_FAST_READ   0x0B        //快速读取数据
	#define  FLASH_CMD_PP          0x02        //页面数据写入
	#define  FLASH_CMD_WRDI        0x04        //写无效
	#define  FLASH_CMD_WREN        0x06        //写使能
	#define  FLASH_CMD_RDSR        0x05        //读状态寄存器
	#define  FLASH_CMD_WRSR        0x01        //写状态寄存器
	#define  FLASH_CMD_RDFR        0x48        //读功能寄存器
	#define  FLASH_CMD_WRFR        0x42        //写功能寄存器
	#define  FLASH_CMD_SER         0xD7        //扇区擦除 
	#define  FLASH_CMD_SE          0xD8        //单块擦除
	#define  FLASH_CMD_CER         0xC7  	     //整片擦除
	#define  FLASH_CMD_RDID        0xAB  	     //读电子标签
	#define  FLASH_CMD_RDJDID      0x9F  	     //读标识
	
	#define  SPI_FLASH_PageSize    256       //每个页面字节大小
	#define  Sector_Size        	 4096  	   //字节
	#define  Block_Size        	   32768  	 //字节
	#define  Chip_Size        	   65536  	 //字节
	#define  FLASH_MAX_CAP         0X00FFFF  	   //最大地址
#endif

#if defined (W25Q128) 
	#define  FLASH_CMD_READ        0x03        //读取数据
	#define  FLASH_CMD_FAST_READ   0x0B        //快速读取数据
	#define  FLASH_CMD_PP          0x02        //页面数据写入
	#define  FLASH_CMD_WRDI        0x04        //写无效
	#define  FLASH_CMD_WREN        0x06        //写使能
	#define  FLASH_CMD_RDSR        0x05        //读状态寄存器
	#define  FLASH_CMD_WRSR        0x01        //写状态寄存器
	#define  FLASH_CMD_RDFR        0x48        //读功能寄存器
	#define  FLASH_CMD_WRFR        0x42        //写功能寄存器
	#define  FLASH_CMD_SER         0x20        //扇区擦除 
	#define  FLASH_CMD_SE          0xD8        //单块擦除
	#define  FLASH_CMD_CER         0xC7  	     //整片擦除
	#define  FLASH_CMD_RDID        0xAB  	     //读电子标签
	#define  FLASH_CMD_RDJDID      0x9F  	     //读标识
	
	#define  SPI_FLASH_PageSize    256       //每个页面字节大小
	#define  Sector_Size        	 4096  	   //字节
	#define  Block_Size        	   32768  	 //字节
	#define  Chip_Size        	   16777216  	 //字节
	#define  FLASH_MAX_CAP         0XFFFFFF  	   //最大地址
#endif

#if defined (N25Q256) 
	#define  FLASH_CMD_READ        0x03        //读取数据
	#define  FLASH_CMD_FAST_READ   0x0B        //快速读取数据
	#define  FLASH_CMD_PP          0x02        //页面数据写入
	#define  FLASH_CMD_WRDI        0x04        //写无效
	#define  FLASH_CMD_WREN        0x06        //写使能
	#define  FLASH_CMD_RDSR        0x05        //读状态寄存器
	#define  FLASH_CMD_WRSR        0x01        //写状态寄存器
	#define  FLASH_CMD_RDFR        0x48        //读功能寄存器
	#define  FLASH_CMD_WRFR        0x42        //写功能寄存器
	#define  FLASH_CMD_SER         0x20        //扇区擦除 
	#define  FLASH_CMD_SE          0xD8        //单块擦除
	#define  FLASH_CMD_CER         0xC7  	     //整片擦除
	#define  FLASH_CMD_RDID        0xAB  	     //读电子标签
	#define  FLASH_CMD_RDJDID      0x9F  	     //读标识
// 	#define  FLASH_CMD_ENTER_4BYTEADDRESS_MODE   0xb7//进入4地址  N25Q256独有
// 	#define  FLASH_CMD_4BYTE_READ                0x0c  	     //
	
	#define  SPI_FLASH_PageSize    256       //每个页面字节大小
	#define  Sector_Size        	 4096  	   //字节
	#define  Block_Size        	   65536  	 //字节
	#define  Chip_Size        	   33554432  	 //字节
	#define  FLASH_MAX_CAP         0X1FFFFFF  	   //最大地址
#endif
	#define  FLASH_CMD_ENTER_4BYTEADDRESS_MODE   0xb7//进入4地址  N25Q256独有
	#define  FLASH_CMD_4BYTE_READ                0x0c  	     //

/////////////////////////////////////////////////////////////////////////////////

#define  FLASH_CMD_ERASE_RESUME       0x7A       //擦除恢复
#define  FLASH_CMD_ERASE_SUSPEND      0x75       //擦除挂起
#define  FLASH_CMD_SECUNLOCK          0x26       //扇区开锁
#define  FLASH_CMD_SECLOCK            0x24       //扇区上锁

//BEGIN----------------------------------------
#ifndef A4_SPI1_CS_PIN
#define A4_SPI1_CS_PIN				GPIO_Pin_4 		//GPIO_Mode_Out_PP
#endif

#define FLASH1CS					A4_SPI1_CS_PIN
#define FLASH1CS_PORT				GPIOA
//END----------------------------------------

// #define  FLASH1CS_PORT     GPIOC
// #define  FLASH1CS          GPIO_Pin_13
#define  Enable            1
#define  Disable           0
//#define  fast_read         1
#define  fast_read         0

extern 	u8 FLASH_u8temp;
extern 	u16 FLASH_u16temp;
	//addr:0x000000-0x003fff  16kB 二维码区
	//addr:0x004000-0x00ffff  48kB 文字区
	//addr:0x01ffff-0xFFFFFF  ?kB 彩图区

void FLASH2_CSSelect(unsigned char cs_select); //片选信号
void FLASH2_WRREG(unsigned char FLASH_CMD,unsigned char rdsr_data);		//写寄存器 FLASH_CMD_WRSR 
unsigned char FLASH2_RDREG(unsigned char FLASH_CMD);                  //读寄存器 FLASH_CMD_WRSR readsrname寄存器读取命令
void FLASH2_WriteEnable(unsigned char cs_select); //开启写使能	 
void FLASH2_WRDI(unsigned char cs_select);				//写使能无效WRDI


void FLASH2_IsBusy(unsigned char cs_select);     //判断是否在写  擦除      
void UnprotectFlash2(void); 	            //  去FLASH整片写保


unsigned char FLASH2_ReadByte(void); 									//从FLASH读取一个字节
void FLASH2_WriteByte(unsigned char value); 					//向FLASH写一个字节
void FLASH2_Read_to_Buffer (unsigned int write_addr,unsigned char *buffer, unsigned int len);//FLASH-读出LEN个字节
void FLASH2_PageWrite_from_Buffer(unsigned int write_addr, unsigned char *desbuffer,unsigned int len); // reentrant//FLASH-写入LEN个字节

void  FLASH2_SER(unsigned int u32_addr);//按页擦除部分flash  每次擦擦4K
void FLASH2_CER(unsigned char cs_select); //擦除整片FLASH	

void  FLASH2_ReadID(unsigned char cs_select,unsigned char *buffer);//读ID 
unsigned char  FLASH2_RDSR(unsigned char cs_select,unsigned char readsrname);//读取状态寄存器 readsrname寄存器读取命令


//dwgl-20160811修改
void FLASH_Read_4Byte(unsigned char addr1,unsigned char addr2, unsigned char addr3,unsigned char addr4, unsigned char *buffer, unsigned int len);//FLASH-读出LEN个字节
//void FLASH_Read_3Byte(void);
void FLASH_Write_4Byte(unsigned char addr1,unsigned char addr2, unsigned char addr3,unsigned char addr4, unsigned char *buffer, unsigned int len);//FLASH-写入LEN个字节，LEN<=256
//void FLASH_Read_3Byte(void);
//void ENTER_4BYTEADDRESS_MODE(void);
//void EXIT_4BYTEADDRESS_MODE(void);
void  FLASH_ERASE4K_4Byte(unsigned char addr1,unsigned char addr2, unsigned char addr3,unsigned char addr4);//按页擦除部分flash  每次擦擦4K

void  delay_cycle(unsigned int cycles);

#endif  //end PM25LQ512_H



/*

CPHA 
0
1
CPOL
0
1

PM25LQ512SPI工作模式 CLK空闲=0，下降沿更新数据，上升沿锁存数据。

*/

