#include "PM25LQ512FLASH.h"
#include "stm32f10x.h"
/*
//PM25LQ512FLASH  ISSI公司

-  Pm25LQ040B: 4Mbit/512Kbyte
-  Pm25LQ020B: 2Mbit/256Kbyte
-  Pm25LQ010B: 1Mbit/128Kbyte
-  Pm25LQ512B: 512Kbit/64Kbyte

-  Uniform 4 Kbyte Sectors（扇区） or 32/64 Kbyte Blocks（块）
-  Flexible 4, 32, 64 Kbytes, or Chip Erase  擦除的最小单位为4K
-  Standard Page Program 1 to 256 bytes 每次写入最多256字节，每个页为256字节

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

堆叠设备 两个256Mb

每页4KB字节 16页为一个子扇区 共64K字节 每次擦除最小单位

Sector 共1024个
Subsector  共16384个


512MB N25Q叠装置包含两个256MB模具。 从用户的角度来看，这 作为一个单片器件堆叠设备的行为，除非涉及到读存储器 和擦除操作和状态轮询。 该器件包含一个单芯片选择（S＃）; 双芯片版本也可用。
请与工厂联系获取更多信息。 内存1024（64KB）组织的主要部门，进一步分成16 每个界别分组（共16,384界别分组）。 存储器可擦除4KB子 部门在同一时间，64KB的时间，部门或单芯片（256MB）在同一时间
。 易失性和非易失性内存可以通过软件写保护亲 TECTION功能，根据实际应用的需要。 保护粒度是 64KB（扇区粒度）挥发性保护 该器件具有64一次性可编程（OTP），可以读取的字节和程序 MED读取OTP和程序OTP命令。
这64个字节也可以是 永久锁定程序OTP命令。 该设备还可以暂停和恢复使用专用的编程和擦除周期 编程/擦除挂起和恢复指令。


地址和3字节4字节地址模式 该器件采用3字节或4个字节的地址模式，访问内存超过128MB。 当启用了4个字节的地址模式，所有命令都需要一个地址必须是EN- tered和一个4字节的地址模式命令退出
：进入4个字节的地址 模式命令和EXIT 4字节地址模式命令。 4个字节的地址 模式也可以通过非易失性配置寄存器启动。 请参见寄存器 了解更多信息。 运营协议 该存储器可与三种不同的协议操作：
?扩展SPI（标准SPI协议升级双核和四操作） ?双I / O SPI ?四I / O SPI 标准SPI协议的扩展和增强，双核和四操作。 在 此外，双SPI和四SPI协议提高了数据的存取时间和 通过一个单一的I / O设备发送的命令
，地址和数据 跨两个或四个数据线。 每个协议包含独特的命令来执行读操作DTR模式。 这使高数据吞吐量，同时运行在较低的时钟频率


第1页
美光的串行NOR闪存 3V，多个I / O，4KB扇区擦除 N25Q512A 特点 ?堆叠设备（两个256MB模具） ?SPI兼容的串行总线接口 ?双传输速率（DTR）模式 ?2.7-3.6V单电源电压 
?108 MHz的时钟频率（最大）支持所有 协议在单一的传输速率（STR）模式 ?54兆赫（MAX）支持所有的时钟频率 DTR模式协议 ?双/四I / O指令提供了增强的 吞吐量最高可达54 MB / s的 ?支持的协议。
-扩展SPI，双I / O和四I / O -支持所有的DTR模式 ?就地执行（XIP）模式，这三个协议 -可通过易失性或非易失性寄存器 -启用内存直接XIP模式AF- 上电后 ?编程/擦除挂起操作 ?可用协议 -读操作
-四核或双输出快读 -四核或双I / O快读 ?灵活，以适应应用 -可配置的数字哑周期 -输出缓冲区配置 ?软件复位 ?其他复位引脚选定部分号码 1 ?支持3字节和4字节的可寻址模式 ?64字节，用户锁定，
一次性可编程 （OTP）专用区 ?擦除能力 -界别分组擦除的4KB粒度均匀块 -扇区擦除64KB粒度均匀块 -单芯片擦除 ?写保护 -软件写保护功能，适用于所有 64KB部门通过挥发性锁定位 
-硬件写保护：保护区面积 所定义的五种非易失性位（BP0，BP1，BP2， BP3，TB） -额外的智能保护功能，可根据重新 探索 ?电子签名 -符合JEDEC标准的2个字节的签名（BA20h）
-独特的ID码（UID）：17只读字节 包括：两个额外的扩展设备ID 字节识别设备厂选项; CUS 的工厂数据tomized（14字节） ?最低100,000每扇区擦除周期 ?超过20年的数据保存 
?软件包-符合JEDEC标准，所有符合RoHS标准 - V-PDFN-8/8mm x 6mm的（也被称为SON DFPN，MLP，MLF） -的SOP2-16/300mils（也被称为SO16W，SO16- 宽，SOIC-16） - T-PBGA-24b05/6mm，宽8mm（也称为 TBGA24）


Sector Erase Time (4Kbyte)   – 70(T)    300(max)  ms
Chip Erase Time for 512kb    – 0.25(T)  1(max)    s 

//-----------------FLASH--PM25LQ512指令------------------------------
#define  FLASH_CMD_READ        0x03			//读取数据
#define  FLASH_CMD_FAST_READ   0x0B			//快速读取数据
#define  FLASH_CMD_PP          0x02			//页面数据写入
#define  FLASH_CMD_WRDI        0x04			//写无效
#define  FLASH_CMD_WREN        0x06			//写使能
#define  FLASH_CMD_RDSR        0x05			//读状态寄存器
#define  FLASH_CMD_WRSR        0x01			//写状态寄存器
#define  FLASH_CMD_RDFR        0x48			//读功能寄存器
#define  FLASH_CMD_WRFR        0x42			//写功能寄存器
#define  FLASH_CMD_SER         0xD7			//扇区擦除
#define  FLASH_CMD_SE          0xD8			//单块擦除
#define  FLASH_CMD_CER         0xC7			//整片擦除
#define  FLASH_CMD_RDID        0xAB			//读电子标签
#define  FLASH_CMD_RDJDID      0x9F			//读标识

 */
 
u8 FLASH_u8temp =0;
u16 FLASH_u16temp =0;


void FLASH2_CSSelect(unsigned char cs_select)   //片选使能
{
	if(cs_select==Enable)
	{
		GPIO_ResetBits(FLASH1CS_PORT, FLASH1CS);   //RAM片选工作
	}
	else
	{
		GPIO_SetBits(FLASH1CS_PORT, FLASH1CS);   //RAM片选不工作
	}
}

//-------FLASH_CMD_WRSR    0x01 //写状态寄存器;FLASH_CMD_WRFR    0x42//写功能寄存器---------------------------
void  FLASH2_WRREG(unsigned char FLASH_CMD,unsigned char rdsr_data)//写寄存器
{
	//RAM_CS = 1;FLASH_CS = 0;   
	FLASH2_CSSelect(Enable);             

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = FLASH_CMD  ;     // 写状态寄存器 FLASH_CMD_WRSR

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = rdsr_data  ;     //  写
	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	delay_cycle(2);

	//RAM_CS = 1;FLASH_CS = 1; 
	FLASH2_CSSelect(Disable);
}

//---------FLASH_CMD_RDSR    0x05//读状态寄存器; FLASH_CMD_RDFR   0x48//读功能寄存器 ---------------------------
unsigned char FLASH2_RDREG(unsigned char FLASH_CMD)//读寄存器 
{
	unsigned char  returndata; 
	
	//RAM_CS   = 1;FLASH_CS   = 0;
	FLASH2_CSSelect( Enable);

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = FLASH_CMD;      //读状态寄存器

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = FLASH_CMD;      //读状态寄存器

	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	returndata = SPI1->DR;
	//RAM_CS = 1;FLASH_CS = 1;
	FLASH2_CSSelect(Disable);
	delay_cycle(1);

	return returndata;
}

//------FLASH-读出LEN个字节---------
//---注意写入和读出之间最少要有10MS延时------------
void FLASH2_Read_to_Buffer (unsigned int write_addr, unsigned char *buffer, unsigned int len)//FLASH-读出LEN个字节-
{
	//U16  i;
	unsigned char addr1,addr2,addr3;
	addr1 = ((write_addr&0xff0000)>>16);
	addr2 = ((write_addr&0xff00)>>8);
	addr3 = ((write_addr&0xff));
	//RAM_CS   = 1;FLASH_CS   = 0;  //片选使能
	//delay_cycle(2);
	FLASH2_CSSelect( Enable);                  

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = FLASH_CMD_READ;       //读取数据

	//FLASH_u8temp=SPI1->DR;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = addr1;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = addr2;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = addr3;

	if(fast_read ==1)
	{
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI1->DR = 0;      //读状态寄存器
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		FLASH_u8temp=SPI1->DR;
		while(len>0)
		{
			// buffer[i]= FLASH2_ReadByte();
			while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
			SPI1->DR = 0;      //读状态寄存器
			while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
			*buffer=SPI1->DR;
			buffer++;
			len--;
		}
	}
	else
	{
		delay_cycle(10);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		FLASH_u8temp=SPI1->DR;
		while(len>0)
		{
			// buffer[i]= FLASH2_ReadByte();
			while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
			SPI1->DR = 0;      //读状态寄存器
			while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
			*buffer=SPI1->DR;
			buffer++;
			len--;
		}
	}
	delay_cycle(2);
	FLASH2_CSSelect(Disable);
}
//------FLASH-写入LEN个字节---------
void FLASH2_PageWrite_from_Buffer(unsigned int write_addr, unsigned char *desbuffer,unsigned int len) // reentrant//FLASH-写入LEN个字节
{
	unsigned char addr1,addr2,addr3;
	addr1 = ((write_addr&0xff0000)>>16);
	addr2 = ((write_addr&0xff00)>>8);
	addr3 = ((write_addr&0xff));

	//RAM_CS = 1;FLASH_CS = 0;
	FLASH2_CSSelect(Enable);      	                   
	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	/* Send byte through the SPI2 peripheral */
	//SPI_I2S_SendData(SPI1, data);
	SPI1->DR = FLASH_CMD_WREN;     //  写使能 

	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	/* Return the byte read from the SPI bus */
	//return SPI_I2S_ReceiveData(SPI1);
	FLASH_u8temp = SPI1->DR;
	delay_cycle(2);
	FLASH2_CSSelect(Disable);

	//RAM_CS = 1;FLASH_CS = 0;
	FLASH2_CSSelect( Enable);              
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = FLASH_CMD_PP; //写页

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = addr1; 	 //写地址

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = addr2;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = addr3;

	while(len>0)
	{
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI1->DR  = *desbuffer; 	 //写数据
		desbuffer++;
		len--;
	}

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	delay_cycle (2);
	FLASH2_CSSelect(Disable);

	//FLASH2_CSSelectEnable( cs_select);
	//Delay_ms (1);

	//SPI0DAT  = FLASH_CMD_WRDI;        //写无效
	//while (!SPIF);
	//SPIF     = 0;

	//RAM_CS   = 1;FLASH_CS   = 1;
	//FLASH2_CSSelectDisable(cs_select);
	//Delay_ms (1);  

	FLASH2_IsBusy(1);
}

//-----扇区擦擦4K-------------------------------------------
void  FLASH2_SER(unsigned int u32_addr)//扇区擦除部分flash  每次擦擦4K
{
	//RAM_CS = 1;FLASH_CS = 0; 
	unsigned char addr1,addr2,addr3;
	addr1 = ((u32_addr&0xff0000)>>16);
	addr2 = ((u32_addr&0xff00)>>8);
	addr3 = ((u32_addr&0xff));
	FLASH2_CSSelect(Enable);

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = FLASH_CMD_WREN  ;     //  写使能

	//RAM_CS = 1;FLASH_CS = 1; 
	delay_cycle(10);
	FLASH2_CSSelect(Disable);
	delay_cycle(2);

	//---------FLASH_CMD_SE--------------
	//RAM_CS = 1;FLASH_CS = 0;
	FLASH2_CSSelect(Enable);

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = FLASH_CMD_SER;//         0xD7        //扇区擦除
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = addr1;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = addr2;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = addr3;
	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	delay_cycle(2);

	//RAM_CS = 1;FLASH_CS = 1;
	FLASH2_CSSelect(Disable);

	//-------指导擦除结束------------------
	//RAM_CS   = 1;FLASH_CS   = 0;
	FLASH2_IsBusy(1);
	delay_cycle(2);
}
//-----------------------------------------------------------------------------
//  FLASH_BE
//  功能：擦除整片flash 整个芯片512Mb
//------------------------------------------------------------------------------
void  FLASH2_CER(unsigned char cs_select) //擦除整片FLASH
{
	/*   
	FLASH2_IsWriteEnble(0x01); //判断是否在写  擦除
	FLASH2_WDExtended_AddressRegister(cs_select ,0x00); //写地址的最高位31-25    zai 24-25   00  默认128       地址00000000h-00FFFFFFh
	FLASH2_IsBusy( cs_select);

	Delay_ms (200); 
	FLASH2_DIE_ERASE( cs_select,0x00,0x00, 0x00);//按页擦除部分flash  每次擦擦256mb  
	*/
	//EA = 0;
	FLASH2_IsBusy(1);

	FLASH2_CSSelect( Enable);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = FLASH_CMD_WREN;     //写使能
	delay_cycle(10);
	FLASH2_CSSelect(Disable);

	FLASH2_CSSelect( Enable);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  =	FLASH_CMD_CER;
	delay_cycle(10);
	FLASH2_CSSelect(Disable);

	FLASH2_CSSelect( Enable);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  =	FLASH_CMD_CER;
	delay_cycle(10);
	FLASH2_CSSelect(Disable);
	delay_cycle (10000);

	///-------指导擦除结束------------------ 
	FLASH2_IsBusy(1);
	//EA = 1;
}

//--------------判断是否在写  擦除--------------------
void FLASH2_IsBusy(unsigned char cs_select) //判断是否在写  擦除
{
	//unsigned char i;
	do
	{
		FLASH2_CSSelect(Enable);
		delay_cycle(10);
		FLASH_u8temp = SPI1->DR;

		//SPI0DAT  = FLASH_CMD_RDSTATUSR;
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI1->DR  =	FLASH_CMD_RDSR;
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		FLASH_u8temp = SPI1->DR;

		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI1->DR  =	FLASH_CMD_RDSR;
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		FLASH_u8temp = SPI1->DR;

		FLASH2_CSSelect(Disable);
		//---LED提示----------
		/*			 
		i++;
		if(i==250)
		{
			i=0;
			j++;
			if(j==250)
			{ 
				j=0;				  
				LED =~LED;				  
			}
		}
		*/
	} 
	//while(((SPI1->DR) & 0x01) == 0x01); //Write In Progress Bit:"0" indicates the device is ready (default);"1" indicates a write cycle is in progress and the device is busy.
	while(((FLASH_u8temp) & 0x01) == 0x01); //Write In Progress Bit:"0" indicates the device is ready (default);"1" indicates a write cycle is in progress and the device is busy.
}
//-------------  去FLASH整片写保护  ，系统上电时调用一次-----------------------   
void UnprotectFlash2(void) //  去FLASH整片写保护
{
	FLASH2_WRREG(FLASH_CMD_WRSR,0x00);
}
//-----------读ID-------------
void  FLASH2_ReadID(unsigned char cs_select,unsigned char *buffer)//N25Q256 20 BA 19 10 00 00 23 14 75 44 08 00 41 00 25 27 01 12 C5 0E
{
	unsigned char  i;

	//RAM_CS = 1;FLASH_CS = 0;   
	FLASH2_CSSelect( Enable);

	delay_cycle(10);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = FLASH_CMD_RDID;  	     //读电子标签
	for(i=0;i<4;i++)
	{
		buffer[i]=0;
	}
	for(i=0;i<4;i++)
	{
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI1->DR = 0;      //读
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		buffer[i]=SPI1->DR;
	}

	//RAM_CS = 1;FLASH_CS = 1;
	FLASH2_CSSelect(Disable);
	delay_cycle(1);

}

/////////////////////////N25q256使用/////////////////////////////
/////////////////////////N25q256使用/////////////////////////////
void FLASH_Read_4Byte(unsigned char addr1,unsigned char addr2, unsigned char addr3,unsigned char addr4, unsigned char *buffer, unsigned int len)  //FLASH-写入LEN个字节，LEN<=256
{
	//调用此函数先确认已开启4地址(查看标志位）
	unsigned int i;

	FLASH2_CSSelect( Enable);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = FLASH_CMD_ENTER_4BYTEADDRESS_MODE;  //进入4地址
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	FLASH2_CSSelect(Disable);

	FLASH2_CSSelect(Enable);
	
	//SPI0DAT  = FLASH_CMD_READ;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = FLASH_CMD_4BYTE_READ;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = addr1;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = addr2;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = addr3;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = addr4;
	for(i=0;i<len;i++)
	{
		//buffer[i]= FLASH2_ReadByte();
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI1->DR = 0;      //读状态寄存器
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		buffer[i]= SPI1->DR;
	}
	delay_cycle (2);
	FLASH2_CSSelect(Disable);
}


void FLASH_Write_4Byte(unsigned char addr1,unsigned char addr2, unsigned char addr3,unsigned char addr4, unsigned char *buffer, unsigned int len)  //FLASH-写入LEN个字节，LEN<=256
{
	//调用此函数先确认已开启4地址(查看标志位）
	//调用此函数先确认写使能被打开(查看标志位）
	/*
	   FLASH1CS = 0;
	   FLASH1CS = 0;
	   SPI0DAT  = FLASH_CMD_WENBLE;  //写入使能         
	   while (!SPIF);                       
	   SPIF     = 0;
	   globle8_temp = SPI0DAT;
	   FLASH1CS = 1;
	   FLASH1CS = 1;
	   Delay_ms(100);

	   FLASH1CS = 0;
	   FLASH1CS = 0;
	   SPI0DAT  = FLASH_CMD_ENTER_4BYTEADDRESS_MODE;  //进入4地址         
	   while (!SPIF);                       
	   SPIF     = 0;
	   globle8_temp = SPI0DAT;
	   FLASH1CS = 1;
	   FLASH1CS = 1;
	   Delay_ms(100);
	 */
	unsigned int i;

	//FLASH2_IsBusy(1); //判断是否在写  擦除

	FLASH2_CSSelect( Enable);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = FLASH_CMD_ENTER_4BYTEADDRESS_MODE;  //进入4地址
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	delay_cycle(2);
	FLASH2_CSSelect(Disable);

	FLASH2_CSSelect( Enable);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = FLASH_CMD_WREN;     //写使能 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	delay_cycle(2);
	FLASH2_CSSelect(Disable);
	/*
	   FLASH1CS = 0;
	   FLASH1CS = 0;
	   SPI0DAT  = FLASH_CMD_RDSTATUSR;  //读状态       
	   while (!SPIF);                       
	   SPIF     = 0;
	   globle8_temp = SPI0DAT;

	   SPI0DAT  = FLASH_CMD_RDSTATUSR;            
	   while (!SPIF);                        
	   SPIF     = 0;
	   globle8_temp = SPI0DAT;
	   FLASH1CS = 1;
	   FLASH1CS = 1;

	   FLASH1CS = 0;
	   FLASH1CS = 0;
	   SPI0DAT  = FLASH_CMD_RDFLAGSTATUSR;  //读标志       
	   while (!SPIF);                       
	   SPIF     = 0;
	   globle8_temp = SPI0DAT;

	   SPI0DAT  = FLASH_CMD_RDFLAGSTATUSR;            
	   while (!SPIF);                        
	   SPIF     = 0;
	   globle8_temp = SPI0DAT;
	   FLASH1CS = 1;
	   FLASH1CS = 1;
	 */
	FLASH2_CSSelect( Enable);

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = FLASH_CMD_PP;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = addr1;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = addr2;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = addr3;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = addr4;
	for(i=0;i<len;i++)
	{
		//FLASH2_WriteByte(buffer[i]);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI1->DR  = buffer[i];
	}

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	delay_cycle (2);
	FLASH2_CSSelect(Disable);

	FLASH2_IsBusy(1); //判断是否在写  擦除

}

void  FLASH_ERASE4K_4Byte(unsigned char addr1,unsigned char addr2, unsigned char addr3,unsigned char addr4)//按页擦除部分flash  每次擦擦4K
{
	FLASH2_CSSelect( Enable);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = FLASH_CMD_WREN  ;     //写使能 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	delay_cycle (2);
	FLASH2_CSSelect(Disable);

	FLASH2_CSSelect( Enable);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = FLASH_CMD_PP;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = addr1;  
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = addr2; 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = addr3;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR  = addr4;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	delay_cycle (2);     
	FLASH2_CSSelect(Disable);

	///-------指导擦除结束------------------	 
	FLASH2_IsBusy(1); 
	//FLASH2_IsWriteEnble(1);          
}

//延时---------------------------------------------------------------------------------
void  delay_cycle(unsigned int cycles)
{
	u32 i;
	i = cycles;
	while(i--);
}

