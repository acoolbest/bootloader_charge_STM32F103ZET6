//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*******************************************
#include "stm32f10x.h"
/***********************************************************************
文件名称：dwgl_chipflash.C
功    能：完成对片上存储的操作
编写时间：2012.11.22
编 写 人：
注    意：本例程是通过判断两个特定的字符来确定一帧数据是否结束的。
***********************************************************************/
 
//---------------------------------------------------------------------------------
u16 base_add;
unsigned char APP_EN;
unsigned char KEEP_EN = 0;
unsigned char PRO_State;
iapfun jump2app; 

void CHIP_PRO(void)
{
	unsigned int i,j;
	unsigned int f_size,f_addr,chip_addr;
	#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
	unsigned int f_size_temp,f_addr_temp;
	#endif
	unsigned short int *p;
	FLASH_Status temp;
	
	FLASH2_GPIOSPI_Read (Addr_info2, &info2STR.head[0], sizeof(info2STR));  //媒休初始化
	if(info2STR.item21_data[2] ==1)
	{
	FLASH2_GPIOSPI_Read (Addr_01min, str_buffer, 256);
	}
	else
	if(info2STR.item21_data[2] ==2)
	{
	FLASH2_GPIOSPI_Read (Addr_02min, str_buffer, 256);
	}
	else
	if(info2STR.item21_data[2] ==3)
	{
	FLASH2_GPIOSPI_Read (Addr_03min, str_buffer, 256);
	}
	else
	if(info2STR.item21_data[2] ==4)
	{
	FLASH2_GPIOSPI_Read (Addr_04min, str_buffer, 256);
	}
	else
	if(info2STR.item21_data[2] ==5)
	{
	FLASH2_GPIOSPI_Read (Addr_05min, str_buffer, 256);
	}
				f_addr = 0;
				f_size = 0;
	if(str_buffer[1]>13)
		{	
			str_buffer[1] = 13;	
		}
		
		for(i=0;i<str_buffer[1];i++)
		{
		if(str_buffer[0]!=frame_headerC)	
			{		break;	}			
				if(str_buffer[4+18*i+1]==info2STR.item21_data[3])//确认文件号
				{
				f_size = str_buffer[4+18*i+2];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+3];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+4];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+5];	
					
				f_addr = str_buffer[4+18*i+14];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+15];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+16];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+17];
				break;
				}
		}
		#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
		f_size_temp	= 	f_size;
		f_addr_temp	= 	f_addr;
		#endif
	if(f_size>0)
		{			
     FLASH_Unlock();  //FLASH解锁
		 FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//?????

		p = (unsigned short int*)str_buffer;
		chip_addr = FLASH_APP1_ADDR;    //偏移64K
//		f_size >>=1;
		for(i=0;i<=f_size;)   //擦除页=2K
			{
			while(1)
				{
				 temp=FLASH_ErasePage(chip_addr+i);     //???????
					if(temp==FLASH_COMPLETE)
					{break;}
				}
			i+=0x800;
			}

		i=0;
		while(f_size)
		{	
		 if((chip_addr&0xfffff800)==chip_addr)
		 {
//		 FLASH2_RAMSPI_Read (f_addr+2*i, str_buffer, 2048);   
 		 FLASH2_GPIOSPI_Read (f_addr+2*i, str_buffer, 2048);  //媒休初始化
			j=0;
		 }		 
		 temp =FLASH_BUSY;
			while(temp!=FLASH_COMPLETE)
			{
     temp = FLASH_ProgramHalfWord(chip_addr,p[j]); //?????addr?????
			}
     FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//?????
		 
		i++;
		j++;
		chip_addr+=2;
		f_size -=2;
		}
		FLASH_Lock();    //??FLASH???????
		
		#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
		//校验更新		
		chip_addr = FLASH_APP1_ADDR;    //偏移64K
		i=0;
		while(f_size_temp)
		{	
		 if((chip_addr&0xfffff800)==chip_addr)
		 {
//		 FLASH2_RAMSPI_Read (f_addr+2*i, str_buffer, 2048);   
 		 FLASH2_GPIOSPI_Read (f_addr_temp+2*i, str_buffer, 2048);  //媒休初始化
			j=0;
		 }	
		 
		 if(p[j]!=(*(vu16*)(chip_addr)))    //
		 {
			  FLASH_Unlock();  //FLASH解锁
			  FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//?????
				while(1)
					{
					 temp=FLASH_ErasePage(FLASH_APP1_ADDR);     //擦除用户程序
						if(temp==FLASH_COMPLETE)
						{break;}
					}
			  FLASH_Lock();    //FLASH上锁					
				break;
		 }
		 		 
		i++;
		j++;
		chip_addr+=2;
		f_size_temp -=2;
		}
				
		if(f_size_temp==0)
		{
				UART_BUFFER[0] = 'C';
				UART_BUFFER[1] = 'h';
				UART_BUFFER[2] = 'e';
				UART_BUFFER[3] = 'c';
				UART_BUFFER[4] = 'k';
				UART_BUFFER[5] = ' ';
				UART_BUFFER[6] = 'T';
				UART_BUFFER[7] = 0;
				tft_DisplayStr(80, (240-8*7)/2, UART_BUFFER,POINT_COLOR, BACK_COLOR,3);
		}
		else
		{
				UART_BUFFER[0] = 'C';
				UART_BUFFER[1] = 'h';
				UART_BUFFER[2] = 'e';
				UART_BUFFER[3] = 'c';
				UART_BUFFER[4] = 'k';
				UART_BUFFER[5] = ' ';
				UART_BUFFER[6] = 'F';
				UART_BUFFER[7] = 0;
				tft_DisplayStr(80, (240-8*7)/2, UART_BUFFER,POINT_COLOR, BACK_COLOR,3);
		}
		#else
		i = 1000;
		while(i--);
		#endif	
	 }


// /* 锁定FLASH  */
// 　FLASH_Lock();
//   /*解锁FLASH*/
// 　FLASH_Unlock();
//   /* 清除所有挂起标志位 */
// 　 FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);        
//   /* 擦除FLASH 页*/
// 　for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
// 　　  {
// 　　    FLASHStatus = FLASH_ErasePage(WRITE_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter));
// 　　  }
//   /* 写入FLASH  */
// 　Address = WRITE_START_ADDR;
// 　while((Address < WRITE_END_ADDR) && (FLASHStatus == FLASH_COMPLETE))
// 　　  {
// 　　    FLASHStatus = FLASH_ProgramWord(Address, Data);
// 　　    Address = Address + 4;
// 　　  }
// /* 锁定FLASH  */
// 　FLASH_Lock();

}

//------------------------------------------------------
void Check_CHIP_PRO(void)   //检查更新情况
{
 	unsigned char temp = 0;
 	unsigned char  i= 0;
	APP_EN = 0;		
	FLASH2_GPIOSPI_Read (Addr_info2, &info2STR.head[0], sizeof(info2STR));  //媒休初始化
	
	if((info2STR.item21[0]==21)&&(info2STR.item21[1]==4))  //
	{
		if(device.Version[0]==Version_FLAG1)
		{
			if(info2STR.item21_data[1]>=3||info2STR.item21_data[0]==0xE0)//更新操作执行限制,自毁
			{
				info2STR.item21[0] = 21;
				info2STR.item21[1] = 4;
				for(i=0;i<4;i++)
				{
				info2STR.item21_data[i] = 0x00;
				}				
				
				info2STR.item15[0] = 15;
				info2STR.item15[1] = 0;
				for(i=0;i<16;i++)
				{
				info2STR.item15_data[i] = 0xff;
				}				
			  FLASH2_GPIOSPI_SER(Addr_info2);  ////每次擦擦4K
				FLASH2_GPIOSPI_Write(Addr_info2, &info2STR.head[0], sizeof(info2STR));
				
			  FLASH_Unlock();  //FLASH解锁
			  FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//?????
				while(1)
					{
					 temp=FLASH_ErasePage(0x08010000);     //擦除用户程序
						if(temp==FLASH_COMPLETE)
						{break;}
					}
			  FLASH_Lock();    //??FLASH???????
				APP_EN = 0X00;
			}
			else
			if(info2STR.item21_data[0]==1)//开启更新
			{
				info2STR.item21_data[1]++; //
			  FLASH2_GPIOSPI_SER(Addr_info2);  ////每次擦擦4K
				FLASH2_GPIOSPI_Write(Addr_info2, &info2STR.head[0], sizeof(info2STR));
				CHIP_PRO();  //到更新的
				APP_EN = 0X01;
			}
			else
			if((((*(vu32*)(0X08010000))&0XFFF00000)==0x20000000) && (((*(vu32*)(0X08010004))&0XFFF00000)==0x08000000))   //堆栈合法
			{
				APP_EN = 0X01;
			}
			else
			{
				APP_EN = 0X00;
			}
		}
		else   //用户程序
		{
			if(info2STR.item21_data[0]==1)//开启更新
			{
				info2STR.item21[0] = 21;
				info2STR.item21[1] = 4;
				for(i=0;i<4;i++)
				{
				info2STR.item21_data[i] = 0x00;
				}				
			  FLASH2_GPIOSPI_SER(Addr_info2);  ////每次擦擦4K
				FLASH2_GPIOSPI_Write(Addr_info2, &info2STR.head[0], sizeof(info2STR));
			}
				APP_EN = 0X00;
		}		
	}
	else
	{
		if(device.Version[0]==Version_FLAG1)
		{
				info2STR.item21[0] = 21;
				info2STR.item21[1] = 4;
				for(i=0;i<4;i++)
				{
				info2STR.item21_data[i] = 0x00;
				}				
			  FLASH2_GPIOSPI_SER(Addr_info2);  ////每次擦擦4K
				FLASH2_GPIOSPI_Write(Addr_info2, &info2STR.head[0], sizeof(info2STR));
			
			if((((*(vu32*)(0X08010000))&0XFFF00000)==0x20000000) && (((*(vu32*)(0X08010004))&0XFFF00000)==0x08000000))   //堆栈合法
			{
				APP_EN = 0X01;
			}
			else
			{
				APP_EN = 0X00;
			}
		}
		else   //用户程序
		{
			if(info2STR.item21_data[0]==1)//开启更新
			{
				info2STR.item21[0] = 21;
				info2STR.item21[1] = 4;
				for(i=0;i<4;i++)
				{
				info2STR.item21_data[i] = 0x00;
				}				
			  FLASH2_GPIOSPI_SER(Addr_info2);  ////每次擦擦4K
				FLASH2_GPIOSPI_Write(Addr_info2, &info2STR.head[0], sizeof(info2STR));
			}
				APP_EN = 0X00;
		}
	}
	
}
//-------------------------------------------
void Goto_APP(u32 appxaddr)  //跳转到用户程序
{
 	unsigned char  en= 0;
	if(((*(vu32*)(appxaddr+4))&0xFFF00000)==0x08000000)   //
	{
		en |= 0x01; 
	}
	if(((*(vu32*)appxaddr)&0xFFF00000)==0x20000000)   //检查栈顶地址是否合法.
	{
		en |= 0x02; 
	}
	if(en ==3)	//
	{ 
		jump2app=(iapfun)*(vu32*)(appxaddr+4);		//用户代码区第二个字为程序开始地址(复位地址)		
		MSR_MSP(*(vu32*)appxaddr);					//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
		jump2app();									//跳转到APP.
	}
	else
	{
		FLASH2_GPIOSPI_Read (Addr_info2, &info2STR.head[0], sizeof(info2STR));  //媒休初始化
		if(info2STR.item15[1]!=0)
		{
		info2STR.item15[0] = 15;
		info2STR.item15[1] = 0;   //版本写无
		FLASH2_GPIOSPI_SER(Addr_info2);  ////每次擦擦4K
		FLASH2_GPIOSPI_Write(Addr_info2, &info2STR.head[0], sizeof(info2STR));
		}
	}

}
//------------------------------------------
__asm void MSR_MSP(u32 addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}
void Get_PRO_State(void)
{
 	unsigned char  temp= 0;
 	unsigned int   aa,bb;
	PRO_State = 0;
	
	FLASH2_GPIOSPI_Read (Addr_info2, &info2STR.head[0], sizeof(info2STR));  //媒休初始化
	if(info2STR.item21_data[0]==1)//开启更新
	{
		PRO_State |= 0x01; 
	}
	if((info2STR.item15[1]<=16)&&(info2STR.item15[1]>0))//有镜像
	{
		PRO_State |= 0x02; 
	}
		aa = ((*(vu32*)(0X08000000)));
		bb = ((*(vu32*)(0X08000004)));
		aa = ((*(vu32*)(0X08010000)));
		bb = ((*(vu32*)(0X08010004)));
	if((aa&0XFFF00000)==0x20000000&&(bb&0XFFF00000)==0x08000000)   //有程序
	{
		PRO_State |= 0x04; 
	}
	if(KEEP_EN==0xff)
	{
		PRO_State |= 0x08; 
	}
	if(info2STR.item21_data[1]>0)   //更新失败
	{
		temp = info2STR.item21_data[1]&0x0f;
		temp <<=4;
		PRO_State |= temp; 
	}
	
	UART_BUFFER[0] = 'B';
	UART_BUFFER[1] = 'L';
	UART_BUFFER[2] = '=';
	UART_BUFFER[3] = (((PRO_State)&0X01)+'0');
	UART_BUFFER[4] = (((PRO_State>>1)&0X01)+'0');
	UART_BUFFER[5] = (((PRO_State>>2)&0X01)+'0');
	UART_BUFFER[6] = (((PRO_State>>3)&0X01)+'0');
	UART_BUFFER[7] = (((PRO_State>>4)&0X07)+'0');
	UART_BUFFER[8] = 0;
	tft_DisplayStr(120, (240-8*8)/2, UART_BUFFER,POINT_COLOR, BACK_COLOR,3);
	if(info2STR.item3[0] == 3 && info2STR.item3[1]==2)//项3有效
		{
		}
		else
		if(device.Version[0]==Version_FLAG1)//
		{
			info2STR.item3_data[0] =  0x01;  //开启提示
			info2STR.item3_data[1] =  0x01;
		}
		
			
	tft_cs_enable(3);
	LCD_WindowMax(289,0,289,239);	           //设置窗口	dwgl	竖屏1	
	LCD_WriteRAM_Prepare();  		//开始写入GRAM  
	aa = 240;
	bb = (time_sys);
	while(aa)
	{
		aa--;
		LCD_WE_RAM(bb);                             //画字体颜色 一个点
	}			
	LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
	tft_cs_disable(3);
}
void Run_Logo(void)
{
 	unsigned int   X,Y;
 	u16  i,temp;
 	unsigned int   len,wide;
	
		i=100;
		while(i--);			
	X=290;
	Y=40;
//	base_add+=0x0841;
	base_add+=1;
	tft_cs_enable(3);
	for(len=0;len<20;len++)
	{
		for(wide=0;wide<20;wide++)
		{
		LCD_SetCursor(len+X,wide+Y);           //设置光标位置  
		i=10;
		while(i--);			
		LCD_WR_REG(0x2C);  			//开始写入GRAM 
		i=10;
		while(i--);
//		temp = 	(((len*len+)+wide*wide)|base_add);	
		temp = 	((len+base_add)*(len+base_add)+(wide+base_add)*(wide+base_add));	
		LCD_WE_RAM(temp); 
		i=10;
		while(i--);			
		}
	}
	tft_cs_disable(3);
	
}
