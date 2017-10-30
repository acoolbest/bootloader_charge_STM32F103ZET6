/**
 ******************************************************************************
 * @file    main.c
 * @author  fire
 * @version V1.0
 * @date    2013-xx-xx
 * @brief   液晶触摸画板实验
 ******************************************************************************
 * @attention
 *
 * 实验平台:野火 ISO-MINI STM32 开发板 
 * 论坛    :http://www.chuxue123.com
 * 淘宝    :http://firestm32.taobao.com
 *
 ******************************************************************************
 */ 

#include "stm32f10x.h"

void uart1_cmd(void)
{
	u16 last_i;
	u8 last_d;
	
	if(UART1_Error==1)			//接收满
	{			
		UART1_Error = 0;
	}
	if(UART2_Error==1)			//接收满
	{
		#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
		UART2_Error = 0;
		#endif
	}
	
	UART1_Receive_Length = UART1_RXBUFFE_LAST - UART1_RXBUFFE_HEAD;
	UART1_Receive_Length &= UART1_RX_MAX;//最大字节
	/********定义的字符作为一帧数据的结束标识************/
	if(UART1_Receive_Length > 0 )	//只有接收到1个数据以上才做判断
	{
		if(UART1_RXBUFFER[UART1_RXBUFFE_HEAD] == frame_headerC) 	//帧起始标志   
		{
			UART1_RX_State = 1;
			if((UART1_Receive_Length >= 2)&&((UART1_Receive_Pointer[(UART1_RXBUFFE_HEAD+1)&UART1_RX_MAX]<<1) <= UART1_Receive_Length)) 	//长度刚好标志   
			{
				UART1_RX_State |= 2;
				last_i = UART1_RXBUFFE_HEAD+(UART1_Receive_Pointer[(UART1_RXBUFFE_HEAD+1)&UART1_RX_MAX]<<1)-1;
				last_i &= UART1_RX_MAX;
				last_d = UART1_Receive_Pointer[last_i];
				if(last_d == frame_last)//校验对上
				{
					testcmd1_time = time_s;
					if((UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+2)&UART1_RX_MAX] ==device.addr) || (UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+2)&UART1_RX_MAX] ==GLOBLE_ADDR)) 
					{			
						switch(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX])
						{    		
							//case 0x51:  cmd_Device_Info();				break;//获取设备信息
							#ifndef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
							case 0x53:  cmd_Port_Info();					break;//获取端口信息
							case 0x54:  cmd_Get_charge_speed();				break;//获取充电速度
							case 0x55:  cmd_Device_Check();					break;//核对信息
							#endif

							case 0x57:  cmd_Device_num();					break;//设备号

							#ifndef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
							case 0x59:  cmd_Power_off();					break;//断电命令
							case 0x5a:  cmd_Power_on();						break;//上电命令
							#endif

							case 0x10:  cmd_Hub_Rst();						break;//复位HUB		 
							case 0x11:  cmd_File_Requst();					break;//文件操作请求
							case 0x12:  cmd_File_Tx();						break;//文件传输
							case 0x13:  cmd_File_Recall();					break;//文件调用
							case 0x14:  cmd_File_Erase();					break;//文件擦除
							case 0x16:  cmd_ShakeHands();					break;//握手
							case 0x30:  cmd_MediaCtrl();					break;//媒体控制命令
							//case 0x32:  cmd_Set_MediaV();					break;//媒体控制命令
							//case 0x33:  cmd_Get_MediaV();					break;//媒体控制命令
							case 0x32:  cmd_Set_Version();					break;//设置版本
							case 0x33:  cmd_Get_Version();					break;//获取版本

							#ifndef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
							case 0x34:  cmd_Get_AD_count();					break;//获取广告计数
							#endif
							
							case 0x3E:  Device_Rst();						break;//复位设备

							//case 0xE1:  cmd_Get_State();					break;//读HUB号到
							//case 0xE2:  cmd_Set_State();					break;//设置HUB号到FLASH
							case 0xE3:  cmd_Erase_Flash();					break;//
							case 0xE4:  cmd_Read_Flash();					break;//
							case 0xE5:  cmd_Write_Flash();					break;//						
							case 0xE6:  cmd_Get_ADC();						break;//						
							case 0xE7:  cmd_Save_ADC();						break;//保存ADC基线
							case 0xE8:  cmd_RGB888_565();					break;//
							case 0xE9:  cmd_RGB_clear();					break;//
							case 0xEA:  cmd_CHIP_PRO();						break;//
							case 0xEB:  cmd_PRO_Version();					break;//

							default:										break;		   	
						}//end switch
					}
					else
						if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+2)&UART1_RX_MAX] ==Broadcast)
						{
							switch(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX])
							{    		
								//case 0x10:  cmd_Hub_Rst();				break;//复位HUB		 
								//case 0x51:  cmd_device_info();			break;//获取设备信息

								default:									break;		   	
							}//end switch
						}
						else
							if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX] ==PC_ADDR)
							{
								switch(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX])
								{    		
									case 0x10:  cmd_Hub_Rst();				break;//复位HUB		 
									//case 0x51:  cmd_device_info();		break;//获取设备信息

									default:								break;		   	
								}//end switch
							}
				}
				else//校验对不上
				{
					UART1_RX_State |= 0xe0;
				}
			}						
			else
			{
				if( (time_sys -time_uart1)>100 )
				{
					UART1_RX_State |= 0xe0;						
					//BUS_Error_Byte(0x11);
				}
				else
				{
					UART1_RX_State = 0x00;
				}
			}

		}
		else
		{
			UART1_RX_State |= 0xe0;
		}

		if((UART1_RX_State &0xe0)== 0xe0)	//接收出错
		{
			UART1_RXBUFFE_HEAD +=1;
			UART1_RXBUFFE_HEAD &= UART1_RX_MAX;//最大字节
		}
		else if(UART1_RX_State ==0)   //等待接收完成
		{
			//UART1_RXBUFFE_HEAD +=0;
			//UART1_RXBUFFE_HEAD &= UART1_RX_MAX;//最大字节
		}
		else//接收完成
		{
			UART1_RXBUFFE_HEAD += (UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+1)&UART1_RX_MAX]<<1);
			UART1_RXBUFFE_HEAD &= UART1_RX_MAX;//最大字节
		}
		UART1_RX_State =0;
	}   //len >0

} 
void uart3_cmd(void)
{
	UART3_Receive_Length = UART3_RXBUFFE_LAST - UART3_RXBUFFE_HEAD;
	UART3_Receive_Length &= UART3_RX_MAX;//最大字节
	/********定义的字符作为一帧数据的结束标识************/
	if(UART3_Receive_Length > 0 )	//只有接收到1个数据以上才做判断
	{
		if(UART3_RXBUFFER[UART3_RXBUFFE_HEAD] == frame_headerC) 	//帧起始标志   
		{
			UART3_RX_State = 1;

			if((UART3_Receive_Length >= 2)&&((UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+1)&UART3_RX_MAX]) <= (UART3_Receive_Length>>1))) 	//长度刚好标志   
			{
				UART3_RX_State |= 2;
				if((UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+2)&UART3_RX_MAX] ==device.addr) || (UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+2)&UART3_RX_MAX] ==GLOBLE_ADDR)) 
				{			
					switch(UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+4)&UART3_RX_MAX])
					{    		
						//case 0x10:  cmd_Hub_Ctrl();						break;//复位		 
						//case 0x51:  charge_device_info();					break;//获取设备信息
						//case 0x16:  Test_device();						break;//
						case 0x16:  cmd3_ShakeHands();						break;//握手

						case 0xE1:  cmd3_Get_State();						break;//读HUB号到
						case 0xE2:  cmd3_Set_State();						break;//  设置HUB号到FLASH
						//case 0xE3:  cmd_Erase_Flash();					break;//
						//case 0xE4:  cmd_Read_Flash();						break;//
						//case 0xE5:  cmd_Write_Flash();					break;//

						default:											break;
					}//end switch
				}
				else
					if(UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+4)&UART3_RX_MAX] ==0xe2)
					{
						//case 0xE2:  
						cmd3_Set_State();//设置HUB号到FLASH
					}
				UART3_RXBUFFE_HEAD += (UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+1)&UART3_RX_MAX]<<1);
				UART3_RXBUFFE_HEAD &= UART3_RX_MAX;//最大字节
			}
		}
		else
		{
			UART3_RX_State = 0;
			UART3_RXBUFFE_HEAD +=1;
			UART3_RXBUFFE_HEAD &= UART3_RX_MAX;//最大字节
		}

	}
} 

void init_base_data(void)
{
	u8 i = 0;
	device.Version[1] = '1'; //硬件功能号 ='1'，ctrl;='2',HUB.
	device.Version[2] = '2'; //硬件版本号
	device.Version[3] = '.';
	
	device.Version[6] = '.';
	
	#ifdef N_VERSION_SOURCECODE//ZHZQ_CHANGE
	device.Version[0] = 'N'; //引导号 BL

	device.Version[4] = '1'; //2017年 写年份的最后两位0-99,4位表一位十进制
	device.Version[5] = '7'; //2017年 
	
	device.Version[7] = '1'; //5月第2版    //Version[1]高四位是月份。低四位是当月产生的版本。
	device.Version[8] = '0'; //5月第2版
	#elif BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
	device.Version[0] = 'B'; //引导号 BL

	device.Version[4] = '1'; //2017年 写年份的最后两位0-99,4位表一位十进制
	device.Version[5] = '7'; //2017年 

	device.Version[7] = '1'; //5月第2版    //Version[1]高四位是月份。低四位是当月产生的版本。
	device.Version[8] = '0'; //5月第2版
	#else
	device.Version[0] = 'A'; //引导号 APP

	device.Version[4] = '1'; //2017年 写年份的最后两位0-99,4位表一位十进制
	device.Version[5] = '7'; //2017年 

	device.Version[7] = '1'; //5月第2版    //Version[1]高四位是月份。低四位是当月产生的版本。
	device.Version[8] = '0'; //5月第2版
	#endif
	
	step =0;
	time_s = 0;
	time_sys = 0;
	device.TASK_state =0x00;

	led_power_ctrl(LED_ALL_INDEX, LED_TURN_OFF);

	for(i=0;i<2;i++)
	{
		Delay_ms(150);	
		led_power_ctrl(LED_INDEX, LED_TURN_NEGATION);
	}
}

/**
 * @brief  Enables or disables the High Speed ADC1 and ADC3 peripheral clock.
 * @param  NewState: new state of the specified peripheral clock.
 *   This parameter can be: ENABLE or DISABLE.
 * @retval None
 */
void ADC1_3_PeriphClockCmd(FunctionalState NewState)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, NewState);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, NewState);
}

void init_LCD_background(void)
{
	ADC1_3_PeriphClockCmd(DISABLE);
	
	LCDC.LCDPOFFTIME[LCD1_INDEX] = 0XFF;
	LCDC.LCDPOFFTIME[LCD2_INDEX] = 0XFF;
	LCDC.PSwitch = 1;
	LCDC.SPSwitch = 1;
	LCDC.PNum = 0;
	LCDC.LCDPTimeSet = 15;  
	LCDC.LCDPTime[LCD1_INDEX] = 0;
	LCDC.LCDPTime[LCD2_INDEX] = 0;
	LCDC.LCDPID[LCD1_INDEX] =0;
	LCDC.LCDPID[LCD2_INDEX] =0;
	LCDC.LCDSPTimeSet = 60;   //屏保
	LCDC.LCDSPTime[LCD1_INDEX] = 0;
	LCDC.LCDSPTime[LCD2_INDEX] = 0;
	LCDC.LCDSPPID[LCD1_INDEX] =0;
	LCDC.LCDSPPID[LCD2_INDEX] =0;

	#ifndef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
	display_flash_BMPE (0,0,3,LCDC.LCDSPPID[LCD1_INDEX],3);//单色彩色都支持 调背景
	#endif

	ADC1_3_PeriphClockCmd(ENABLE);
}

void init_LCD_config()
{
	GPIO_ResetBits(LCD_RST_PORT, LCD_RST_PIN);
	Delay_ms(200);	
	GPIO_SetBits(LCD_RST_PORT, LCD_RST_PIN);
	Delay_ms(300);	
	LCD_Init(); 
	LCD_Init1();
	
	#ifndef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
	Delay_ms(10);
	LCD_Init();
	LCD_Init1();
	#endif
	
	GPIO_ResetBits(LCD_CS1_PORT, LCD_CS1_PIN);
	GPIO_ResetBits(LCD_CS2_PORT, LCD_CS2_PIN);
	
	#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
	LCD_Clear(RED);
	Delay_ms(500);	
	LCD_Clear(GREEN);
	Delay_ms(500);
	#else
	Delay_ms(1);
	#endif
	
	LCD_Clear(BLUE);
	//version		
	Version_display(290,device.Version);
	
	#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
	Delay_ms(100); 	
	#else
	Delay_ms(2000);
	#endif
	
	//tft_16bitdeep_BMP (220,0,gImage11_240_100_16bitC);		//原LOGO
	GPIO_SetBits(LCD_CS1_PORT, LCD_CS1_PIN);
	GPIO_SetBits(LCD_CS2_PORT, LCD_CS2_PIN);
}

static void init_all_local_data_and_display_on_LCD(void)
{
	u32 i = 0;
	u8 flag = 0;
	FLASH2_GPIOSPI_Read (Addr_04min, str_buffer, 64);  //读取图片张数
	if(str_buffer[0] == 0x67)//项有效
	{
		LCDC.PNum = str_buffer[1];
	}
	FLASH2_GPIOSPI_Read (Addr_info2, &info2STR.head[0], sizeof(info2STR));	//媒休初始化
	if(info2STR.item1[0] == 1 && info2STR.item1[1]==3)//项有效
	{
		LCDC.PSwitch = info2STR.item1_data[0];
		LCDC.LCDPTimeSet = info2STR.item1_data[1];	
		LCDC.LCDPTimeSet <<= 8; 
		LCDC.LCDPTimeSet += info2STR.item1_data[2]; 
	}

	if(info2STR.item2[0] == 2  && info2STR.item2[1]==3)//项有效
	{
		LCDC.SPSwitch = info2STR.item2_data[0];
		LCDC.LCDSPTimeSet = info2STR.item2_data[1];
		LCDC.LCDSPTimeSet <<= 8;	
		LCDC.LCDSPTimeSet += info2STR.item2_data[2];
	}
	if(info2STR.item3[0] == 3 && info2STR.item3[1]==2)//项3有效
	{
	}
	else
	{
		info2STR.item3_data[0] =  0x00;  //关闭提示
		info2STR.item3_data[1] =  0x00;
		//info2STR.item3_data[0] =  0x01;  //开启提示
		//info2STR.item3_data[1] =  0x01;
	}
	flag = 0;
	if(info2STR.item31[0] == 31 && info2STR.item31[1]==9)//项31有效
	{
		flag = 1;
		
		#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
		for(i=0;i<9;i++)
		#else
		for(i=1;i<3;i++)
		#endif
		{
			if(info2STR.item31_data[i] != device.Version[i])
			{
				flag = 0;
				break;
			}
		}			
	}
	#if 1
	if(flag==0)  //项31不正常重启
	{	
		#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
		info2STR.item31[0] = 31;
		info2STR.item31[1] = 9;
		for(i=0;i<9;i++)
		{
			info2STR.item31_data[i] = device.Version[i];
		}
		FLASH2_GPIOSPI_SER(Addr_info2);  ////每次擦擦4K
		FLASH2_GPIOSPI_Write(Addr_info2, &info2STR.head[0], sizeof(info2STR));//写入当前版本号
		//#else
		//NVIC_SystemReset();
		#endif
	}
	#endif
	//获取设备号
	if(info2STR.item81[0] == 81 && info2STR.item81[1]<=16)//项81有效设备号
	{
		for(i=0;i<info2STR.item81[1];i++)
		{
			device_num[i]= info2STR.item81_data[i];
		}
	}
	else			
	{
		info2STR.item81[1] = 8;
		for(i=0;i<info2STR.item81[1];i++)
		{
			device_num[i]= '?';
		}
	}
	device_num[i++]= 0;
	
	#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
	tft_DisplayStr(270, 125, device_num, POINT_COLOR, BACK_COLOR, 3);
	#else
	tft_DisplayStr(270, 125, device_num, 0x0000, 0xffff, 3);
	#endif
	
	//初始时间
	for(i=0;i<2;i++)
	{
		Uport_PowerUseTime[i] = 0;
		Uport_PowerShowTime[i] = 0;
	}

	//tft_DisplayStr(52, 0, AnsiChardata,POINT_COLOR, BACK_COLOR,3);

	//display_flash_BMPE (14,118,3,((Uport_PowerUseTime[0]%3600/600)+'0'),3);//时间 分H
	//display_flash_BMPE (14,139,3,((Uport_PowerUseTime[0]%600/60)+'0'),3);//时间 分L
	//display_flash_BMPE (14,174,3,((Uport_PowerUseTime[0]%60/10)+'0'),3);//时间 秒H
	//display_flash_BMPE (14,195,3,((Uport_PowerUseTime[0]%10)+'0'),3);//时间 秒L



	//获取基线,设备号
	//Addr_info,格式：0X67 LEN ADDR F0 XX XX (16字节AD基线) (12字节设备号) CHECK 0X99
	FLASH2_GPIOSPI_Read (Addr_info1, str_buffer, 64);
	global_u8p = (u8*)ADC_Base0;
	for(i=0;i<ADC1_3_ENABLE_CHANNEL_NUM;i++)
	{
		global_u8p[i] = str_buffer[6+i];
		//global_u8p[i] = str_buffer[i];
	}
	//DisplayADC_BL(150, 0, ADC_Base0,POINT_COLOR, BACK_COLOR,1);
	//DisplayADC_BL(150, 0, &ADC_Base0[3],POINT_COLOR, BACK_COLOR,2);
	//DisplayADC_BL(150, 0, &ADC_Base0[5],POINT_COLOR, BACK_COLOR,3);
	
	#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
	Get_ADC_BaseLine();
 	DisplayADC_BL(240, (240-8*4)/2, &ADC_Base0[6],POINT_COLOR, BACK_COLOR,3);
	#endif

	//文本区初始	
	//FiletoBuffer_ID(u8 area,u8 id,u8 *p);

	#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
	FiletoBuffer_ID(2,48,LCD_TxtBuffer[LCD1_INDEX]);
	FiletoBuffer_ID(2,48,LCD_TxtBuffer[LCD2_INDEX]);
	#else
	LCD_TxtBuffer[LCD1_INDEX][0]=0;
	LCD_TxtBuffer[LCD2_INDEX][0]=0;
	#endif

	LCD_TxtBuffer[LCD1_INDEX][2048]=0;
	LCD_TxtBuffer[LCD1_INDEX][2049]=0;
	LCD_TxtBuffer[LCD2_INDEX][2048]=0;
	LCD_TxtBuffer[LCD2_INDEX][2049]=0;

	//二维码初始
	FLASH2_GPIOSPI_Read (Addr_01min, str_buffer, 128);
	if(str_buffer[0]==frame_headerC)  //判断有没有二维码
	{
		device.use &= ~0x0C; //有二维码 
	}
	else
	{
		device.use |= 0x0C;//没有二维码
	}
	if((device.use&0x04)==0x0) //有码显示
	{
		#ifndef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
		DisplayPROT_EWM(80,56,0,1);  //128
		DisplayPROT_EWM(80,56,1,2);  //128
		#endif
	}
}

static void check_embeded_program_update(void)//更新检查
{
#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
	Get_PRO_State();
	Check_CHIP_PRO();	
	Get_PRO_State();
	Delay_ms(100);	
	time_sys = 2000;
#else
	Check_CHIP_PRO();
	Delay_ms(100);
	time_sys = 0;
#endif
}

void check_key_press_down_to_reset_board(void)
{
	u32 i = 0;
	if(GPIO_ReadInputDataBit(KEY_PORT,KEY_PIN) ==0)  //按键=0,1S进行人为复位
	{
		#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
		KEY_time = time_sys;
		while(GPIO_ReadInputDataBit(KEY_PORT,KEY_PIN) ==0)
		{
			led_power_ctrl(LED_INDEX, LED_TURN_ON);
			if(time_sys-KEY_time>=10000)
			{
				FLASH2_GPIOSPI_Read (Addr_info2, &info2STR.head[0], sizeof(info2STR));	//媒休初始化
				info2STR.item21[0] = 21;
				info2STR.item21[1] = 4;
				info2STR.item21_data[0] = 0xE0;  //自毁
				info2STR.item21_data[1] = 0x00;
				info2STR.item21_data[2] = 0;  
				info2STR.item21_data[3] = 0;
				FLASH2_GPIOSPI_SER(Addr_info2);  //每次擦擦4K
				FLASH2_GPIOSPI_Write(Addr_info2, &info2STR.head[0], sizeof(info2STR));
				break;
			}
		}
		#endif
		rewrite_ADC_BaseLine_flash_data();
		i=1000;
		while(i--);
		NVIC_SystemReset();
	}
}

#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
static void bootloader_check_app_program_at_regular_time(void)
{
	Get_PRO_State();
	if(KEEP_EN==0xff) return;

	if((device.Version[0]==Version_FLAG1) &&(APP_EN==1))
	{			
		UART_BUFFER[0] = 'R';
		UART_BUFFER[1] = 'u';
		UART_BUFFER[2] = 'n';
		UART_BUFFER[3] = ' ';
		UART_BUFFER[4] = 'A';
		UART_BUFFER[5] = 'P';
		UART_BUFFER[6] = 'P';
		UART_BUFFER[7] = 0;
		tft_DisplayStr(100, (240-8*7)/2, UART_BUFFER,POINT_COLOR, BACK_COLOR,3);
		Delay_ms(1000);	
		Goto_APP(FLASH_APP1_ADDR);
	}
	else
	{
		Check_CHIP_PRO();
		Get_PRO_State();
	}
}
#endif

void update_qrcode_at_regular_time(void)
{
	#ifndef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
	u8 lcd_index = 0;
	u8 lcd_cs = 0;
	u8 file_num = 0;
	#endif
	
	if(time_sys-check_time >= 5000)			//定时检测设备
	{
		#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
		bootloader_check_app_program_at_regular_time();
		#endif
		check_time = time_sys;
		led_power_ctrl(LED_INDEX, LED_TURN_ON);

		if((device.use&0x10)==0x10) //二维码有更新
		{
			device.use &= ~0x10;
			#ifndef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
			for(lcd_index=0;lcd_index<2;lcd_index++)
			{
				lcd_cs = lcd_index+1;
				file_num = lcd_index;
				if(LCDC.LCDSPPID[lcd_index]==1)
				{
					DisplayPROT_EWM(115,56,file_num,lcd_cs);  //128
					//tft_DisplayStr(15, 125, device_num,0x0000,0xffff,lcd_cs);
				}
				else if(LCDC.LCDSPPID[lcd_index]==0)
				{
					DisplayPROT_EWM(80,56,file_num,lcd_cs);  //128
					//tft_DisplayStr(270, 125, device_num,0x0000,0xffff,lcd_cs);
				}

			}
			#endif
		}
	}
}

/**
 * @brief  update LCD1 or LCD2 screen protect.
 * @param  lcd_index: specifies the LCD index.
 *   This parameter can be: LCD1_INDEX or LCD2_INDEX.
 * @retval None
 */
static void update_screen_protect(u8 lcd_index)
{
	#ifndef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
	u8 lcd_cs = lcd_index+1;
	u8 file_num = lcd_index;
	#endif
	
	if((LCDC.LCDSPTime[lcd_index]>=LCDC.LCDSPTimeSet)&&(time_sys-time_s_temp<300))//LCD更新屏保
	{
		LCDC.LCDSPTime[lcd_index] -=LCDC.LCDSPTimeSet;
		
		#ifndef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
		if(LCDC.LCDSPPID[lcd_index]!=2)
		{
			if(LCDC.LCDSPPID[lcd_index]<1)
			{
				LCDC.LCDSPPID[lcd_index]++;
			}
			else
			{
				LCDC.LCDSPPID[lcd_index] = 0;
			}
			display_flash_BMPE (0,0,3,LCDC.LCDSPPID[lcd_index],lcd_cs);//单色彩色都支持 调背景
			if(LCDC.LCDSPPID[lcd_index]==1)
			{
				DisplayPROT_EWM(115,56,file_num,lcd_cs);  //128
				tft_DisplayStr(15, 125, device_num,0x0000,0xffff,lcd_cs);
			}
			else
			{
				DisplayPROT_EWM(80,56,file_num,lcd_cs);  //128
				tft_DisplayStr(270, 125, device_num,0x0000,0xffff,lcd_cs);
			}
		}
		#endif
	}
}

void update_LCD1_screen_protect(void)
{
	update_screen_protect(LCD1_INDEX);
}

void update_LCD2_screen_protect(void)
{
	update_screen_protect(LCD2_INDEX);
}

void update_advertisement()
{
	u8 i = 0;
	u8 lcd_index = 0;
	u8 lcd_cs = 0;
	for(i=0;i<2;i++)
	{
		lcd_index = i;
		lcd_cs = lcd_index+1;
		if((LCDC.LCDPTime[lcd_index]>=LCDC.LCDPTimeSet)&&(time_sys-time_s_temp<300))//LCD更新广告
		{
			LCDC.LCDPTime[lcd_index] -=LCDC.LCDPTimeSet;

			if(LCDC.PSwitch==1)
			{
				LCDC.LCDPID[lcd_index]++;
				if(LCDC.LCDPID[lcd_index]>=LCDC.PNum)
				{
					LCDC.LCDPID[lcd_index]=0;
				}
				#ifndef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
				AD_count[LCDC.LCDPID[lcd_index]]++;   //广告计数
				#endif
				display_flash_BMPE (116,0,4,LCDC.LCDPID[lcd_index],lcd_cs);//广告
			}
		}
	}

}

void LCD_into_power_on_state()
{
	u8 i = 0;
	u8 lcd_index = 0;
	u8 lcd_cs = 0;
	u8 Uport_PowerUseTime_index = 0;

	for(i=0;i<2;i++)
	{
		lcd_index = i;
		lcd_cs = lcd_index+1;
		Uport_PowerUseTime_index = lcd_index;
		if((Uport_PowerUseTime[Uport_PowerUseTime_index]>0)&&(LCDC.LCDSPPID[lcd_index]!=2))//LCD进入充电
		{
			LCDC.LCDSPPID[lcd_index] = 2;
			display_flash_BMPE (0,0,3,LCDC.LCDSPPID[lcd_index],lcd_cs);//单色彩色都支持 调背景
		}
	}
}

void synthesize_function(u8 lcd_index)
{
	u8 lcd_cs = lcd_index+1;
	u8 Uport_PowerUseTime_index = lcd_index;
	u8 file_num = lcd_index;
	u8 led_index = lcd_index+1;
	u32 i = 0;
#if 1
	AnsiChardata[12] = Uport_PowerUseTime[Uport_PowerUseTime_index]/36000+'0';
	AnsiChardata[13] = Uport_PowerUseTime[Uport_PowerUseTime_index]%36000/3600+'0';
	AnsiChardata[19] = Uport_PowerUseTime[Uport_PowerUseTime_index]%3600/600+'0';
	AnsiChardata[20] = Uport_PowerUseTime[Uport_PowerUseTime_index]%600/60+'0';
	AnsiChardata[24] = Uport_PowerUseTime[Uport_PowerUseTime_index]%60/10+'0';
	AnsiChardata[25] = Uport_PowerUseTime[Uport_PowerUseTime_index]%10+'0';
#endif

	if(LCDC.LCDSPPID[lcd_index]==2)
	{
		display_flash_BMPE (18,120,3,((Uport_PowerShowTime[Uport_PowerUseTime_index]%3600/600)+'0'),lcd_cs);//时间 分H
		display_flash_BMPE (18,141,3,((Uport_PowerShowTime[Uport_PowerUseTime_index]%600/60)+'0'),lcd_cs);//时间 分L
		display_flash_BMPE (18,176,3,((Uport_PowerShowTime[Uport_PowerUseTime_index]%60/10)+'0'),lcd_cs);//时间 秒H
		display_flash_BMPE (18,197,3,((Uport_PowerShowTime[Uport_PowerUseTime_index]%10)+'0'),lcd_cs);//时间 秒L
	}

	if(Uport_PowerUseTime[Uport_PowerUseTime_index]>0)
	{
		Uport_PowerUseTime[Uport_PowerUseTime_index]--;
		//led_power_ctrl(led_index, LED_TURN_OFF);
		LCDC.LCDPOFFTIME[lcd_index]=0; //断电计时
	}
	else
	{
		//led_power_ctrl(led_index, LED_TURN_ON);
		for(i=lcd_index*3;i<(lcd_index+1)*3;i++)
		{
			usb_power_ctrl(i, USB_POWER_OFF);
			Dport_State[i] = 0;
		}
		LCDC.LCDPTime[lcd_index]=0;		//广告计时
	}

	if(Uport_PowerShowTime[Uport_PowerUseTime_index]>0)
	{
		Uport_PowerShowTime[Uport_PowerUseTime_index]--;
		led_power_ctrl(led_index, LED_TURN_ON);
	}
	else
	{
		led_power_ctrl(led_index, LED_TURN_OFF);
	}

	if(LCDC.LCDPOFFTIME[lcd_index]==1)  //断电1秒调文字
	{
		FiletoBuffer_ID(2,48,LCD_TxtBuffer[lcd_index]);
		LCD_TxtBuffer[lcd_index][2048]=0;
		LCD_TxtBuffer[lcd_index][2049]=0;
	}
	else if(LCDC.LCDPOFFTIME[lcd_index]==5)  //断电后5秒调文字
	{
		LCDC.LCDSPTime[lcd_index] = 0;  //屏保时间
		LCDC.LCDSPPID[lcd_index] = 0;
		display_flash_BMPE (0,0,3,LCDC.LCDSPPID[lcd_index],lcd_cs);//单色彩色都支持 调背景
		DisplayPROT_EWM(80,56,file_num,lcd_cs);  //128
		tft_DisplayStr(270, 125, device_num,0x0000,0xffff,lcd_cs);
		
		#ifndef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
		for(i=0;i<sizeof(gImage10_16_288_1bitC);i++)
		{
			LCD_TxtBuffer[lcd_index][i] = gImage10_16_288_1bitC[i];
		}
		LCD_TxtBuffer[lcd_index][2048]=0;
		LCD_TxtBuffer[lcd_index][2049]=0;
		#endif
	}
}

void check_hub_link_state(void)
{
	static u32 time_sys_temp;
	if(time_sys-time_sys_temp >= 2000)			//时间控制任务
	{
		time_sys_temp = time_sys;
		LCD_TEST();
		if((time_s-testcmd3_time)>=5)  //0x16 0xe2都有确认连接功能
		{
			UART_BUFFER[0] = 'U';
			//UART_BUFFER[1] = 'n';
			//UART_BUFFER[2] = 'l';
			//UART_BUFFER[3] = 'i';
			//UART_BUFFER[4] = 0;
			UART_BUFFER[1] = (device.addr>>4)+'0';
			UART_BUFFER[2] = (device.addr&0x0f)+'0';
			UART_BUFFER[3] = 0;
			if((time_s-testcmd1_time)>=30)  //30秒没收到正确命令认为断开连接
			{
				UART_BUFFER[0] = 'u';
			}
			if(info2STR.item3_data[1]==0x01)
			{
				tft_DisplayStr(0, 0, UART_BUFFER,POINT_COLOR, BACK_COLOR,3);
			}
			GPIO_ResetBits(RJ45_IO1_PORT, RJ45_IO1_PIN);
		}
		else
		{
			UART_BUFFER[0] = 'L';
			//UART_BUFFER[1] = 'i';
			//UART_BUFFER[2] = 'n';
			//UART_BUFFER[3] = 'k';
			//UART_BUFFER[4] = 0;
			UART_BUFFER[1] = (device.addr>>4)+'0';
			UART_BUFFER[2] = (device.addr&0x0f)+'0';
			UART_BUFFER[3] = 0;
			if((time_s-testcmd1_time)>=30)  //30秒没收到正确命令认为断开连接
			{
				UART_BUFFER[0] = 'l';
			}
			if(info2STR.item3_data[1]==0x01)
			{
				tft_DisplayStr(0, 0, UART_BUFFER,POINT_COLOR, BACK_COLOR,3);
			}
		}
	}
}

void check_device_plugin()
{
	u8 lcd_index = 0;

	for(lcd_index=0;lcd_index<2;lcd_index++)
	{
		if(Uport_PowerUseTime[lcd_index]>0)
		{
			port_Charge_State(lcd_index);
		}
		else
		{
			checking_port[lcd_index] = 0x00;  //SUB置空
		}
	}
}


void deal_LCD_task_at_regular_time(u8 lcd_index)
{
	u8 lcd_cs = lcd_index+1;
	static u32 time_sys_temp[2];

	if((time_sys-time_sys_temp[lcd_index] >= 1000))			//时间控制任务
	{
		time_sys_temp[lcd_index] = time_sys;
		if(LCDC.LCDSPPID[lcd_index]==2)
		{
			tft_1bitdeep_TXT (87, 0, LCD_TxtBuffer[lcd_index],POINT_COLOR, 0xffff,lcd_cs);
		}
	}
}

void deal_LCD1_task_at_regular_time(void)
{
	deal_LCD_task_at_regular_time(LCD1_INDEX);
}

void deal_LCD2_task_at_regular_time(void)
{
	deal_LCD_task_at_regular_time(LCD2_INDEX);
}

void deal_task_at_regular_time0(u8 lcd_index)//ZHZQ_CHANGE
{
	u8 i = 0;
	u8 lcd_cs = lcd_index+1;
	u8 lcd_index_ascii = lcd_index+'1';
	if(LCDC.LCDPOFFTIME[lcd_index]<0xff)	//断电计时
	{
		LCDC.LCDPOFFTIME[lcd_index]++;
	}
	LCDC.LCDPTime[lcd_index]++;		//广告计时
	LCDC.LCDSPTime[lcd_index]++;  //屏保时间

	if((checking_port[lcd_index]&0xF0)==0x20)
	{
		usb_mutually_exclusive_power_on(lcd_index);	//互斥上电
	}

	UART_BUFFER[0] = lcd_index_ascii;
	for(i=0;i<3;i++)
	{
		UART_BUFFER[1+i] = Dport_State[3*lcd_index+i]+'0';
	}
	UART_BUFFER[1+i] =0;
	if(info2STR.item3_data[0]==0x01)
	{
		tft_DisplayStr(0, 32, UART_BUFFER, POINT_COLOR, BACK_COLOR, lcd_cs);
	}
}

void deal_task_at_regular_time()
{
	if(time_sys-time_s_temp >= 1000)			//时间控制任务
	{
		time_s_temp +=1000;
		time_s++;
		#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
		if(device.Version[0]==Version_FLAG1)   
		{
			UART_BUFFER[0] = time_s/36000+'0';
			UART_BUFFER[1] = time_s%36000/3600+'0';
			UART_BUFFER[2] = time_s%3600/600+'0';
			UART_BUFFER[3] = time_s%600/60+'0';
			UART_BUFFER[4] = time_s%60/10+'0';
			UART_BUFFER[5] = time_s%10+'0';
			UART_BUFFER[6] = 'S';
			UART_BUFFER[7] = 0;
			tft_DisplayStr(270, 0, UART_BUFFER, POINT_COLOR, BACK_COLOR, 3);
		}
		#endif
		deal_task_at_regular_time0(LCD1_INDEX);
		deal_task_at_regular_time0(LCD2_INDEX);
		synthesize_function(LCD1_INDEX);
		synthesize_function(LCD2_INDEX);
	}
}

static VOID_FUNC_START_ROUTINE void_func[VOID_FUNC_COUNT] = {
	check_key_press_down_to_reset_board,
	update_qrcode_at_regular_time,
	deal_task_at_regular_time,
	deal_LCD1_task_at_regular_time,
	deal_LCD2_task_at_regular_time,
	check_hub_link_state,
	check_device_plugin,
	update_LCD1_screen_protect,
	update_LCD2_screen_protect,
	update_advertisement,
	LCD_into_power_on_state
};


/**
 * @brief  主函数
 * @param  无  
 * @retval 无
 */
int main(void)
{
	u32 i;

	/* 系统定时器 1us 定时初始化 */
	SysTick_Init();
	//CPU_CRITICAL_ENTER();
	LED_GPIO_Config();
	ADC_Configuration();
	//DAC_Configuration();
	UART_Configuration();
	SPI_Configuration();
	DMA_Configuration();
	LCD_IOConfig();
	FSMC_LCDInit();
	NVIC_Configuration();
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);//开中断
	ADC_ITConfig(ADC3, ADC_IT_EOC, ENABLE);//开中断
	//DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
	//SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
	IWDG_Init(6,0xfff);   //OPEN DOG =26S
	//CPU_CRITICAL_EXIT();

	init_base_data();

	//Addr_Set();
	//hub_id_info();

	usb_power_ctrl(USB_ALL_INDEX, USB_POWER_OFF);//初始化USB上电状态，关闭上电

	//初始LCD
	init_LCD_config();
	init_LCD_background();
	init_all_local_data_and_display_on_LCD();
	
	//更新检查
	check_embeded_program_update();
	
	while(1)
	{
		for(i=0;i<VOID_FUNC_COUNT;i++) 
		{
			void_func[i]();
			uart1_cmd();
			uart3_cmd();
		}
	}
}

/* ------------------------------------------end of file---------------------------------------- */

