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
#ifdef ZHZQ_TEST
u16 sprintf_com_cmd(u8 * sz_cmd, struct com_send_stru stru_com_send)
{
	u8 * p = sz_cmd;
	*p = SEND_COM_MSG_HEAD;
	*(p += 2) = stru_com_send.dst_addr;
	*(++p) = device.addr;
	*(++p) = stru_com_send.cmd_type;
	switch (stru_com_send.cmd_type)
	{
		case COM_CMD_GET_PORT_STATE:
			*(++p) = stru_com_send.port_num;
			*(++p) = (stru_com_send.power_set_time>>8);
			*(++p) = (stru_com_send.power_set_time&0xff);
			*(++p) = (stru_com_send.power_remaining_time>>8);
			*(++p) = (stru_com_send.power_remaining_time&0xff);
			break;
		case COM_CMD_GET_CHARGE_SPEED:
			*(++p) = stru_com_send.power_speed;
			break;
		case COM_CMD_CHECK_QR_CODE:
			*(++p) = 0x02;
			*(++p) = stru_com_send.check_port[0];
			memcpy(p+1, stru_com_send.qr_code[0], 8);
			p += 8;
			*(++p) = stru_com_send.check_port[1];
			memcpy(p+1, stru_com_send.qr_code[1], 8);
			p += 8;
			break;
		case COM_CMD_POWER_ON:
			*(++p) = stru_com_send.port_num;
			*(++p) = stru_com_send.charge_model;
			*(++p) = stru_com_send.time_ctrl;
			*(++p) = (u8)(stru_com_send.power_on_time >> 8);
			*(++p) = (u8)(stru_com_send.power_on_time);
			break;
		case COM_CMD_SET_DEVICE_NUM:
		case COM_CMD_FILE_OPERATIONS_REQ:
		case COM_CMD_FILE_CALLED:
		case COM_CMD_FILE_ERASE:
		case COM_CMD_MEDIA_TIME_CTRL:
		case COM_CMD_SET_AREA_VERSION:
		case COM_CMD_WRITE_FLASH:
		case COM_CMD_SAVE_ADC:
		case COM_CMD_RGB888_565:
		case COM_CMD_CHIP_PRO:
			*(++p) = stru_com_send.confirm_flag;
			break;
		case COM_CMD_FILE_TRANSFER:
			*(++p) = stru_com_send.transfer_flag;
			*(++p) = stru_com_send.section_num;
			*(++p) = stru_com_send.confirm_flag;
			break;
		case COM_CMD_HANDSHAKE:
			*(++p) = stru_com_send.handshake_mode;
			break;
		case COM_CMD_GET_AREA_VERSION:
		case COM_CMD_AD_DATA_STATISTICS:
		case COM_CMD_GET_EMBEDDED_VERSION:
			*(++p) = stru_com_send.item_number;
			*(++p) = stru_com_send.item_len;
			memcpy(p+1, stru_com_send.item_content, stru_com_send.item_len);
			p += stru_com_send.item_len;
			break;
		case COM_CMD_POWER_OFF:
		case COM_CMD_HUB_RESET:
		default:
			break;
	}
	//When the frame length is odd, in front of the checksum zero padding, make it into an even number.
	if((p - sz_cmd + 1) % 2) *(++p) = 0x00;
	*(p += 2) = DEFAULT_COM_MSG_TAIL;
	sz_cmd[1] = (p - sz_cmd + 1)/2;
	*(p - 1) = get_checksum(sz_cmd, sz_cmd[1]*2);
	return sz_cmd[1]*2;
}

void com_get_port_state(u8 * p_recv_data)
{
	u8 port_num_index = p_recv_data[5]%2;
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_GET_PORT_STATE,
		.dst_addr = p_recv_data[3],
		.port_num = p_recv_data[5],
		.power_set_time = Uport_PowerSetTime[port_num_index],
		.power_remaining_time = Uport_PowerUseTime[port_num_index]
	};

	u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
	UART1_Send_Data(sz_cmd,frame_len);
}
void com_get_charge_speed(u8 * p_recv_data)   //获取充电速度
{
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_GET_CHARGE_SPEED,
		.dst_addr = p_recv_data[3],
		.port_num = p_recv_data[5]
	};
	
	u8 lcd_index = p_recv_data[5]%2;
	u8 i_min = lcd_index*3;
	u8 i_max = lcd_index*3+3;
	
	u16 ADC_data[ADC1_3_ENABLE_CHANNEL_NUM] = {0};
	get_ADC1_3_data(ADC_data);
	
	u8 i = 0;
	for(i=i_min;i<i_max;i++)
	{
		if((Dport_State[i]&0x0c)==0x0c)
		{
			if(ADC_data[i]-ADC_Base0[i]>0x640) stru_com_send.power_speed = 3;			//0x12C0=800MA	0x640 = 0x12C0/3
			else if(ADC_data[i]-ADC_Base0[i]>0x455) stru_com_send.power_speed = 2;		//0XD00=560MA		0x455 = 0XD00/3
			else if(ADC_data[i]-ADC_Base0[i]>0xAA) stru_com_send.power_speed = 1;		//0x200=80MA		0xAA = 0x200/3
			else stru_com_send.power_speed = 0;
			break;
		}
	}

	u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
	UART1_Send_Data(sz_cmd,frame_len);
}

void com_check_qr_code(u8 * p_recv_data)//同时对两个端口的二维码进行核对
{
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_CHECK_QR_CODE,
		.dst_addr = p_recv_data[3],
		.port_num = p_recv_data[5]
	};

	FLASH2_GPIOSPI_Read (Addr_01min, str_buffer, 256);

	if(str_buffer[0]==frame_headerC)
	{
		u8 sum_port_count = 0;
		u8 i = 0;
		for(sum_port_count = 0; sum_port_count<2;sum_port_count++)
		{
			if(str_buffer[1]>sum_port_count)
			{
				stru_com_send.check_port[sum_port_count] = str_buffer[5+18*sum_port_count];
				for(i=0;i<8;i++) stru_com_send.qr_code[sum_port_count][i] = str_buffer[10+i+18*sum_port_count];
			}
		}
	}

	u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
	UART1_Send_Data(sz_cmd,frame_len);
}

void com_set_device_num(u8 * p_recv_data)
{
	u8 i,en,temp;
	u8 lcd_index = 0;
	u8 lcd_cs = 0;
	unsigned int display_x = 0, display_y = 0;
	en = 0x0;
	FLASH2_GPIOSPI_Read (Addr_info2, &info2STR.head[0], sizeof(info2STR));  //媒休初始化
	temp = info2STR.item81[1];
	if(temp>16)
	{		
		temp = 16;
	}
	else if(temp==0)
	{	
		temp = 8;
	}

	if(p_recv_data[5]==81)  //项目号
	{
		en = 0xff;
		for(i=0;i<(p_recv_data[6]+2);i++)  //判断要不要重写
		{
			if(info2STR.item81[i] != p_recv_data[5+i])
			{
				en = 0;
			}
		}

		if(en == 0)  //要重写
		{				
			info2STR.head[0] = frame_headerC;
			info2STR.head[1] = sizeof(info2STR); 
			info2STR.head[1] >>=1;
			info2STR.temp[0] = p_recv_data[3];
			info2STR.temp[1] = device.addr;
			info2STR.temp[2] = 0xff;//项目数不确定值

			for(i=0;i<18;i++)
			{
				info2STR.item81[i] = p_recv_data[5+i];
			}
			FLASH2_GPIOSPI_SER(Addr_info2);  ////每次擦擦4K
			FLASH2_GPIOSPI_Write(Addr_info2, &info2STR.head[0], sizeof(info2STR));
		}
		for(i=0;i<sizeof(info2STR);i++)
		{
			str_buffer[i] =info2STR.head[i];
		}

		en = 0xff;
		FLASH2_GPIOSPI_Read (Addr_info2, &str_buffer[1024], sizeof(info2STR));			
		for(i=0;i<sizeof(info2STR);i++)
		{
			if(str_buffer[i]!=str_buffer[1024+i])
			{
				en = 0;
			}
		}
	}

	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_SET_DEVICE_NUM,
		.dst_addr = p_recv_data[3],
		.port_num = p_recv_data[5]
	};
	stru_com_send.confirm_flag = en;
	
	u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
	UART1_Send_Data(sz_cmd,frame_len);

	if(info2STR.item81[1]<=16)
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

	for(lcd_index=0;lcd_index<2;lcd_index++)
	{
		lcd_cs = lcd_index+1;

		for(i=0;i<temp;i++) UART_BUFFER[i] = ' ';
		UART_BUFFER[i] = 0;

		if(LCDC.LCDSPPID[lcd_index]==1) display_x = 15;
		else if(LCDC.LCDSPPID[lcd_index]==0) display_x = 270;
		display_y = 125;

		tft_DisplayStr(display_x, display_y, UART_BUFFER, POINT_COLOR, BACK_COLOR, lcd_cs);
		#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
		tft_DisplayStr(display_x, display_y, device_num, POINT_COLOR, BACK_COLOR, lcd_cs);
		#else
		tft_DisplayStr(display_x, display_y, device_num, 0x0000, 0xffff, lcd_cs);
		#endif
	}
}


void com_power_off(u8 * p_recv_data)
{
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_POWER_OFF,
		.dst_addr = p_recv_data[3],
		.port_num = p_recv_data[5]
	};
	u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
	
	u8 lcd_index = p_recv_data[5]%2;
	u8 i = 0;
	
	Uport_PowerSetTime[lcd_index] = 0;
	Uport_PowerUseTime[lcd_index] = 0;
	Uport_PowerShowTime[lcd_index] = 0;
	
	if(lcd_index == LCD1_INDEX)//ZHZQ_CHANGE
	{
		i= (device.addr>>4);
		i = i*2+4;
		while((time_sys-time_uart1) <i);
	}
	
	UART1_Send_Data(sz_cmd,frame_len);
}

void com_power_on(u8 * p_recv_data)
{
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_POWER_ON,
		.dst_addr = p_recv_data[3],
		.port_num = p_recv_data[5]
	};

	stru_com_send.charge_model = p_recv_data[6];
	stru_com_send.time_ctrl = p_recv_data[7];
	stru_com_send.power_on_time = (u16)p_recv_data[8]<<8 | p_recv_data[9];

	u8 lcd_index = p_recv_data[5]%2;
	
	if(stru_com_send.time_ctrl != 0)
	{
		Uport_PowerSetTime[lcd_index] = stru_com_send.power_on_time;
		Uport_PowerUseTime[lcd_index]	= stru_com_send.power_on_time;
		Uport_PowerShowTime[lcd_index] = 0;
	}

	u8 i_min = lcd_index*3;
	u8 i_max = lcd_index*3+3;
	u8 i = 0;
	
	if(stru_com_send.charge_model == 0x02) //快充 上电方式选择2
	{
		Uport_PowerShowTime[lcd_index] = Uport_PowerUseTime[lcd_index];
		for(i=i_min;i<i_max;i++) usb_power_ctrl(i, USB_POWER_OFF);
		Delay_ms(200);
		
		if(lcd_index == LCD1_INDEX) GPIO_ResetBits(EN_KC0_PORT, EN_KC0_PIN); //快充
		else GPIO_ResetBits(EN_KC1_PORT, EN_KC1_PIN); //快充
		
		checking_port[lcd_index] &= 0x0f; 
	}
	else if(stru_com_send.charge_model == 0x01) //USB 上电方式选择1
	{
		for(i=i_min;i<i_max;i++) usb_power_ctrl(i, USB_POWER_OFF);
		Delay_ms(200);

		if(lcd_index == LCD1_INDEX) GPIO_SetBits(EN_KC0_PORT, EN_KC0_PIN); //USB上电方式
		else GPIO_SetBits(EN_KC1_PORT, EN_KC1_PIN); //USB上电方式

		checking_port[lcd_index] &= 0x0f; 
	}

	if(lcd_index == LCD1_INDEX)//ZHZQ_CHANGE
	{
		i= (device.addr>>4);
		i = i*2+4;
		while((time_sys-time_uart1) <i);
	}

	u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
	UART1_Send_Data(sz_cmd,frame_len);

	FLASH2_GPIOSPI_Read (Addr_04min, str_buffer, 64);  //读取图片张数
	if(str_buffer[0] == 0x67)//项有效
	{
		LCDC.PNum = str_buffer[1];
	}
}

void com_hub_reset(u8 * p_recv_data)
{
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_HUB_RESET,
		.dst_addr = p_recv_data[3],
		.port_num = p_recv_data[5]
	};
	stru_com_send.reset_type = p_recv_data[6];

	switch(stru_com_send.reset_type)
	{
		case 0x01://根HUB复位
			if(stru_com_send.dst_addr == 0xff || stru_com_send.dst_addr == (device.addr>>4))
			{
				GPIO_ResetBits(HUB0_REST_PORT, HUB0_REST_PIN);
				GPIO_ResetBits(HUB1_REST_PORT, HUB1_REST_PIN);
				Delay_ms(100);
				GPIO_SetBits(HUB0_REST_PORT, HUB0_REST_PIN);	
				GPIO_SetBits(HUB1_REST_PORT, HUB1_REST_PIN);
			}
			break;
		case 0x02://子HUB复位
			if((stru_com_send.dst_addr/2) == (device.addr/2))
			{
				GPIO_ResetBits(HUB0_REST_PORT, HUB0_REST_PIN);  
				GPIO_ResetBits(HUB1_REST_PORT, HUB1_REST_PIN);  
				Delay_ms(100);	
				GPIO_SetBits(HUB0_REST_PORT, HUB0_REST_PIN);  			
				GPIO_SetBits(HUB1_REST_PORT, HUB1_REST_PIN);  			
			}
			break;
		case 0x04://孙HUB复位
			if(stru_com_send.dst_addr == device.addr)
			{
				if(stru_com_send.dst_addr == device.port_id[0])
				{
					GPIO_ResetBits(HUB0_REST_PORT, HUB0_REST_PIN);  
					Delay_ms(100);	
					GPIO_SetBits(HUB0_REST_PORT, HUB0_REST_PIN);  			
				}
				else if(stru_com_send.dst_addr == device.port_id[1])
				{
					GPIO_ResetBits(HUB1_REST_PORT, HUB1_REST_PIN);  
					Delay_ms(100);	
					GPIO_SetBits(HUB1_REST_PORT, HUB1_REST_PIN);  			
				}
			}
			break;
		case 0x40://单片机复位E
			if((stru_com_send.dst_addr == 0xff) || (stru_com_send.dst_addr == device.addr))
			{
				rewrite_ADC_BaseLine_flash_data();
				NVIC_SystemReset();
			}
			break;
		case 0x80://单片机复位
			if((stru_com_send.dst_addr == 0xff) || (stru_com_send.dst_addr == device.addr))
			{
				NVIC_SystemReset();
			}
			break;
	}
}

void com_file_operations_request(u8 * p_recv_data)
{
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_FILE_OPERATIONS_REQ,
		.dst_addr = p_recv_data[3]
	};

	stru_com_send.file_operation_type = p_recv_data[5];
	stru_com_send.file_area = p_recv_data[6];
	stru_com_send.file_num = p_recv_data[7];
	stru_com_send.file_size = (u32)p_recv_data[8]<<24 | (u32)p_recv_data[9]<<16 | (u32)p_recv_data[10]<<8 | (u32)p_recv_data[11];

	u32 read_flash_len = 0;
	u8 str_buffer_1_data = 0;
	u8 com_file_area_index = stru_com_send.file_area - 1;
	switch(com_file_area_index)
	{
		case QR_CODE_AREA:
			read_flash_len = 256;
			str_buffer_1_data = 13;
			break;
		case FONT_AREA:
			read_flash_len = 256*4;
			str_buffer_1_data = 13*4;
			break;
		case BACKGROUND_AREA:
			read_flash_len = 256*4;
			str_buffer_1_data = 13*4;
			break;
		case MEDIA_AREA:
			read_flash_len = 256*4;
			str_buffer_1_data = 13*4;
			break;
		case EMBEDDED_AREA:
			read_flash_len = 256;
			str_buffer_1_data = 13;
			break;
	}

	FLASH2_GPIOSPI_Read (file_area_addr_min[com_file_area_index], str_buffer, read_flash_len);

	u16 i,len,re_send;
	u32 f_size,f_temp;
	u16 len_tmp, len_tmp_1;

	if((file_id == stru_com_send.file_num) && ((file_wr&0X0F)==1))
	{
		re_send =0xff;//确定为重发
		stru_com_send.confirm_flag	= 0xff;
	}
	else
	{
		re_send =0x00;//非重发
		stru_com_send.confirm_flag	= 0x00;
	}

	
	if(str_buffer[0] != RECV_COM_MSG_HEAD)
	{
		str_buffer[0]= RECV_COM_MSG_HEAD;
		str_buffer[1]= 0x00;
		f_temp = file_area_addr_min[com_file_area_index] + 0x1000;
		str_buffer[2]= ((f_temp>>16)&0xff);
		str_buffer[3]= ((f_temp>>8)&0xff);
	}
	
	if(str_buffer[0] != RECV_COM_MSG_HEAD || (str_buffer[0]==RECV_COM_MSG_HEAD && re_send==0x00))
	{
		len = str_buffer[1];		
		len = len*18+6;
		file_addr = str_buffer[2];
		file_addr <<=8;
		file_addr += str_buffer[3];
		file_addr <<=8;

		f_temp = stru_com_send.file_size;
		f_size = file_addr+f_temp;

		if(f_size < file_area_addr_max[com_file_area_index] && str_buffer[1] <= str_buffer_1_data)
		{
			stru_com_send.confirm_flag = 0xff;
			if((f_size&0xff)==0)
			{
				NextFileAddr = f_size;
			}
			else
			{
				NextFileAddr = (f_size & 0xffffff00);
				NextFileAddr += 0x100;
			}
			FLASH2_GPIOSPI_SER(file_area_addr_min[com_file_area_index]);
			for(i=0;i<14;i++)
			{
				str_buffer[len-2+i]=	p_recv_data[6+i];
			}
			
			if(com_file_area_index == QR_CODE_AREA)
				str_buffer[len-2+6] += 4;	//先写假文件名。固定加4在文件名最高字节。

			str_buffer[len-2+i] = (file_addr>>24);
			i++;
			str_buffer[len-2+i] = (file_addr>>16)&0xff;
			i++;
			str_buffer[len-2+i] = (file_addr>>8)&0xff;
			i++;
			str_buffer[len-2+i] = (file_addr)&0xff;

			str_buffer[0]	= frame_headerC;
			str_buffer[1]	+= 1;
			str_buffer[2]	= (NextFileAddr>>16)&0xff;
			str_buffer[3]	= (NextFileAddr>>8)&0xff;
			len = str_buffer[1];		
			len = len*18+6;

			str_buffer[len-2] = 0;
			for(i=1;i<(len-2);i++)
			{
				str_buffer[len-2] += str_buffer[i];
			}
			str_buffer[len-1] = frame_last;	

			len_tmp = 256;
			len_tmp_1 = 1024;
			if(com_file_area_index == QR_CODE_AREA)
				len_tmp = len;
			if(com_file_area_index == QR_CODE_AREA || com_file_area_index == EMBEDDED_AREA)
				len_tmp_1 = 1;
			
			for(i=0;i<len_tmp_1;i+=256)
			{
				FLASH2_GPIOSPI_Write(file_area_addr_min[com_file_area_index]+i, &str_buffer[i], len_tmp);	
			}
		}
		else
		{
			stru_com_send.confirm_flag	= 0x00;
		}
	}

	if(stru_com_send.confirm_flag == 0xff)
	{
		file_id = stru_com_send.file_num;		//当前写的
		file_wr = 1;	//文件操作允许
		file_wr |= (stru_com_send.file_area<<4);
		file_hook = 0;
	}
	u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
	UART1_Send_Data(sz_cmd,frame_len);
}

void com_file_transfer(u8 * p_recv_data)
{
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_FILE_TRANSFER,
		.dst_addr = p_recv_data[3]
	};
	
	stru_com_send.transfer_flag = p_recv_data[5];
	stru_com_send.section_num = p_recv_data[6];
	
	u8 * p_data = p_recv_data + 7;
	u16 data_len = (stru_com_send.transfer_flag == 0 ? 256 : ((u16)p_recv_data[1]*2 - 10));

	
	u16 i,j,n,len,delay_i;

	if((stru_com_send.section_num==file_hook)&& ((file_wr&0x01) ==1))//钩子有没有对上
	{
		file_wr |= 0x02;//开始写文件
		
		stru_com_send.confirm_flag = 0xFF;
		memcpy(str_buffer, p_data, data_len);

		FLASH2_GPIOSPI_Write(file_addr, str_buffer, data_len);
		
		n=3;
		while(n--)
		{
			stru_com_send.confirm_flag = 0xFF;
			
			delay_i=10000;
			while(delay_i--);
			
			FLASH2_GPIOSPI_Read(file_addr, str_buffer1, data_len);

			if(memcmp(str_buffer, str_buffer1, data_len)) stru_com_send.confirm_flag = 1;
			else break;
		}
		if(stru_com_send.confirm_flag == 1)
		{
			j= file_addr&0xfff;
			n=100;
			while(n--)
			{
				stru_com_send.confirm_flag = 0xff;
				delay_i=10000;
				while(delay_i--);
				FLASH2_GPIOSPI_Read(file_addr&0xfffff000, str_buffer, j);
				delay_i=10000;
				while(delay_i--);
				FLASH2_GPIOSPI_Read(file_addr&0xfffff000, str_buffer1, j);

				if(memcmp(str_buffer, str_buffer1, j)) stru_com_send.confirm_flag = 1;
				else break;
			}
		}
		if(stru_com_send.confirm_flag == 0xFF)
		{
			file_hook++;
			file_addr += data_len;
		}
	}
	else if(stru_com_send.section_num + 1 == file_hook)//发上一包
	{
		stru_com_send.confirm_flag  = 0xFF;
	}
	else
	{
		stru_com_send.confirm_flag  = 0x00;
	}
	
	if(stru_com_send.transfer_flag==0xff)//传输完成
	{
		u8 com_file_area_index = (file_wr >> 4) - 1;
		if(com_file_area_index == QR_CODE_AREA)
		{
			device.use |= 0x10;
			FLASH2_GPIOSPI_Read (file_area_addr_min[com_file_area_index], str_buffer, 256);

			len = str_buffer[1];	//改上一个文件的文件名	
			len = len*18+6;
			str_buffer[len-2+6-18] -= 4;//先写假文件名。固定加4在文件名最高字节。
			str_buffer[len-2] = 0;
			for(i=1;i<(len-2);i++)
			{
				str_buffer[len-2] += str_buffer[i];
			}
			str_buffer[len-1] = frame_last;	

			FLASH2_GPIOSPI_SER(file_area_addr_min[com_file_area_index]);
			FLASH2_GPIOSPI_Write(file_area_addr_min[com_file_area_index], str_buffer, len);
		}
		file_wr = 0;
	}

	u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
	UART1_Send_Data(sz_cmd,frame_len);
}

void com_file_called(u8 * p_recv_data)
{
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_FILE_CALLED,
		.dst_addr = p_recv_data[3]
	};

	stru_com_send.port_num = p_recv_data[5];
	stru_com_send.file_area = p_recv_data[6];
	stru_com_send.file_num = p_recv_data[7];
	
	u16 i,x_load,y_load;
	u32 f_addr,f_size = 0;
	
	u32 read_flash_len = 0;
	u8 str_buffer_1_data = 0;
	u8 com_file_area_index = stru_com_send.file_area - 1;

	stru_com_send.confirm_flag = 0x01;
	switch(com_file_area_index)
	{
		case QR_CODE_AREA:
			read_flash_len = 256;
			str_buffer_1_data = 13;
			break;
		case FONT_AREA:
			read_flash_len = 256*4;
			str_buffer_1_data = 13*4;
			break;
		case BACKGROUND_AREA:
			read_flash_len = 256*4;
			str_buffer_1_data = 13*4;
			break;
		case EMBEDDED_AREA:
			read_flash_len = 256;
			str_buffer_1_data = 13;
			break;
		default:
			stru_com_send.confirm_flag = 0x00;
			break;
	}
	if(stru_com_send.confirm_flag)
	{
		FLASH2_GPIOSPI_Read (file_area_addr_min[com_file_area_index], str_buffer, read_flash_len);
		f_addr = 0;
		f_size = 0;
		if(str_buffer[1]>str_buffer_1_data)
		{	
			str_buffer[1] = str_buffer_1_data;	
		}

		stru_com_send.confirm_flag = 0x00;
		for(i=0;i<str_buffer[1];i++)
		{
			if(str_buffer[0]!=frame_headerC)
				break;

			if(str_buffer[4+18*i+1]==stru_com_send.file_num)//确认文件号
			{
				stru_com_send.confirm_flag = 0xff; 
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
	}
	u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
	UART1_Send_Data(sz_cmd,frame_len);

	if(stru_com_send.confirm_flag && f_size>0)
	{
		u8 lcd_index = stru_com_send.port_num%2;
		u8 lcd_cs = lcd_index + 1;
		switch(com_file_area_index)
		{
			case QR_CODE_AREA:
				display_flash_BMP (80,56,f_addr,lcd_cs);
				break;
			case FONT_AREA:
				FLASH2_GPIOSPI_Read (f_addr, LCD_TxtBuffer[lcd_index], f_size);
				LCD_TxtBuffer[lcd_index][2048]=0;
				LCD_TxtBuffer[lcd_index][2049]=0;
				break;
			case BACKGROUND_AREA:
				if(p_recv_data[1]>=7)
				{
					x_load = p_recv_data[8];
					x_load <<= 8;
					x_load += p_recv_data[9];
					y_load = p_recv_data[10];
					y_load <<= 8;
					y_load += p_recv_data[11];
				}
				else
				{
					x_load = 0;
					y_load = 0;
				}
				ADC1_3_PeriphClockCmd(DISABLE);
				display_flash_BMP (x_load,y_load,f_addr,lcd_cs);
				ADC1_3_PeriphClockCmd(ENABLE);
				break;
			case EMBEDDED_AREA:
				FLASH2_GPIOSPI_Read (file_area_addr_max[INFO_AREA], info2STR.head, sizeof(info2STR));
				info2STR.item21[0] = 21;
				info2STR.item21[1] = 4;
				info2STR.item21_data[0] = 0x01;
				info2STR.item21_data[1] = 0x00;
				info2STR.item21_data[2] = 0x05;
				info2STR.item21_data[3] = stru_com_send.file_num;
				FLASH2_GPIOSPI_SER(file_area_addr_max[INFO_AREA]);
				FLASH2_GPIOSPI_Write(file_area_addr_max[INFO_AREA], info2STR.head, sizeof(info2STR));

				if(device.Version[0]==Version_FLAG1)
				{
					Check_CHIP_PRO();
				}
				else
				{
					NVIC_SystemReset();
				}
				break;
		}
	}
}

void com_file_erase(u8 * p_recv_data)
{
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_FILE_ERASE,
		.dst_addr = p_recv_data[3]
	};
	
	stru_com_send.confirm_flag = 0x01;
	u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
	UART1_Send_Data(sz_cmd,frame_len);

	u32 temp = 0;
	stru_com_send.file_area = p_recv_data[5];
	u8 com_file_area_index = stru_com_send.file_area - 1;
	
	FLASH2_GPIOSPI_Read (file_area_addr_max[INFO_AREA], info2STR.head, sizeof(info2STR));  //媒休初始化
	stru_com_send.confirm_flag = 0xff;
	switch(com_file_area_index)
	{
		case QR_CODE_AREA:
			info2STR.item11[0] = 11;
			info2STR.item11[1] = 0;
			break;
		case FONT_AREA:
			info2STR.item12[0] = 12;
			info2STR.item12[1] = 0;
			break;
		case BACKGROUND_AREA:
			info2STR.item13[0] = 13;
			info2STR.item13[1] = 0;
			break;
		case MEDIA_AREA:
			info2STR.item14[0] = 14;
			info2STR.item14[1] = 0;
			break;
		case EMBEDDED_AREA:
			info2STR.item15[0] = 15;
			info2STR.item15[1] = 0;
			break;
		default:
			stru_com_send.confirm_flag = 0x00;
			break;
	}
	
	if(stru_com_send.confirm_flag)
	{
		for(temp=file_area_addr_min[com_file_area_index];temp<file_area_addr_max[com_file_area_index];) 
		{
			FLASH2_GPIOSPI_SE(temp);
			temp +=0x10000;
		}
		
		FLASH2_GPIOSPI_SER(file_area_addr_max[INFO_AREA]);	//每次擦擦4K
		FLASH2_GPIOSPI_Write(file_area_addr_max[INFO_AREA], info2STR.head, sizeof(info2STR));
	}
	
	frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
	UART1_Send_Data(sz_cmd,frame_len);
}

void com_handshake(u8 * p_recv_data)
{
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_HANDSHAKE,
		.dst_addr = p_recv_data[3]
	};
	u8 handshake_mode = p_recv_data[5];
	u8 display_char[2] = {'C', 'S'};

	stru_com_send.handshake_mode = 0x00;
	
	switch(handshake_mode)
	{
		case 0x01:
		case 0x02:
			stru_com_send.handshake_mode = handshake_mode;
			UART_BUFFER[0] = display_char[handshake_mode-1];
			UART_BUFFER[1] = 0;
			tft_DisplayStr(0, 0, UART_BUFFER,0XFFFF, 0X0000,3);
			#ifndef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
			LCDC.LCDSPTime[LCD1_INDEX] = 0;
			LCDC.LCDSPTime[LCD2_INDEX] = 0;
			#endif
			break;
		case 0x03://APP MODE
			if(device.Version[0]!=Version_FLAG1)
			{
				stru_com_send.handshake_mode = handshake_mode;
			}
			break;
		case 0x04://BL MODE
		case 0x1f://BL保持
		case 0x10://BL退出
			if(device.Version[0]==Version_FLAG1)
			{
				stru_com_send.handshake_mode = handshake_mode;
				if(handshake_mode >= 0x10)
				{
					KEEP_EN = (handshake_mode == 0x10 ? 0x00 : 0xff);
				}
			}
			break;
	}
	
	u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
	UART1_Send_Data(sz_cmd,frame_len);
}

void com_media_time_ctrl(u8 * p_recv_data)
{
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_MEDIA_TIME_CTRL,
		.dst_addr = p_recv_data[3]
	};
	
	u8 item_number = p_recv_data[5];
	u8 * p_data = p_recv_data + 5;
	u8 i;

	stru_com_send.confirm_flag = 0x01;

	FLASH2_GPIOSPI_Read (file_area_addr_max[INFO_AREA], info2STR.head, sizeof(info2STR));  //媒休初始化

	info2STR.head[0] = frame_headerC;
	info2STR.head[1] = sizeof(info2STR); 
	info2STR.head[1] >>= 1;
	info2STR.temp[0] = p_recv_data[3];
	info2STR.temp[1] = device.addr;
	info2STR.temp[2] = 0xff;//项目数不确定值
	
	if(item_number==1)  //项目号
	{
		for(i=0;i<5;i++)
		{
			info2STR.item1[i] = p_data[i];
		}
		LCDC.PSwitch = info2STR.item1_data[0];
		LCDC.LCDPTimeSet = info2STR.item1_data[1];	
		LCDC.LCDPTimeSet <<= 8;	
		LCDC.LCDPTimeSet += info2STR.item1_data[2];	
	}
	else if(item_number==2)  //项目号
	{
		for(i=0;i<5;i++)
		{
			info2STR.item2[i] = p_data[i];
		}
		LCDC.SPSwitch = info2STR.item2_data[0];
		LCDC.LCDSPTimeSet = info2STR.item2_data[1];	
		LCDC.LCDSPTimeSet <<= 8;	
		LCDC.LCDSPTimeSet += info2STR.item2_data[2];	
	}
	else if(item_number==3)  //项目号提示字开关
	{
		for(i=0;i<4;i++)
		{
			info2STR.item3[i] = p_data[i];
		}
	}
	FLASH2_GPIOSPI_SER(file_area_addr_max[INFO_AREA]);  ////每次擦擦4K
	FLASH2_GPIOSPI_Write(file_area_addr_max[INFO_AREA], info2STR.head, sizeof(info2STR));
	
	memcpy(str_buffer, &info2STR, sizeof(info2STR));
	FLASH2_GPIOSPI_Read (file_area_addr_max[INFO_AREA], &str_buffer[1024], sizeof(info2STR));
	if(memcmp(str_buffer, str_buffer+1024, sizeof(info2STR))) stru_com_send.confirm_flag= 0;
	else stru_com_send.confirm_flag = 0xff;

	u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
	UART1_Send_Data(sz_cmd,frame_len);
	
	FLASH2_GPIOSPI_Read (file_area_addr_min[MEDIA_AREA], str_buffer, 64);  //读取图片张数
	LCDC.PNum = str_buffer[1];
	LCDC.LCDPTime[LCD1_INDEX]=0;		//广告计时
	LCDC.LCDPTime[LCD2_INDEX]=0;		//广告计时

}

void com_set_version(u8 * p_recv_data)
{
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_SET_AREA_VERSION,
		.dst_addr = p_recv_data[3]
	};
	
	stru_com_send.item_number = p_recv_data[5];
	u8 * p_data = p_recv_data + 5;

	FLASH2_GPIOSPI_Read (file_area_addr_max[INFO_AREA], info2STR.head, sizeof(info2STR));  //媒休初始化

	info2STR.head[0] = RECV_COM_MSG_HEAD;
	info2STR.head[1] = sizeof(info2STR); 
	info2STR.head[1] >>= 1;
	info2STR.temp[0] = p_recv_data[3];
	info2STR.temp[1] = device.addr;
	info2STR.temp[2] = 0xff;//项目数不确定值

	if(stru_com_send.item_number >= 11 && stru_com_send.item_number <= 15)
	{
		u8 * p_item = info2STR.item11 + 18 * (stru_com_send.item_number - 11);
		memcpy(p_item, p_data, 18);
	}

	FLASH2_GPIOSPI_SER(file_area_addr_max[INFO_AREA]);  ////每次擦擦4K
	FLASH2_GPIOSPI_Write(file_area_addr_max[INFO_AREA], info2STR.head, sizeof(info2STR));

	memcpy(str_buffer, &info2STR, sizeof(info2STR));
	FLASH2_GPIOSPI_Read (file_area_addr_max[INFO_AREA], &str_buffer[1024], sizeof(info2STR));
	if(memcmp(str_buffer, str_buffer+1024, sizeof(info2STR))) stru_com_send.confirm_flag= 0;
	else stru_com_send.confirm_flag = 0xff;

	u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
	UART1_Send_Data(sz_cmd,frame_len);
}

void com_get_version(u8 * p_recv_data)
{
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_GET_AREA_VERSION,
		.dst_addr = p_recv_data[3]
	};

	stru_com_send.item_number = p_recv_data[5];
	u8 item_content[16] = {0};
	stru_com_send.item_content = item_content;
	FLASH2_GPIOSPI_Read (file_area_addr_max[INFO_AREA], info2STR.head, sizeof(info2STR));  //媒休初始化

	if(stru_com_send.item_number >= 11 && stru_com_send.item_number <= 15)
	{
		u8 * p_item = info2STR.item11 + 18 * (stru_com_send.item_number - 11);

		if(info2STR.item11[1] <= 16)
		{
			stru_com_send.item_len = p_item[1];
			memcpy(stru_com_send.item_content, p_item + 2, stru_com_send.item_len);
		}
		else
		{
			stru_com_send.item_len = 0;
		}

		u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
		UART1_Send_Data(sz_cmd,frame_len);
	}
}

void com_get_ad_count(u8 * p_recv_data)
{
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_AD_DATA_STATISTICS,
		.dst_addr = p_recv_data[3]
	};

	stru_com_send.item_number = p_recv_data[5];
	stru_com_send.item_len = LCDC.PNum << 1;
	stru_com_send.item_content = (u8 *)AD_count;

	if(stru_com_send.item_number == 129)
	{
		u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
		UART1_Send_Data(sz_cmd,frame_len);
		memset(AD_count, 0, sizeof(AD_count));
	}
}

void com_get_embedded_version(u8 * p_recv_data)
{
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_GET_EMBEDDED_VERSION,
		.dst_addr = p_recv_data[3]
	};

	stru_com_send.item_number = p_recv_data[5];
	stru_com_send.item_len = 9;

	stru_com_send.item_content = device.Version;
	
	#ifdef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
	if((device.Version[0]==Version_FLAG1)&&((PRO_State&4)==4))	 //在B类中有A类
	{
		u8 version[12];
		memcpy(version, device.Version, sizeof(version));
		version[0] ='E';//无效版本
		stru_com_send.item_content = version;
	}
	#endif
	
	if(stru_com_send.item_number == 130)
	{
		u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
		UART1_Send_Data(sz_cmd,frame_len);
	}
}

void com_read_flash(u8 * p_recv_data)
{
	u32 addr,i;
	u32 end_addr;

	addr = p_recv_data[6];
	addr <<= 8;
	addr += p_recv_data[7];
	addr <<= 8;

	end_addr = p_recv_data[8];
	end_addr <<= 8;
	end_addr += p_recv_data[9];
	end_addr <<= 8;

	for(i=addr;i<end_addr;)  //设置开始页，与结束页
	{
		FLASH2_GPIOSPI_Read(i, str_buffer,1024);
		i+=1024;
		UART1_Send_Data(str_buffer,1024);
	}
}

//---------------------------------------------------------------------------------
void com_write_flash(u8 * p_recv_data)
{
	u32 addr;
	u16 len,i;

	addr = p_recv_data[6];
	addr <<= 8;
	addr += p_recv_data[7];
	addr <<= 8; //设置开始页

	if(addr>=Addr_info1)
	{
		FLASH2_GPIOSPI_SER(addr);

		len = p_recv_data[1];
		len <<= 1;
		len -= 10; //除两头尾效字节
		for(i=0;i<(len);i++)
		{
			str_buffer[i] = p_recv_data[8+i];
		}
		FLASH2_GPIOSPI_Write(addr, str_buffer, len);

		u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
		struct com_send_stru stru_com_send = {
			.cmd_type = COM_CMD_WRITE_FLASH,
			.dst_addr = p_recv_data[3]
		};
		stru_com_send.confirm_flag = 0xff;
		u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
		UART1_Send_Data(sz_cmd,frame_len);
	}
}

void com_get_adc(u8 * p_recv_data)
{
	u8 i;
	u16 ADC_data[ADC1_3_ENABLE_CHANNEL_NUM] = {0};
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	get_ADC1_3_data(ADC_data);
	
	for(i=0;i<ADC1_3_ENABLE_CHANNEL_NUM;i++)
	{
		sz_cmd[2*i] = (ADC_data[i]>>8);
		sz_cmd[2*i+1] = (ADC_data[i]&0xff);
	}

	for(i=0;i<ADC1_3_ENABLE_CHANNEL_NUM;i++)
	{
		sz_cmd[ADC1_3_ENABLE_CHANNEL_NUM*2+2*i] = (ADC_Base0[i]>>8);
		sz_cmd[ADC1_3_ENABLE_CHANNEL_NUM*2+2*i+1] = (ADC_Base0[i]&0xff);
	}
	UART1_Send_Data(sz_cmd,ADC1_3_ENABLE_CHANNEL_NUM*4);

}

void com_save_adc(u8 * p_recv_data)
{
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_SAVE_ADC,
		.dst_addr = p_recv_data[3]
	};
	
	rewrite_ADC_BaseLine_flash_data();
	
	FLASH2_GPIOSPI_Read (Addr_info1, str_buffer, 64);

	if(memcmp(str_buffer+6, global_u8p, ADC1_3_ENABLE_CHANNEL_NUM*2)) stru_com_send.confirm_flag = 0x00;
	else stru_com_send.confirm_flag = 0xff;

	u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
	UART1_Send_Data(sz_cmd,frame_len);
}

void com_rgb888_565(u8 * p_recv_data)
{
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_RGB888_565,
		.dst_addr = p_recv_data[3]
	};
	stru_com_send.file_area = p_recv_data[6];
	
	u32 read_flash_len = 0;
	u8 str_buffer_1_data = 0;
	u8 com_file_area_index = stru_com_send.file_area - 1;
	u16 i;
	u32 f_addr,f_size;
	
	stru_com_send.confirm_flag = 0xff;
	switch(com_file_area_index)
	{
		case QR_CODE_AREA:
			read_flash_len = 256;
			str_buffer_1_data = 10;
			break;
		case FONT_AREA:
			read_flash_len = 256*8;
			str_buffer_1_data = 100;
			break;
		case BACKGROUND_AREA:
			read_flash_len = 256;
			str_buffer_1_data = 10;
			break;
		default:
			stru_com_send.confirm_flag = 0x00;
			break;
	}

	if(stru_com_send.confirm_flag)
	{
		FLASH2_GPIOSPI_Read (file_area_addr_min[com_file_area_index], str_buffer, read_flash_len);
		
		stru_com_send.confirm_flag = 0x00;
		f_addr = 0;
		f_size = 0;
		if(str_buffer[1]>str_buffer_1_data)
		{	
			str_buffer[1] = str_buffer_1_data;	
		}

		for(i=0;i<str_buffer[1];i++)
		{
			if(str_buffer[0] != RECV_COM_MSG_HEAD)	
				break;
			
			if(str_buffer[4+18*i+1]==UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+7)&UART1_RX_MAX])//确认文件号
			{
				stru_com_send.confirm_flag = 0xff;
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
	}
	u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
	UART1_Send_Data(sz_cmd,frame_len);
	
	if(stru_com_send.confirm_flag && f_size>0)
	{
		RGB888_565(f_addr);
	}
}

void com_rgb_clear(u8 * p_recv_data)
{
	u16 colour_t,x_l,y_l,x_o, y_o;
	if(p_recv_data[1]>=9)
	{		 		 
		colour_t  = p_recv_data[6];
		colour_t <<= 8; 
		colour_t += p_recv_data[7];

		x_l  = p_recv_data[8];
		x_l <<= 8; 
		x_l += p_recv_data[9];
		y_l  = p_recv_data[10];
		y_l <<= 8; 
		y_l += p_recv_data[11];

		x_o  = p_recv_data[12];
		x_o <<= 8; 
		x_o += p_recv_data[13];
		y_o  = p_recv_data[14];
		y_o <<= 8; 
		y_o += p_recv_data[15];
		tft_Clear(x_l,y_l,x_o,y_o,colour_t,p_recv_data[5]);
	}
}

//-----------------------------------------
void com_chip_pro(u8 * p_recv_data)
{
	u8 sz_cmd[MAX_SERIAL_BUFFER_LENGHT] = {0};
	struct com_send_stru stru_com_send = {
		.cmd_type = COM_CMD_CHIP_PRO,
		.dst_addr = p_recv_data[3]
	};

	stru_com_send.file_area = p_recv_data[6];
	stru_com_send.file_num = p_recv_data[7];
	
	u8 com_file_area_index = stru_com_send.file_area - 1;
	u16 i;
	u32 f_addr,f_size;
	
	if(com_file_area_index <= EMBEDDED_AREA)
	{
		FLASH2_GPIOSPI_Read (file_area_addr_min[com_file_area_index], str_buffer, 256);
	}

	stru_com_send.confirm_flag = 0x00;

	if(stru_com_send.file_area == 0xE0)
	{
		stru_com_send.confirm_flag = 0xff;
		str_buffer[0] = 0;
		str_buffer[1] = 0;
		FLASH2_GPIOSPI_Read (Addr_info2, info2STR.head, sizeof(info2STR));  //媒休初始化
		info2STR.item21[0] = 21;
		info2STR.item21[1] = 4;
		info2STR.item21_data[0] = 0xE0;		//自毁
		info2STR.item21_data[1] = 0x00;
		info2STR.item21_data[2] = 0;
		info2STR.item21_data[3] = 0;
		FLASH2_GPIOSPI_SER(Addr_info2);		//每次擦擦4K
		FLASH2_GPIOSPI_Write(Addr_info2, info2STR.head, sizeof(info2STR));

		if(device.Version[0]==Version_FLAG1)	//bl程序中
		{
			Check_CHIP_PRO();
		}
		else								//USER程序中
		{
			u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
			UART1_Send_Data(sz_cmd,frame_len);
			NVIC_SystemReset();				//复位
		}						
	}
	f_addr = 0;
	f_size = 0;
	if(str_buffer[1]>13)
	{	
		str_buffer[1] = 13;
	}

	for(i=0;i<str_buffer[1];i++)
	{
		if(str_buffer[0] != RECV_COM_MSG_HEAD)	
			break;
		
		if(str_buffer[4+18*i+1] == stru_com_send.file_num)//确认文件号
		{
			stru_com_send.confirm_flag = 0xff; 
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

	u16 frame_len = sprintf_com_cmd(sz_cmd, stru_com_send);
	UART1_Send_Data(sz_cmd,frame_len);

	if(f_size>0)
	{
		FLASH2_GPIOSPI_Read (Addr_info2, info2STR.head, sizeof(info2STR));  //媒休初始化
		info2STR.item21[0] = 21;
		info2STR.item21[1] = 4;
		info2STR.item21_data[0] = 0x01;
		info2STR.item21_data[1] = 0x00;
		info2STR.item21_data[2] = stru_com_send.file_area;
		info2STR.item21_data[3] = stru_com_send.file_num;
		FLASH2_GPIOSPI_SER(Addr_info2);			//每次擦擦4K
		FLASH2_GPIOSPI_Write(Addr_info2, info2STR.head, sizeof(info2STR));

		if(device.Version[0]==Version_FLAG1)		//bl程序中
		{
			Check_CHIP_PRO();
		}
		else									//USER程序中
		{
			NVIC_SystemReset();					//复位
		}						
	}
}

void deal_cmd_data(struct cmd_recv_stru *p_cmd_recv_stru)
{
	if(p_cmd_recv_stru->cmd_recv_state == COM_CMD_RECV_COMPLETE)
	{
		u16 recv_len = p_cmd_recv_stru->cmd_length;
		u8 recv_data[MAX_SERIAL_BUFFER_LENGHT] = {0};
		memcpy(recv_data, p_cmd_recv_stru->cmd_buffer, recv_len);
		
		if((recv_len == recv_data[1]*2)
			&& (recv_data[0] == RECV_COM_MSG_HEAD)
			&& (recv_data[3] == COM_PC_ADDR || recv_data[3] == COM_GLOBLE_ADDR)
			&& (recv_data[recv_len-1] == DEFAULT_COM_MSG_TAIL)
			&& (recv_data[recv_len-2] == get_checksum(recv_data, recv_len)))
		{
			switch(recv_data[4])
			{
				//case 0x51:  cmd_Device_Info();						break;//获取设备信息
				#ifndef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
				case 0x53:  com_get_port_state(recv_data);			break;//获取端口信息
				case 0x54:  com_get_charge_speed(recv_data);			break;//获取充电速度
				case 0x55:  com_check_qr_code(recv_data);				break;//核对二维码信息
				#endif

				case 0x57:  com_set_device_num(recv_data);			break;//设备号

				#ifndef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
				case 0x59:  com_power_off(recv_data);					break;//断电命令
				case 0x5a:  com_power_on(recv_data);					break;//上电命令
				#endif

				case 0x10:  com_hub_reset(recv_data);					break;//复位HUB		 
				case 0x11:  com_file_operations_request(recv_data);	break;//文件操作请求
				case 0x12:  com_file_transfer(recv_data);				break;//文件传输
				case 0x13:  com_file_called(recv_data);				break;//文件调用
				case 0x14:  com_file_erase(recv_data);				break;//文件擦除
				case 0x16:  com_handshake(recv_data);					break;//握手
				case 0x30:  com_media_time_ctrl(recv_data);			break;//媒体控制命令
				//case 0x32:  cmd_Set_MediaV();					break;//媒体控制命令
				//case 0x33:  cmd_Get_MediaV();					break;//媒体控制命令
				case 0x32:  com_set_version(recv_data);				break;//设置版本
				case 0x33:  com_get_version(recv_data);				break;//获取版本

				#ifndef BOOTLOADER_SOURCECODE//ZHZQ_CHANGE
				case 0x34:  com_get_ad_count(recv_data);				break;//获取广告计数
				#endif
				
				case 0x3E:  NVIC_SystemReset();							break;//复位设备
				//case 0xE1:  cmd_Get_State();					break;//读HUB号到
				//case 0xE2:  cmd_Set_State();					break;//设置HUB号到FLASH
				//case 0xE3:  cmd_Erase_Flash();				break;//
				case 0xE4:  com_read_flash(recv_data);				break;//
				case 0xE5:  com_write_flash(recv_data);				break;//	
				case 0xE6:  com_get_adc(recv_data);					break;//						
				case 0xE7:  com_save_adc(recv_data);					break;//保存ADC基线
				case 0xE8:  com_rgb888_565(recv_data);				break;//
				case 0xE9:  com_rgb_clear(recv_data);					break;//
				case 0xEA:  com_chip_pro(recv_data);					break;//
				case 0xEB:  com_get_embedded_version(recv_data);		break;//

				default:												break;		
			}
		}
		p_cmd_recv_stru->cmd_recv_state = COM_CMD_RECV_INCOMPLETE;
	}
}
#endif
#ifndef ZHZQ_TEST
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
							//case 0xE3:  cmd_Erase_Flash();					break;//
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
			#ifndef BOOTLOADER_SOURCECODE
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
			#endif
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
#endif
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
	device.Version[8] = '2'; //5月第2版
	
	#elif defined(BOOTLOADER_SOURCECODE)//ZHZQ_CHANGE
	device.Version[0] = 'B'; //引导号 BL
	
	device.Version[4] = '1'; //2017年 写年份的最后两位0-99,4位表一位十进制
	device.Version[5] = '7'; //2017年 

	device.Version[7] = '1'; //5月第2版    //Version[1]高四位是月份。低四位是当月产生的版本。
	device.Version[8] = '2'; //5月第2版
	
	#else
	device.Version[0] = 'A'; //引导号 APP

	device.Version[4] = '1'; //2017年 写年份的最后两位0-99,4位表一位十进制
	device.Version[5] = '7'; //2017年 

	device.Version[7] = '1'; //5月第2版    //Version[1]高四位是月份。低四位是当月产生的版本。
	device.Version[8] = '6'; //5月第2版
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
		#else
		NVIC_SystemReset();
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
			#ifdef ZHZQ_TEST
			deal_cmd_data(&stru_cmd_recv);
			#else
			uart1_cmd();
			#endif
			uart3_cmd();
		}
	}
}

/* ------------------------------------------end of file---------------------------------------- */

