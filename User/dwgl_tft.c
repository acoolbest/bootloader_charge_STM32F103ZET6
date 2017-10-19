//dwgl_tft.c
//for ssd1963 320*240
//20161101
#include "stm32f10x.h"


u32 g32_temp =0;
u16 g16_temp =0;
unsigned char AnsiChardata[32] = {
	0xBF, 0xC9, 0xB3, 0xE4, 0xB5, 0xE7, 0xCA, 0xB1, 0xBC, 0xE4, 0x3A, 0x20, 0x30, 0x30, 0xD0, 
	0xA1, 0xCA, 0xB1, 0x20, 0x30, 0x30, 0xB7, 0xD6, 0x20, 0x30, 0x30, 0xC3, 0xEB,0
};
unsigned char HZ_XZWM[9] = {
	0xD0, 0xBE, 0xD6, 0xD0, 0xCE, 0xDE, 0xC2, 0xEB,0
};
unsigned char HZ_SRFY[9] = {
	0xCA, 0xC0, 0xC8, 0xF4, 0xB8, 0xA1, 0xd4, 0xc6,0
};
/****************************************************************************
 * 名    称：void ili9320_PutChar(u16 x,u16 y,u8 c,u16 charColor,u16 bkColor)
 * 功    能：在指定座标显示一个8x16点阵的ascii字符
 * 入口参数：x          行座标
 *           y          列座标
 *           charColor  字符的颜色
 *           bkColor    字符背景颜色
 * 出口参数：无
 * 说    明：显示范围限定为可显示的ascii码
 * 调用方法：ili9320_PutChar(10,10,'a',0x0000,0xffff);
 ****************************************************************************/
//u16 x_offset,u16 y_offset
//void tft_1bitdeep_array(u16 x,u16 y,u16 wide,u16 len,u8 *array)
void tft_1bitdeep_array(u16 x,u16 y,u16 x_offset,u16 y_offset,const u8 *array)
{
	u8 temp_char,k=0;//
	u32 i=0,n=0;//

	//   for (i=0;i<y_offset;i++)
	//   {
	//     for (j=0;j<x_offset;j++)
	//     {
	// 			if((n%8)==0)
	// 			{
	// 			tmp_char=array[n/8];
	// 			}
	// 			k= j%8;
	//       if ( (tmp_char >> 7-k) & 0x01 == 0x01)
	//         {
	//           ili9320_SetPoint(x+j,y+i,0x0000); // 字符颜色
	//         }
	//         else
	//         {
	//           ili9320_SetPoint(x+j,y+i,0xffff); // 背景颜色
	//         }
	// 				n++;
	//     }
	//   }

	LCD_WindowMax (x, y, x+x_offset-1, y+y_offset-1);       //设置窗口	
	LCD_WriteRAM_Prepare();                       //开始写入GRAM*6 	   	   
	n=(u32)(x_offset)*(y_offset);     
	while(n--)
	{
		if((i%8)==0)
		{
			temp_char=array[i/8];
		}
		k= i%8;
		if ( (temp_char >> 7-k) & 0x01 == 0x01)
		{
			g16_temp=0x0000; // 字符颜色
		}
		else
		{
			g16_temp=0xffff; // 背景颜色
		}

		i++;
		LCD_WE_RAM(g16_temp);
	}               //显示所填充的颜色

	LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
}

//		tft_16bitdeep_array(2,2,240,171,gImage1_240_171_16bitC);
void tft_16bitdeep_array(u16 x,u16 y,u16 x_offset,u16 y_offset,const u8 *array)
{
	u16 i=0;//wide
	u32 n=0;//已经显示的点数
	//  u16 tmp_char=0;

	LCD_WindowMax (x, y, x+x_offset-1, y+y_offset-1);       //设置窗口	
	LCD_WriteRAM_Prepare();                       //开始写入GRAM*6 	   	   
	n=(u32)(x_offset)*(y_offset);     
	while(n--)
	{
		g16_temp = array[i+1];	
		g16_temp <<= 8;		
		g16_temp += array[i];		
		i++;
		i++;
		LCD_WE_RAM(g16_temp);
	}               //显示所填充的颜色

	LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
}

//从左向右扫，从下向上
// typedef struct tagBITMAPFILEHEADER
// {
//     WORD bfType;//BM(1-2字节)
//     DWORD bfSize;//文件大小单为字节(3-6字节低位在前)
//     WORD bfReserved1;//0(7-8??)
//     WORD bfReserved2;//0(9-10??)
//     DWORD bfOffBits;//图像数据偏移地址(11-14字节)
// }BITMAPFILEHEADER;

// typedef struct tagBITMAPINFOHEADER{
// DWORD biSize;//该段字节数(15-18??)
// LONG biWidth;//位图宽(19-22??)
// LONG biHeight;//位图高(23-26??)
// WORD biPlanes;//???????,???1(27-28??)
// WORD biBitCount;//?????????,???1(??),(29-30??)
// //4(16?),8(256?)16(???)?24(???)??
// DWORD biCompression;//??????,???0(???),(31-34??)
// //1(BI_RLE8????)?2(BI_RLE4????)??
// DWORD biSizeImage;//?????(????????????4??????????),??????(35-38??)
// LONG biXPelsPerMeter;//???????,?????(39-42??)
// LONG biYPelsPerMeter;//???????,?????(43-46??)
// DWORD biClrUsed;//???????????????(47-50??)
// DWORD biClrImportant;//?????????????(51-54??)
// }BITMAPINFOHEADER;
void tft_1bitdeep_BMP (u16 x,u16 y,const u8 *array,u16 PenColor, u16 BackColor,u8 cs)
{
	u8 temp_char,k;//
	u32 i=0,j=0;//
	u32 offset,x_offset,y_offset,x_offset1;
	offset = array[13];
	offset <<= 8;
	offset += array[12];
	offset <<= 8;
	offset += array[11];
	offset <<= 8;
	offset += array[10];

	x_offset = array[21];
	x_offset <<= 8;
	x_offset += array[20];
	x_offset <<= 8;
	x_offset += array[19];
	x_offset <<= 8;
	x_offset += array[18];

	y_offset = array[25];
	y_offset <<= 8;
	y_offset += array[24];
	y_offset <<= 8;
	y_offset += array[23];
	y_offset <<= 8;
	y_offset += array[22];

	if((array[0]!='B')||(array[1]!='M'))  //内容认为无效
	{
		x_offset = 0;
	}

	tft_cs_enable(cs);

	if(x_offset>0)
	{
		if(x_offset>240)
		{
			x_offset1 = 240;
		}
		else
		{
			x_offset1 = x_offset;
		}
		offset +=0;
		//	LCD_WindowMax (x, y, x+x_offset-1, y+y_offset-1);       //设置窗口	
		LCD_WindowMax (x, y, x+y_offset-1, y+x_offset1-1);       //设置窗口	
		LCD_WriteRAM_Prepare();                       //开始写入GRAM*6 	   	   

		for(i=0;i<y_offset;i++)	
		{
			for(j=0;j<(x_offset1/8);j++)	
			{
				temp_char = 	array[offset+i*(x_offset/8)+j];

				for(k=0;k<8;k++)
				{
					if ( ((temp_char >> (7-k)) &0x01) == 0x01)
					{
						g16_temp=PenColor; // 字符颜色
					}
					else
					{
						g16_temp=BackColor; // 背景颜色
					}

					LCD_WE_RAM(g16_temp);
				}
			}
		}

		LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
	}
	if(x_offset>240)
	{
		if(x_offset>480)
		{
			x_offset1 = 240;
		}
		else
		{
			x_offset1 = x_offset-240;
		}
		offset +=30;
		//	LCD_WindowMax (x, y, x+x_offset-1, y+y_offset-1);       //设置窗口	
		LCD_WindowMax (x-y_offset-1, y, x+y_offset-1, y+x_offset1-1);       //设置窗口	
		LCD_WriteRAM_Prepare();                       //开始写入GRAM*6 	   	   

		for(i=0;i<y_offset;i++)	
		{
			for(j=0;j<(x_offset1/8);j++)	
			{
				temp_char = 	array[offset+i*(x_offset/8)+j];

				for(k=0;k<8;k++)
				{
					if ( ((temp_char >> (7-k)) &0x01) == 0x01)
					{
						g16_temp=PenColor; // 字符颜色
					}
					else
					{
						g16_temp=BackColor; // 背景颜色
					}

					LCD_WE_RAM(g16_temp);
				}
			}
		}

		LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
	}
	tft_cs_disable(cs);

}
//------------------------------------------------------------
void tft_1bitdeep_TXT (u16 x,u16 y, u8 *array,u16 PenColor, u16 BackColor,u8 cs)
{
	u8 temp_char,k;//
	u32 i=0,j=0;//
	u32 offset,x_offset,y_offset;  //
	u32 x_offset1;  //实际要的窗口长宽
	u32 x_wide;  //图片实际取点的宽度 。为32的倍数
	offset = array[13];
	offset <<= 8;
	offset += array[12];
	offset <<= 8;
	offset += array[11];
	offset <<= 8;
	offset += array[10];

	x_offset = array[21];
	x_offset <<= 8;
	x_offset += array[20];
	x_offset <<= 8;
	x_offset += array[19];
	x_offset <<= 8;
	x_offset += array[18];

	if((x_offset%32)!=0)
	{
		x_wide = x_offset + 32;
		x_wide &= ~0x1f;			
	}
	else
	{
		x_wide = x_offset;
	}
	y_offset = array[25];
	y_offset <<= 8;
	y_offset += array[24];
	y_offset <<= 8;
	y_offset += array[23];
	y_offset <<= 8;
	y_offset += array[22];

	if((array[0]!='B')||(array[1]!='M'))  //内容认为无效
	{
		x_offset = 0;
	}

	tft_cs_enable(cs);

	if((x_offset>0)&&(x_offset<=240))
	{
		x_offset1 = x_offset;

		offset +=0;
		//	LCD_WindowMax (x, y, x+x_offset-1, y+y_offset-1);       //设置窗口	
		LCD_WindowMax (x, y, x+y_offset-1, y+x_offset1-1);       //设置窗口					
		LCD_WriteRAM_Prepare();                       //开始写入GRAM*6 	   	   

		for(i=0;i<y_offset;i++)	
		{
			for(j=0;j<(x_offset1/8);j++)	
			{
				temp_char = 	array[offset+i*(x_wide/8)+j];

				for(k=0;k<8;k++)
				{
					if ( ((temp_char >> (7-k)) &0x01) == 0x01)
					{
						g16_temp=PenColor; // 字符颜色
					}
					else
					{
						g16_temp=BackColor; // 背景颜色
					}

					LCD_WE_RAM(g16_temp);
				}
			}
		}			 
		LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小

		if(y+x_offset<240)   //不足240像素部分要清背景
		{			
			LCD_WindowMax (x,y+x_offset1, x+y_offset-1, 239);
			LCD_WriteRAM_Prepare();                       //开始写入GRAM*6 	   	   
			i= (240-y-x_offset1)*y_offset;
			while(i--)
			{
				LCD_WE_RAM(BackColor);
			}
			//			LCD_WindowMax (0, 0, LCD_W, LCD_H);      	    //恢复窗口
			LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
		}
	}
	else
		if(x_offset>240)
		{
			x_offset1 = 240;
			i = array[2048];  //上次显示后的平移地址
			i <<= 8;			
			i += array[2049];
			offset +=	i;		
			if((i+30)*8<x_offset)
			{
				i+=1;//平移速度
				//			i+=2;//平移速度
			}
			else
			{
				i=0;
			}
			array[2048] = (i>>8)&0xff;  //上次显示后的平移地址
			array[2049] = (i)&0xff;  //上次显示后的平移地址

			//	LCD_WindowMax (x, y, x+x_offset-1, y+y_offset-1);       //设置窗口	
			LCD_WindowMax (x, y, x+y_offset-1, y+x_offset1-1);       //设置窗口	
			LCD_WriteRAM_Prepare();                       //开始写入GRAM*6 	   	   

			for(i=0;i<y_offset;i++)	
			{
				for(j=0;j<30;j++)	
					//		for(j=0;j<(x_offset1/8);j++)	
				{
					temp_char = 	array[offset+i*(x_wide/8)+j];

					for(k=0;k<8;k++)
					{
						if ( ((temp_char >> (7-k)) &0x01) == 0x01)
						{
							g16_temp=PenColor; // 字符颜色
						}
						else
						{
							g16_temp=BackColor; // 背景颜色
						}

						LCD_WE_RAM(g16_temp);
					}
				}
			}

			LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
		}
	tft_cs_disable(cs);

}

//------------------------------------------------------------------
void DisplayPROT_EWM(unsigned int x, unsigned int y, u8 num,u8 cs)
{
	u16 i;
	u32 f_addr,f_size;
	FLASH2_GPIOSPI_Read (Addr_01min, str_buffer, 256);
	f_addr = 0;
	f_size = 0;
	if(str_buffer[1]>13)
	{	
		str_buffer[1] = 13;	
	}

	for(i=0;i<str_buffer[1];i++)
	{
		if(str_buffer[0]!=frame_headerC)	//device.port_id[0]
		{
			break;
		}

		if((str_buffer[4+18*i+1]%2)==num)//确认文件号
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

	if(f_size>0)
	{
		//FLASH2_GPIOSPI_Read (f_addr, str_buffer, f_size);
		//tft_1bitdeep_EWM(x,y,str_buffer,0xffff,0x0000,cs);  //显示单色二维码
		display_flash_BMP (x,y,f_addr,cs);//单色彩色都支持
	}
}
//-------------------------------------
void display_flash_BMP (u16 x,u16 y,unsigned int addr,u8 cs)
{
	u8 temp_char,k;//
	u32 i=0,j=0;//
	u32 offset,x_offset,y_offset;  //
	u32 x_wide;  //图片实际取点的宽度 。为32的倍数
	u32 x_offset1;  //实际要的窗口长宽
	u32 pix;  //显示到的子节数
	unsigned int pix_temp,spi_size;

	FLASH2_GPIOSPI_Read (addr, str_buffer, 64);
	if(str_buffer[28] ==0x01)   //1bit
	{
		offset = str_buffer[13];
		offset <<= 8;
		offset += str_buffer[12];
		offset <<= 8;
		offset += str_buffer[11];
		offset <<= 8;
		offset += str_buffer[10];

		x_offset = str_buffer[21];
		x_offset <<= 8;
		x_offset += str_buffer[20];
		x_offset <<= 8;
		x_offset += str_buffer[19];
		x_offset <<= 8;
		x_offset += str_buffer[18];

		if((x_offset%32)!=0)
		{
			x_wide = x_offset + 32;
			x_wide &= ~0x1f;			
		}
		else
		{
			x_wide = x_offset;
		}
		y_offset = str_buffer[25];
		y_offset <<= 8;
		y_offset += str_buffer[24];
		y_offset <<= 8;
		y_offset += str_buffer[23];
		y_offset <<= 8;
		y_offset += str_buffer[22];

		if((str_buffer[0]!='B')||(str_buffer[1]!='M'))  //内容认为无效
		{
			x_offset = 0;
		}
		if(y_offset>320)  //内容认为无效
		{
			x_offset = 0;
		}

		if((x_offset>0)&&(x_offset<=240))
		{
			tft_cs_enable(cs);

			x_offset1 = x_offset;				
			offset +=0;				
			//	LCD_WindowMax (x, y, x+x_offset-1, y+y_offset-1);       //设置窗口	
			LCD_WindowMax (x, y, x+y_offset-1, y+x_offset1-1);       //设置窗口					
			LCD_WriteRAM_Prepare();  				//开始写入GRAM*6 	

			for(i=0;i<y_offset;i++)	
			{
				for(j=0;j<(x_offset1/8);j++)	
				{
					pix = i*(x_wide/8)+j; 
					if(pix%2048==0)
					{
						if((y_offset*x_wide/8-pix)>2048)
						{
							spi_size= 2048;
						}
						else
						{
							spi_size= (y_offset*x_wide/8-pix);
						}
						FLASH2_GPIOSPI_Read (addr+offset+pix, str_buffer, spi_size);  //读像素内容2K
						pix_temp = pix;
					}
					temp_char = 	str_buffer[pix-pix_temp];

					for(k=0;k<8;k++)
					{
						if ( ((temp_char >> (7-k)) &0x01) == 0x01)
						{
							g16_temp=0x0000; // 字符颜色
						}
						else
						{
							g16_temp=0xffff; // 背景颜色
						}

						LCD_WE_RAM(g16_temp);
					}
				}
			}			 
			LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
			tft_cs_disable(cs);				
		}

	}

	if(str_buffer[28] ==0x10)   //16bit
	{
		offset = str_buffer[13];
		offset <<= 8;
		offset += str_buffer[12];
		offset <<= 8;
		offset += str_buffer[11];
		offset <<= 8;
		offset += str_buffer[10];

		x_offset = str_buffer[21];
		x_offset <<= 8;
		x_offset += str_buffer[20];
		x_offset <<= 8;
		x_offset += str_buffer[19];
		x_offset <<= 8;
		x_offset += str_buffer[18];

		if((x_offset%2)!=0)  //规定扫描一行数据必是4字节的倍数，不足的补0
		{
			x_wide = x_offset + 2;
			x_wide &= ~0x01;			
		}
		else
		{
			x_wide = x_offset;
		}
		y_offset = str_buffer[25];
		y_offset <<= 8;
		y_offset += str_buffer[24];
		y_offset <<= 8;
		y_offset += str_buffer[23];
		y_offset <<= 8;
		y_offset += str_buffer[22];

		if((str_buffer[0]!='B')||(str_buffer[1]!='M'))  //内容认为无效
		{
			x_offset = 0;
		}
		if(y_offset>320)  //内容认为无效
		{
			x_offset = 0;
		}

		if((x_offset>0)&&(x_offset<=240))
		{
			tft_cs_enable(cs);

			x_offset1 = x_offset;
			LCD_WindowMax (x, y, x+y_offset-1, y+x_offset1-1);       //设置窗口					
			LCD_WriteRAM_Prepare();  				//开始写入GRAM*6 

			global_u16p = (u16*)str_buffer;  //便用取一次取16位

			for(i=0;i<y_offset;i++)	
			{
				for(j=0;j<(x_offset);j++)	
				{
					pix = i*(x_wide)+j;  //像素加1
					if(pix%2048==0)
					{
						if(y_offset*x_wide-pix>2048)
						{
							spi_size= 4096;
						}
						else
						{
							spi_size= (y_offset*x_wide-pix)*2;
						}
						FLASH2_GPIOSPI_Read (addr+offset+(pix<<1), str_buffer, spi_size);  //读像素内容
						pix_temp = pix;
					}

					// 	 				g16_temp = str_buffer[((pix-pix_temp)<<1)+1];	
					// 	 				g16_temp <<= 8;		
					// 	 				g16_temp += str_buffer[((pix-pix_temp)<<1)];		

					g16_temp =global_u16p[pix-pix_temp];
					LCD_WE_RAM(g16_temp);
				}
			}			 

			LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
			tft_cs_disable(cs);				
		}
	}
}
//-------------------------------------------------------------------
void display_flash_BMPE (u16 x,u16 y,u8 area,u8 num,u8 cs)
{
	u16 i;
	u32 f_addr,f_size;

	if(area==1)//二维码区
	{
		FLASH2_GPIOSPI_Read (Addr_01min, str_buffer, 256);
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
			if(str_buffer[4+18*i+1]==num)//确认文件号
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

		if(f_size>0)
		{
			display_flash_BMP (x,y,f_addr,cs);
		}
	}
	else
		if(area==2)//文字区
		{
			FLASH2_GPIOSPI_Read (Addr_02min, str_buffer, 1024);
			f_addr = 0;
			f_size = 0;
			if(str_buffer[1]>52)
			{	
				str_buffer[1] = 52;	
			}

			for(i=0;i<str_buffer[1];i++)
			{
				if(str_buffer[0]!=frame_headerC)	
				{		break;	}			
				if(str_buffer[4+18*i+1]==num)//确认文件号
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

			if(f_size>0)
			{
				display_flash_BMP (x,y,f_addr,cs);
			}

		}
		else
			if(area==3)//彩图区
			{
				FLASH2_GPIOSPI_Read (Addr_03min, str_buffer, 1024);
				f_addr = 0;
				f_size = 0;
				if(str_buffer[1]>52)
				{	
					str_buffer[1] = 52;	
				}

				for(i=0;i<str_buffer[1];i++)
				{
					if(str_buffer[0]!=frame_headerC)	
					{		break;	}			
					if(str_buffer[4+18*i+1]==num)//确认文件号
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

				if(f_size>0)
				{
					display_flash_BMP (x,y,f_addr,cs);
				}
			}
			else
				if(area==4)//彩图区
				{
					FLASH2_GPIOSPI_Read (Addr_04min, str_buffer, 1024);
					f_addr = 0;
					f_size = 0;
					if(str_buffer[1]>52)
					{	
						str_buffer[1] = 52;	
					}

					for(i=0;i<str_buffer[1];i++)
					{
						if(str_buffer[0]!=frame_headerC)	
						{		break;	}			
						if(str_buffer[4+18*i+1]==num)//确认文件号
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

					if(f_size>0)
					{
						display_flash_BMP (x,y,f_addr,cs);
					}
				}
}

//-------------------------------------------------------------------
void display_IC_BMP (u16 x,u16 y,const u8 *p,u8 cs)  
{
	u8 temp_char,k;//
	u32 i=0,j=0,n=0;//
	u32 offset,x_offset,y_offset;  //
	u32 x_wide;  //图片实际取点的宽度 。为32的倍数
	u32 x_offset1;  //实际要的窗口长宽
	u32 pix;  //显示到的子节数
	unsigned int pix_temp;

	//		FLASH2_GPIOSPI_Read (addr, str_buffer, 64);
	for(n=0;n<64;n++)
	{
		str_buffer[n] = p[n];
	}
	if(str_buffer[28] ==0x01)   //1bit
	{
		offset = str_buffer[13];
		offset <<= 8;
		offset += str_buffer[12];
		offset <<= 8;
		offset += str_buffer[11];
		offset <<= 8;
		offset += str_buffer[10];

		x_offset = str_buffer[21];
		x_offset <<= 8;
		x_offset += str_buffer[20];
		x_offset <<= 8;
		x_offset += str_buffer[19];
		x_offset <<= 8;
		x_offset += str_buffer[18];

		if((x_offset%32)!=0)
		{
			x_wide = x_offset + 32;
			x_wide &= ~0x1f;			
		}
		else
		{
			x_wide = x_offset;
		}
		y_offset = str_buffer[25];
		y_offset <<= 8;
		y_offset += str_buffer[24];
		y_offset <<= 8;
		y_offset += str_buffer[23];
		y_offset <<= 8;
		y_offset += str_buffer[22];

		if((str_buffer[0]!='B')||(str_buffer[1]!='M'))  //内容认为无效
		{
			x_offset = 0;
		}
		if(y_offset>320)  //内容认为无效
		{
			x_offset = 0;
		}

		if((x_offset>0)&&(x_offset<=240))
		{
			tft_cs_enable(cs);

			x_offset1 = x_offset;				
			offset +=0;				
			//	LCD_WindowMax (x, y, x+x_offset-1, y+y_offset-1);       //设置窗口	
			LCD_WindowMax (x, y, x+y_offset-1, y+x_offset1-1);       //设置窗口					
			LCD_WriteRAM_Prepare();  				//开始写入GRAM*6 	

			for(i=0;i<y_offset;i++)	
			{
				for(j=0;j<(x_offset1/8);j++)	
				{
					pix = i*(x_wide/8)+j; 
					if(pix%2048==0)
					{
						//				FLASH2_GPIOSPI_Read (addr+offset+pix, str_buffer, 2048);  //读像素内容2K
						for(n=0;n<2048;n++)
						{
							str_buffer[n] = p[offset+pix+n];
						}
						pix_temp = pix;
					}
					temp_char = 	str_buffer[pix-pix_temp];

					for(k=0;k<8;k++)
					{
						if ( ((temp_char >> (7-k)) &0x01) == 0x01)
						{
							g16_temp=0x0000; // 字符颜色
						}
						else
						{
							g16_temp=0xffff; // 背景颜色
						}

						LCD_WE_RAM(g16_temp);
					}
				}
			}			 
			LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
			tft_cs_disable(cs);				
		}

	}

	if(str_buffer[28] ==0x10)   //16bit
	{
		offset = str_buffer[13];
		offset <<= 8;
		offset += str_buffer[12];
		offset <<= 8;
		offset += str_buffer[11];
		offset <<= 8;
		offset += str_buffer[10];

		x_offset = str_buffer[21];
		x_offset <<= 8;
		x_offset += str_buffer[20];
		x_offset <<= 8;
		x_offset += str_buffer[19];
		x_offset <<= 8;
		x_offset += str_buffer[18];

		if((x_offset%2)!=0)  //规定扫描一行数据必是4字节的倍数，不足的补0
		{
			x_wide = x_offset + 2;
			x_wide &= ~0x01;			
		}
		else
		{
			x_wide = x_offset;
		}
		y_offset = str_buffer[25];
		y_offset <<= 8;
		y_offset += str_buffer[24];
		y_offset <<= 8;
		y_offset += str_buffer[23];
		y_offset <<= 8;
		y_offset += str_buffer[22];

		if((str_buffer[0]!='B')||(str_buffer[1]!='M'))  //内容认为无效
		{
			x_offset = 0;
		}
		if(y_offset>320)  //内容认为无效
		{
			x_offset = 0;
		}

		if((x_offset>0)&&(x_offset<=240))
		{
			tft_cs_enable(cs);

			x_offset1 = x_offset;
			LCD_WindowMax (x, y, x+y_offset-1, y+x_offset1-1);       //设置窗口					
			LCD_WriteRAM_Prepare();  				//开始写入GRAM*6 

			global_u16p = (u16*)str_buffer;  //便用取一次取16位

			for(i=0;i<y_offset;i++)	
			{
				for(j=0;j<(x_offset);j++)	
				{
					pix = i*(x_wide)+j;  //像素加1
					if(pix%2048==0)
					{
						//					FLASH2_GPIOSPI_Read (addr+offset+(pix<<1), str_buffer, 4096);  //读像素内容
						for(n=0;n<4096;n++)
						{
							str_buffer[n] = p[offset+(pix<<1)+n];
						}
						pix_temp = pix;
					}

					// 				g16_temp = str_buffer[pix+1-pix_temp];	
					// 				g16_temp <<= 8;		
					// 				g16_temp += str_buffer[pix-pix_temp];		

					g16_temp =global_u16p[pix-pix_temp];
					LCD_WE_RAM(g16_temp);
				}
			}			 

			LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
			tft_cs_disable(cs);				
		}
	}
}

//------------------------------------------------------------
void tft_1bitdeep_EWM (u16 x,u16 y, u8 *array,u16 PenColor, u16 BackColor,u8 cs)
{
	u8 temp_char,k;//
	u32 i=0,j=0;//
	u32 offset,x_offset,y_offset,x_offset1;
	offset = array[13];
	offset <<= 8;
	offset += array[12];
	offset <<= 8;
	offset += array[11];
	offset <<= 8;
	offset += array[10];

	x_offset = array[21];
	x_offset <<= 8;
	x_offset += array[20];
	x_offset <<= 8;
	x_offset += array[19];
	x_offset <<= 8;
	x_offset += array[18];

	y_offset = array[25];
	y_offset <<= 8;
	y_offset += array[24];
	y_offset <<= 8;
	y_offset += array[23];
	y_offset <<= 8;
	y_offset += array[22];

	if((array[0]!='B')||(array[1]!='M'))  //内容认为无效
	{
		x_offset = 0;
	}

	tft_cs_enable(cs);


	if((x_offset>0)&&(x_offset<=240))
	{
		x_offset1 = x_offset;

		offset +=0;
		//	LCD_WindowMax (x, y, x+x_offset-1, y+y_offset-1);       //设置窗口	
		LCD_WindowMax (x, y, x+y_offset-1, y+x_offset1-1);       //设置窗口	
		LCD_WriteRAM_Prepare();                       //开始写入GRAM*6 	   	   

		for(i=0;i<y_offset;i++)	
		{
			for(j=0;j<(x_offset1/8);j++)	
			{
				temp_char = 	array[offset+i*(x_offset/8)+j];

				for(k=0;k<8;k++)
				{
					if ( ((temp_char >> (7-k)) &0x01) == 0x01)
					{
						g16_temp=PenColor; // 字符颜色
					}
					else
					{
						g16_temp=BackColor; // 背景颜色
					}

					LCD_WE_RAM(g16_temp);
				}
			}
		}

		LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
	}

	tft_cs_disable(cs);

}
//--------------------------------------------------------------------
void tft_16bitdeep_BMP (u16 x,u16 y,const u8 *array)
{
	u32 i=0,n=0;//
	u32 offset,x_offset,y_offset;
	offset = array[13];
	offset <<= 8;
	offset += array[12];
	offset <<= 8;
	offset += array[11];
	offset <<= 8;
	offset += array[10];

	x_offset = array[21];
	x_offset <<= 8;
	x_offset += array[20];
	x_offset <<= 8;
	x_offset += array[19];
	x_offset <<= 8;
	x_offset += array[18];

	y_offset = array[25];
	y_offset <<= 8;
	y_offset += array[24];
	y_offset <<= 8;
	y_offset += array[23];
	y_offset <<= 8;
	y_offset += array[22];

	if(SET_XY==1)
	{
		LCD_WindowMax (x, y, x+x_offset-1, y+y_offset-1);       //设置窗口	
		LCD_WriteRAM_Prepare();                       //开始写入GRAM*6 	   	   
		n=(u32)(x_offset)*(y_offset);  

		//		i=offset+n*2;
		i=offset;
		while(n--)
		{
			g16_temp = array[i+1];	
			g16_temp <<= 8;		
			g16_temp += array[i];		
			i++;
			i++;
			LCD_WE_RAM(g16_temp);
		}               //显示所填充的颜色

		LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
	}
	else
		if(SET_XY==2)
		{
			LCD_WindowMax (x, y, x+x_offset-1, y+y_offset-1);       //设置窗口	
			LCD_WriteRAM_Prepare();                       //开始写入GRAM*6 	   	   
			n=(u32)(x_offset)*(y_offset);  

			//		i=offset+n*2;
			i=offset;
			while(n--)
			{
				g16_temp = array[i+1];	
				g16_temp <<= 8;		
				g16_temp += array[i];		
				i++;
				i++;
				LCD_WE_RAM(g16_temp);
			}               //显示所填充的颜色

			LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
		}
		else
			if(SET_XY==3)
			{
				LCD_WindowMax (x, y, x+y_offset-1,y+x_offset-1);       //设置窗口	
				LCD_WriteRAM_Prepare();                       //开始写入GRAM*6 	   	   
				n=(u32)(x_offset)*(y_offset);  

				//		i=offset+n*2;
				i=offset;
				while(n--)
				{
					g16_temp = array[i+1];	
					g16_temp <<= 8;		
					g16_temp += array[i];		
					i++;
					i++;
					LCD_WE_RAM(g16_temp);
				}               //显示所填充的颜色

				LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
			}
			else
				if(SET_XY==4)
				{
					LCD_WindowMax (x, y, y+y_offset-1,x+x_offset-1);       //设置窗口	
					LCD_WriteRAM_Prepare();                       //开始写入GRAM*6 	   	   
					n=(u32)(x_offset)*(y_offset);  

					//		i=offset+n*2;
					i=offset;
					while(n--)
					{
						g16_temp = array[i+1];	
						g16_temp <<= 8;		
						g16_temp += array[i];		
						i++;
						i++;
						LCD_WE_RAM(g16_temp);
					}               //显示所填充的颜色

					LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
				}
	//	LCD_WindowMax (x, y, x+x_offset-1, y+y_offset-1);       //设置窗口	
}

void tft_Clear(u16 x,u16 y,u16 x_offset,u16 y_offset,u16 Color,u8 cs)
{
	//   u16 temp_x=0;
	//   u16 temp_y=0;
	tft_cs_enable(cs);
	LCD_Fill(x, y, x+x_offset-1, y+y_offset-1,  Color);
	tft_cs_disable(cs);
}

void tft_ShowHz(u16 x, u16 y, u8 *pcStr, u16 PenColor, u16 BackColor)
{	
	u16 temph,templ;
	u8 temp;
	u8 pos,t;
	u32 offset;

	//  if(x>MAX_CHAR_POSX||y>MAX_CHAR_POSY)return;		 

	LCD_WR_REG(0x0036);             //rotation
	LCD_WE_RAM(0x00E0);//dwgl				//横屏1
	//		 LCD_WindowMax(x,y,x+16-1,y+16-1);	        //设置窗口  横屏1
	LCD_WindowMax(x,y,x+16-1,y+16-1);	           //设置窗口	dwgl	竖屏1

	LCD_WriteRAM_Prepare();                                //开始写入GRAM   
	temph = *pcStr;	
	temph -= 0xb0; 
	templ = *(pcStr+1);
	templ -= 0xa1; 
	offset = temph*94*32+templ*32;
	for(pos=0;pos<32;pos+=2)
	{
		temp=HzLib[offset+pos+1];		                     //调用1616字体

		for(t=0;t<8;t++)
		{                 
			if(temp&0x01)			                                   //从低位开始
			{
				LCD_WE_RAM(PenColor);                             //画字体颜色 一个点
			}
			else 
			{
				LCD_WE_RAM(BackColor);	                           //画背景颜色 一个点      
			}
			temp>>=1; 
		}

		temp=HzLib[offset+pos];		                     //调用1608字体

		for(t=0;t<8;t++)
		{                 
			if(temp&0x01)			                                   //从低位开始
			{
				LCD_WE_RAM(PenColor);                             //画字体颜色 一个点
			}
			else 
			{
				LCD_WE_RAM(BackColor);	                           //画背景颜色 一个点      
			}
			temp>>=1; 
		}
	}			
	LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
	LCD_WR_REG(0x0036);                                //rotation
	LCD_WE_RAM(0x0023);//dwgl				//横屏1

}

void tft_ShowHzE(u16 x, u16 y, u8 *pcStr, u16 PenColor, u16 BackColor)
{	
	u16 temph,templ;
	u8 temp;
	u8 pos,t;
	u32 offset;

	LCD_WindowMax(x,y,x+16-1,y+16-1);	           //设置窗口	dwgl	竖屏1

	LCD_WriteRAM_Prepare();                                //开始写入GRAM   
	temph = *pcStr;	
	temph -= 0xb0; 
	templ = *(pcStr+1);
	templ -= 0xa1; 
	offset = temph*94*32+templ*32;
	pos=32;
	while(pos)
		//	for(pos=0;pos<32;pos+=2)
	{
		pos--; 
		pos--; 
		temp=HzLib[offset+pos];		                     //调用1616字体

		for(t=0;t<8;t++)
		{                 
			if(temp&0x80)			                                   //从低位开始
			{
				LCD_WE_RAM(PenColor);                             //画字体颜色 一个点
			}
			else 
			{
				LCD_WE_RAM(BackColor);	                           //画背景颜色 一个点      
			}
			temp<<=1; 
		}

		temp=HzLib[offset+pos+1];		                     //调用1608字体

		for(t=0;t<8;t++)
		{                 
			if(temp&0x80)			                                   //从低位开始
			{
				LCD_WE_RAM(PenColor);                             //画字体颜色 一个点
			}
			else 
			{
				LCD_WE_RAM(BackColor);	                           //画背景颜色 一个点      
			}
			temp<<=1; 
		}
	}			
	LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小

}
//--------------------------------------------------------------------------------------
void tft_ShowChar(unsigned int x, unsigned int y, unsigned char num,u16 PenColor, u16 BackColor)
{
	u8 temp;
	u32 offset;
	u8 pos,t;
	if(x>MAX_CHAR_POSX||y>MAX_CHAR_POSY)return;		 

	LCD_WR_REG(0x0036);                                //rotation
	LCD_WE_RAM(0x00E0);//dwgl				//横屏1
	//		 LCD_WindowMax(x,y,x+8-1,y+16-1);	        //设置窗口  横屏1
	LCD_WindowMax(x,y,x+16-1,y+8-1);	           //设置窗口	dwgl	竖屏1

	LCD_WriteRAM_Prepare();                                //开始写入GRAM   
	num=num-' ';                                            //得到偏移后的值 
	offset =num*16;                                            //得到偏移后的值 
	for(pos=0;pos<16;pos++)
	{
		temp=AsciiLib[offset+pos];		                     //调用1608字体

		for(t=0;t<8;t++)
		{                 
			if(temp&0x01)			                                   //从低位开始
			{
				LCD_WE_RAM(PenColor);                             //画字体颜色 一个点
			}
			else 
			{
				LCD_WE_RAM(BackColor);	                           //画背景颜色 一个点      
			}
			temp>>=1; 
		}
	}			
	LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
	LCD_WR_REG(0x0036);                                //rotation
	LCD_WE_RAM(0x0023);//dwgl				//横屏1
}
void tft_ShowCharE(unsigned int x, unsigned int y, unsigned char num,u16 PenColor, u16 BackColor)
{
	u8 temp;
	u32 offset;
	u8 pos,t;
	if(x>MAX_CHAR_POSX||y>MAX_CHAR_POSY)return;		 

	LCD_WindowMax(x,y,x+16-1,y+8-1);	           //设置窗口	dwgl	竖屏1

	LCD_WriteRAM_Prepare();                                //开始写入GRAM   
	num=num-' ';                                            //得到偏移后的值 
	offset =num*16;                                            //得到偏移后的值 
	pos=16;
	while(pos)
		//	for(pos=0;pos<16;pos++)
	{
		pos--;
		temp=AsciiLib[offset+pos];		                     //调用1608字体

		for(t=0;t<8;t++)
		{                 
			if(temp&0x80)			                                   //从低位开始
			{
				LCD_WE_RAM(PenColor);                             //画字体颜色 一个点
			}
			else 
			{
				LCD_WE_RAM(BackColor);	                           //画背景颜色 一个点      
			}
			temp<<=1; 
		}
	}			
	LCD_WindowMax(0,0,HDP,VDP);      //恢复窗体大小
}
//--------------------------------------------------------------------------------------
void tft_DisplayStr(unsigned int x, unsigned int y, unsigned char *s,u16 PenColor, u16 BackColor,u8 cs)
{
	tft_cs_enable(cs);
	while (*s) 
	{  	
		if(*s>0x80)                                            //显示汉字
		{
			if(y+16<240)
			{
				y += 16;
			}
			else
			{
				x -= 16;
				y = 16;
			}
			//tft_ShowHz(x, y, s, PenColor, BackColor);
			tft_ShowHzE(x, y-16, s, PenColor, BackColor);
			s += 2;
		}
		else                                                 //非汉字
		{	
			if(y+8<240)
			{
				y += 8;
			}
			else
			{
				x -= 16;
				y = 8;
			}
			//tft_ShowChar(x, y, *s++, PenColor, BackColor);
			tft_ShowCharE(x, y-8, *s++, PenColor, BackColor);
			//s += 1;
		}
	}
	tft_cs_disable(cs);
}

void tft_cs_enable(u8 cs)
{
	if((cs&0x01)==1)
	{
		GPIO_ResetBits(LCD_CS1_PORT, LCD_CS1_PIN);
	}
	if((cs&0x02)==2)
	{
		GPIO_ResetBits(LCD_CS2_PORT, LCD_CS2_PIN);
	}
}
void tft_cs_disable(u8 cs)
{
	if((cs&0x01)==1)
	{
		GPIO_SetBits(LCD_CS1_PORT, LCD_CS1_PIN);
	}
	if((cs&0x02)==2)
	{
		GPIO_SetBits(LCD_CS2_PORT, LCD_CS2_PIN);
	}
}
