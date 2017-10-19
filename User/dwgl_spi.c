//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*****************************************************************************
#include "stm32f10x.h"

//变量
u8  *SPI1_Receive_Pointer;
u16  SPI1_Receive_BufSize;
u16  SPI1_Receive_Length;
u8  *SPI1_Trans_Pointer;
u16  SPI1_Trans_BufSize;
u16  SPI1_Trans_Length;
u8   SPI1_TX_State;
u8   SPI1_RX_State;
u8   SPI1_Error;

u8  *SPI2_Receive_Pointer;
u16  SPI2_Receive_BufSize;
u16  SPI2_Receive_Length;
u8  *SPI2_Trans_Pointer;
u16  SPI2_Trans_BufSize;
u16  SPI2_Trans_Length;
u8   SPI2_TX_State;
u8   SPI2_RX_State;
u8   SPI2_Error;

//*****************************************************************************
static void SPI1_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1, ENABLE);

	//初始引脚
	GPIO_InitStructure.GPIO_Pin = A7_SPI1_MOSI_PIN|A5_SPI1_CLK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = A6_SPI1_MISO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//初始化片选引脚
	GPIO_InitStructure.GPIO_Pin = A4_SPI1_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_SetBits(GPIOA, A4_SPI1_CS_PIN);//PA4/NSS  失能

	/* SPI Config */
	SPI_I2S_DeInit(SPI1);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	//SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	//SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	/* 开中断 */
	//SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);

	/* Enable SPI2 */
	SPI_Cmd(SPI1, ENABLE);
}

void SPI2_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//初始引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//初始化片选引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);

	/* SPI Config */ 								  
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);

	/* 开中断 */
	//SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);

	/* Enable SPI2 */
	SPI_Cmd(SPI2, ENABLE);
}

//*****************************************************************************
void SPI1_Send_Data(u8 * txp,u8 * rxp,u16 len)
{
	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	/* Send byte through the SPI2 peripheral */
	//  SPI_I2S_SendData(SPI1, data);
	SPI1->DR = *(txp+len);

	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

	/* Return the byte read from the SPI bus */
	//  return SPI_I2S_ReceiveData(SPI1);
	*(rxp+len) = SPI1->DR;
}

//*****************************************************************************
void SPI1_Send_Data1(u8 * txp,u8 * rxp,u16 len)
{
	if(SPI1_Trans_Length==0)
	{
		SPI1_Receive_Pointer = rxp;
		SPI1_Trans_Pointer = txp;
		SPI1_Trans_Length = len;
	}
}

//*****************************************************************************
void SPI2_Send_Data(u8 * p,u16 len)
{

}

/*******************************************************************************
 * Function Name  : Flash_ReadWriteByte
 * Description    : 通过硬件SPI发送一个字节到SST25VF016B
 * Input          : - data: 发送的数据
 * Output         : None
 * Return         : SST25VF016B 返回的数据
 * Attention		 : None
 *******************************************************************************/
uint8_t Flash_ReadWriteByte(uint8_t data)		
{
	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	/* Send byte through the SPI2 peripheral */
	SPI_I2S_SendData(SPI1, data);

	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI1);                                              
}

//*****************************************************************************
void SPI1_IRQHandler(void)
{
	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) != RESET)//接收到了数据
	{
		if(SPI1_Trans_Length>0)
		{
			SPI1->DR = SPI1_Trans_Pointer[SPI1_Receive_Length];
			SPI1_Receive_Pointer[SPI1_Receive_Length] = SPI1->DR;
			SPI1_Trans_Length--;
			SPI1_Receive_Length++;
		}
		else
		{
			SPI1_TX_State = 1;
		}
		SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_TXE);
	}
}

//*****************************************************************************
void SPI2_IRQHandler(void)  
{

}

//*****************************************************************************
// #define SPI2_MOSI_PIN			GPIO_Pin_15
// #define SPI2_MOSI_PORT			GPIOB
// #define SPI2_MISO_PIN			GPIO_Pin_14
// #define SPI2_MISO_PORT			GPIOB
// #define SPI2_CLK_PIN			  GPIO_Pin_13
// #define SPI2_CLK_PORT			GPIOB

void GPIOSPI_Send_BYTE(u8 data)
{
	u16 tx_data,t_data,T_NY;

	tx_data = data;
	tx_data = (tx_data<<8);

	T_NY = (tx_data&0X8000);   //bit7

	t_data = (GPIOB->ODR&0X7FFF);
	t_data |= T_NY;
	GPIOB->ODR = t_data;
	GPIOB->BSRR = SPI2_CLK_PIN;	

	tx_data <<= 1;	
	T_NY = (tx_data&0X8000);   //bit6
	GPIOB->BRR = SPI2_CLK_PIN;	

	t_data = (GPIOB->ODR&0X7FFF);
	t_data |= T_NY;
	GPIOB->ODR = t_data;
	GPIOB->BSRR = SPI2_CLK_PIN;	

	tx_data <<= 1;	
	T_NY = (tx_data&0X8000);   //bit5
	GPIOB->BRR = SPI2_CLK_PIN;	

	t_data = (GPIOB->ODR&0X7FFF);
	t_data |= T_NY;
	GPIOB->ODR = t_data;
	GPIOB->BSRR = SPI2_CLK_PIN;	

	tx_data <<= 1;	
	T_NY = (tx_data&0X8000);   //bit4
	GPIOB->BRR = SPI2_CLK_PIN;	

	t_data = (GPIOB->ODR&0X7FFF);
	t_data |= T_NY;
	GPIOB->ODR = t_data;
	GPIOB->BSRR = SPI2_CLK_PIN;	

	tx_data <<= 1;	
	T_NY = (tx_data&0X8000);   //bit3
	GPIOB->BRR = SPI2_CLK_PIN;	

	t_data = (GPIOB->ODR&0X7FFF);
	t_data |= T_NY;
	GPIOB->ODR = t_data;
	GPIOB->BSRR = SPI2_CLK_PIN;	

	tx_data <<= 1;	
	T_NY = (tx_data&0X8000);   //bit2
	GPIOB->BRR = SPI2_CLK_PIN;	

	t_data = (GPIOB->ODR&0X7FFF);
	t_data |= T_NY;
	GPIOB->ODR = t_data;
	GPIOB->BSRR = SPI2_CLK_PIN;	

	tx_data <<= 1;	
	T_NY = (tx_data&0X8000);   //bit1
	GPIOB->BRR = SPI2_CLK_PIN;	

	t_data = (GPIOB->ODR&0X7FFF);
	t_data |= T_NY;
	GPIOB->ODR = t_data;
	GPIOB->BSRR = SPI2_CLK_PIN;	

	tx_data <<= 1;	
	T_NY = (tx_data&0X8000);   //bit0
	GPIOB->BRR = SPI2_CLK_PIN;	

	t_data = (GPIOB->ODR&0X7FFF);
	t_data |= T_NY;
	GPIOB->ODR = t_data;
	GPIOB->BSRR = SPI2_CLK_PIN;	

	tx_data <<= 1;	
	T_NY = (tx_data&0X8000);   //bit
	GPIOB->BRR = SPI2_CLK_PIN;	

	//GPIO_ResetBits(SPI2_MOSI_PORT, SPI2_MOSI_PIN);
	//GPIO_SetBits(SPI2_MISO_PORT, SPI2_MISO_PIN);
	//GPIO_ResetBits(SPI2_MISO_PORT, SPI2_MISO_PIN);
}
void GPIOSPI_Send1_BYTE(u8 data)
{
	u8 tx_data,T_NY;
	
	tx_data = data;

	T_NY = (tx_data&0X80);   //bit7
	if(T_NY>0)
	{
		GPIOB->BSRR = SPI2_MOSI_PIN;	
	}
	else
	{
		GPIOB->BRR = SPI2_MOSI_PIN;	
	}
	GPIOB->BSRR = SPI2_CLK_PIN;	
	tx_data <<= 1;

	T_NY = (tx_data&0X80);   //bit6
	GPIOB->BRR = SPI2_CLK_PIN;	

	if(T_NY>0)
	{
		GPIOB->BSRR = SPI2_MOSI_PIN;	
	}
	else
	{
		GPIOB->BRR = SPI2_MOSI_PIN;	
	}
	GPIOB->BSRR = SPI2_CLK_PIN;	
	tx_data <<= 1;

	T_NY = (tx_data&0X80);   //bit5
	GPIOB->BRR = SPI2_CLK_PIN;	

	if(T_NY>0)
	{
		GPIOB->BSRR = SPI2_MOSI_PIN;	
	}
	else
	{
		GPIOB->BRR = SPI2_MOSI_PIN;	
	}
	GPIOB->BSRR = SPI2_CLK_PIN;	
	tx_data <<= 1;

	T_NY = (tx_data&0X80);   //bit4
	GPIOB->BRR = SPI2_CLK_PIN;	

	if(T_NY>0)
	{
		GPIOB->BSRR = SPI2_MOSI_PIN;	
	}
	else
	{
		GPIOB->BRR = SPI2_MOSI_PIN;	
	}
	GPIOB->BSRR = SPI2_CLK_PIN;	
	tx_data <<= 1;

	T_NY = (tx_data&0X80);   //bit3
	GPIOB->BRR = SPI2_CLK_PIN;	

	if(T_NY>0)
	{
		GPIOB->BSRR = SPI2_MOSI_PIN;	
	}
	else
	{
		GPIOB->BRR = SPI2_MOSI_PIN;	
	}
	GPIOB->BSRR = SPI2_CLK_PIN;	
	tx_data <<= 1;

	T_NY = (tx_data&0X80);   //bit2
	GPIOB->BRR = SPI2_CLK_PIN;	

	if(T_NY>0)
	{
		GPIOB->BSRR = SPI2_MOSI_PIN;	
	}
	else
	{
		GPIOB->BRR = SPI2_MOSI_PIN;	
	}
	GPIOB->BSRR = SPI2_CLK_PIN;	
	tx_data <<= 1;

	T_NY = (tx_data&0X80);   //bit1
	GPIOB->BRR = SPI2_CLK_PIN;	

	if(T_NY>0)
	{
		GPIOB->BSRR = SPI2_MOSI_PIN;	
	}
	else
	{
		GPIOB->BRR = SPI2_MOSI_PIN;	
	}
	GPIOB->BSRR = SPI2_CLK_PIN;	
	tx_data <<= 1;

	T_NY = (tx_data&0X80);   //bit0
	GPIOB->BRR = SPI2_CLK_PIN;	

	if(T_NY>0)
	{
		GPIOB->BSRR = SPI2_MOSI_PIN;	
	}
	else
	{
		GPIOB->BRR = SPI2_MOSI_PIN;	
	}
	GPIOB->BSRR = SPI2_CLK_PIN;	
	tx_data <<= 1;

	T_NY = (tx_data&0X80);   //bitx
	GPIOB->BRR = SPI2_CLK_PIN;
}

//*****************************************************************************
u8 GPIOSPI_SR_BYTE(u8 data)
{
	u8 rx_data,tx_data,T_NY;
	u16 R_NY;
	tx_data = data;
	rx_data =0;

	T_NY = (tx_data&0X80);   //bit7
	if(T_NY>0)
	{
		GPIOA->BSRR = SPI2_MOSI_PIN;	
	}
	else
	{
		GPIOA->BRR = SPI2_MOSI_PIN;	
	}
	tx_data <<= 1;
	rx_data <<= 1;
	GPIOA->BSRR = SPI2_CLK_PIN;	
	R_NY = GPIOA->IDR & SPI2_MISO_PIN;
	if(R_NY>0)
	{
		rx_data++;
	}
	else
	{
		R_NY = rx_data;
	}	
	GPIOA->BRR = SPI2_CLK_PIN;	

	T_NY = (tx_data&0X80);   //bit6
	if(T_NY>0)
	{
		GPIOA->BSRR = SPI2_MOSI_PIN;	
	}
	else
	{
		GPIOA->BRR = SPI2_MOSI_PIN;	
	}
	tx_data <<= 1;
	rx_data <<= 1;
	GPIOA->BSRR = SPI2_CLK_PIN;	
	R_NY = GPIOA->IDR & SPI2_MISO_PIN;
	if(R_NY>0)
	{
		rx_data++;
	}
	else
	{
		R_NY = rx_data;
	}	
	GPIOA->BRR = SPI2_CLK_PIN;	

	T_NY = (tx_data&0X80);   //bit5
	if(T_NY>0)
	{
		GPIOA->BSRR = SPI2_MOSI_PIN;	
	}
	else
	{
		GPIOA->BRR = SPI2_MOSI_PIN;	
	}
	tx_data <<= 1;
	rx_data <<= 1;
	GPIOA->BSRR = SPI2_CLK_PIN;	
	R_NY = GPIOA->IDR & SPI2_MISO_PIN;
	if(R_NY>0)
	{
		rx_data++;
	}
	else
	{
		R_NY = rx_data;
	}	
	GPIOA->BRR = SPI2_CLK_PIN;	

	T_NY = (tx_data&0X80);   //bit4
	if(T_NY>0)
	{
		GPIOA->BSRR = SPI2_MOSI_PIN;	
	}
	else
	{
		GPIOA->BRR = SPI2_MOSI_PIN;	
	}
	tx_data <<= 1;
	rx_data <<= 1;
	GPIOA->BSRR = SPI2_CLK_PIN;	
	R_NY = GPIOA->IDR & SPI2_MISO_PIN;
	if(R_NY>0)
	{
		rx_data++;
	}
	else
	{
		R_NY = rx_data;
	}	
	GPIOA->BRR = SPI2_CLK_PIN;	

	T_NY = (tx_data&0X80);   //bit3
	if(T_NY>0)
	{
		GPIOA->BSRR = SPI2_MOSI_PIN;	
	}
	else
	{
		GPIOA->BRR = SPI2_MOSI_PIN;	
	}
	tx_data <<= 1;
	rx_data <<= 1;
	GPIOA->BSRR = SPI2_CLK_PIN;	
	R_NY = GPIOA->IDR & SPI2_MISO_PIN;
	if(R_NY>0)
	{
		rx_data++;
	}
	else
	{
		R_NY = rx_data;
	}	
	GPIOA->BRR = SPI2_CLK_PIN;	

	T_NY = (tx_data&0X80);   //bit2
	if(T_NY>0)
	{
		GPIOA->BSRR = SPI2_MOSI_PIN;	
	}
	else
	{
		GPIOA->BRR = SPI2_MOSI_PIN;	
	}
	tx_data <<= 1;
	rx_data <<= 1;
	GPIOA->BSRR = SPI2_CLK_PIN;	
	R_NY = GPIOA->IDR & SPI2_MISO_PIN;
	if(R_NY>0)
	{
		rx_data++;
	}
	else
	{
		R_NY = rx_data;
	}	
	GPIOA->BRR = SPI2_CLK_PIN;	

	T_NY = (tx_data&0X80);   //bit1
	if(T_NY>0)
	{
		GPIOA->BSRR = SPI2_MOSI_PIN;	
	}
	else
	{
		GPIOA->BRR = SPI2_MOSI_PIN;	
	}
	tx_data <<= 1;
	rx_data <<= 1;
	GPIOA->BSRR = SPI2_CLK_PIN;	
	R_NY = GPIOA->IDR & SPI2_MISO_PIN;
	if(R_NY>0)
	{
		rx_data++;
	}
	else
	{
		R_NY = rx_data;
	}	
	GPIOA->BRR = SPI2_CLK_PIN;	

	T_NY = (tx_data&0X80);   //bit0
	if(T_NY>0)
	{
		GPIOA->BSRR = SPI2_MOSI_PIN;	
	}
	else
	{
		GPIOA->BRR = SPI2_MOSI_PIN;	
	}
	tx_data <<= 1;
	rx_data <<= 1;
	GPIOA->BSRR = SPI2_CLK_PIN;	
	R_NY = GPIOA->IDR & SPI2_MISO_PIN;
	if(R_NY>0)
	{
		rx_data++;
	}
	else
	{
		R_NY = rx_data;
	}	
	GPIOA->BRR = SPI2_CLK_PIN;	

	//NY = GPIO_ReadInputDataBit(SPI2_MISO_PORT,SPI2_MISO_PIN);	
	//GPIO_ResetBits(SPI2_MOSI_PORT, SPI2_MOSI_PIN);
	//GPIO_SetBits(SPI2_MISO_PORT, SPI2_MISO_PIN);
	//GPIO_ResetBits(SPI2_MISO_PORT, SPI2_MISO_PIN);

	return rx_data;

}
//*****************************************************************************
void GPIOSPI_Send_Data(u8 * p,u16 len)
{
	u16 i;
	for(i=0;i<len;i++)
	{
		GPIOSPI_Send_BYTE(*p);
		p++;
	}
}

//*****************************************************************************
void GPIOSPI_Send1_Data(u8 * p,u16 len)
{
	u16 i;
	for(i=0;i<len;i++)
	{
		GPIOSPI_Send1_BYTE(*p);
		p++;
	}
}
//*****************************************************************************
void GPIOSPI_SR_Data(u8 * tx,u8 * rx,u16 len)
{
	u16 i;
	for(i=0;i<len;i++)
	{
		*rx= GPIOSPI_SR_BYTE(*tx);
		tx++;
		rx++;
	}
}

void FLASH2_GPIOSPI_Read (unsigned int write_addr, unsigned char *buffer, unsigned int len)//FLASH-读出LEN个字节-
{ 
	//u8  i;
	unsigned char addr1,addr2,addr3;
	addr1 = ((write_addr&0xff0000)>>16);
	addr2 = ((write_addr&0xff00)>>8);
	addr3 = ((write_addr&0xff));
	//RAM_CS   = 1;FLASH_CS   = 0;  //片选使能
	//delay_cycle(2);  
	FLASH2_CSSelect(Enable);                  

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = FLASH_CMD_READ;       //读取数据

	//FLASH_u8temp=SPI1->DR;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = addr1;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = addr2;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = addr3;

	{
		delay_cycle(500);  
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		FLASH_u8temp=SPI1->DR;			
		SPI1_DMARecive( buffer, len );			
		while(len>0)
		{
			//buffer[i]= FLASH2_ReadByte();
			while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
			SPI1->DR = 0;      //读状态寄存器
			len--;
		}
		delay_cycle(500);
		/* Enable SPI1 DMA RX request */
		SPI1->CR2 &= ~((u32)1<<0);                    //关闭接收缓冲区DMA使能
		delay_cycle(500);
	}
	delay_cycle(2);
	FLASH2_CSSelect(Disable); 

}

//------FLASH-写入LEN个字节---------
void FLASH2_GPIOSPI_Write(unsigned int write_addr, unsigned char *desbuffer,unsigned int len) // reentrant//FLASH-写入LEN个字节
{ 
	unsigned char addr1,addr2,addr3;
	addr1 = ((write_addr&0xff0000)>>16);
	addr2 = ((write_addr&0xff00)>>8);
	addr3 = ((write_addr&0xff));

	//RAM_CS   = 1;FLASH_CS   = 0; 
	FLASH2_CSSelect( Enable);                  	                   
	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	/* Send byte through the SPI2 peripheral */
	//SPI_I2S_SendData(SPI1, data);
	SPI1->DR = FLASH_CMD_WREN  ;     //  写使能 
	delay_cycle(500);   
	FLASH2_CSSelect(Disable);  

	//	RAM_CS   = 1;FLASH_CS   = 0; 
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

	delay_cycle (500);                        
	FLASH2_CSSelect(Disable); 

	//FLASH2_CSSelectEnable( cs_select);
	//Delay_ms (1);

	//SPI0DAT  = FLASH_CMD_WRDI;        //写无效
	//while (!SPIF);                       
	//SPIF     = 0;                        

	//RAM_CS   = 1;FLASH_CS   = 1;   
	//FLASH2_CSSelectDisable(cs_select);
	//Delay_ms (1);  

	//FLASH2_IsBusy(1); 
	FLASH2_GPIOSPI_IsBusy();
}
//------------------------------------------------------------
void FLASH2_GPIOSPI_IsBusy(void)// 
{
	do
	{ 
		FLASH2_CSSelect(Enable);   
		delay_cycle(10);  
		FLASH_u8temp = SPI1->DR;

		//SPI0DAT  = FLASH_CMD_RDSTATUSR;   
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI1->DR  =	FLASH_CMD_RDSR  ;  
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		FLASH_u8temp = SPI1->DR;

		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI1->DR  =	FLASH_CMD_RDSR  ;       
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		FLASH_u8temp = SPI1->DR;

		FLASH2_CSSelect(Disable); 
	}
	//while(((SPI1->DR) & 0x01) == 0x01); //Write In Progress Bit:"0" indicates the device is ready (default);"1" indicates a write cycle is in progress and the device is busy.
	while(((FLASH_u8temp) & 0x01) == 0x01); //Write In Progress Bit:"0" indicates the device is ready (default);"1" indicates a write cycle is in progress and the device is busy.
}

void FLASH2_GPIOSPI_SER(unsigned int u32_addr)  ////每次擦擦4K
{
	unsigned char addr1,addr2,addr3;
	addr1 = ((u32_addr&0xff0000)>>16);
	addr2 = ((u32_addr&0xff00)>>8);
	addr3 = ((u32_addr&0xff));
	FLASH2_CSSelect( Enable);                  

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = FLASH_CMD_WREN  ;     //  写使能 
	delay_cycle(1000);   
	FLASH2_CSSelect(Disable);                                     

	//---------FLASH_CMD_SE--------------
	//  RAM_CS   = 1;FLASH_CS   = 0;  
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
	delay_cycle(1000);   

	//	RAM_CS   = 1;FLASH_CS   = 1; 
	FLASH2_CSSelect(Disable); 

	///-------指导擦除结束------------------	 
	//	 RAM_CS   = 1;FLASH_CS   = 0;   
	//	FLASH2_IsBusy(1); 
	FLASH2_GPIOSPI_IsBusy();	 
	delay_cycle(2);  
}

void FLASH2_GPIOSPI_SE(unsigned int u32_addr)  ////每次擦擦64K
{	
	unsigned char addr1,addr2,addr3;
	addr1 = ((u32_addr&0xff0000)>>16);
	addr2 = ((u32_addr&0xff00)>>8);
	addr3 = ((u32_addr&0xff));
	FLASH2_CSSelect( Enable);                  

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = FLASH_CMD_WREN  ;     //  写使能 

	//RAM_CS   = 1;FLASH_CS   = 1;   
	delay_cycle(1000);   
	FLASH2_CSSelect(Disable);                                     

	//---------FLASH_CMD_SE--------------
	//RAM_CS   = 1;FLASH_CS   = 0;  
	FLASH2_CSSelect(Enable);   

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = FLASH_CMD_SE;//         0xD7        //扇区擦除
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = addr1;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = addr2;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI1->DR = addr3;
	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	delay_cycle(1000);

	//RAM_CS   = 1;FLASH_CS   = 1; 
	FLASH2_CSSelect(Disable); 

	///-------指导擦除结束------------------	 
	//RAM_CS   = 1;FLASH_CS   = 0;   
	//FLASH2_IsBusy(1); 
	FLASH2_GPIOSPI_IsBusy();	 
	delay_cycle(2);  
}

// void FLASH2_GPIOSPI_Read (unsigned int write_addr, unsigned char *buffer, unsigned int len)//FLASH-读出LEN个字节-
// { 
//    //u8  i;
//    unsigned char addr1,addr2,addr3;
// 		addr1 = ((write_addr&0xff0000)>>16);
// 		addr2 = ((write_addr&0xff00)>>8);
// 		addr3 = ((write_addr&0xff));
//   //   RAM_CS   = 1;FLASH_CS   = 0;  //片选使能
// // delay_cycle(2);  
//    FLASH2_CSSelect( Enable);    //RAM片选工作              
//    
// // 		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
// //	  SPI1->DR = FLASH_CMD_READ;       //读取数据
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(FLASH_CMD_READ);
// 	
// // 		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
// //	  SPI1->DR = addr1;
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(addr1);
// // 		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
// //	  SPI1->DR = addr2;
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(addr2);
// // 		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
// //	  SPI1->DR = addr3;
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(addr3);

// //		if(fast_read ==1)
// 		{
// 		while(len>0)
// 			{
// 		*buffer=GPIOSPI_SR_BYTE(0x55);
// 	  buffer++;
// 		len--;
// 			}

// 		}
//     delay_cycle(2);
// 	FLASH2_CSSelect(Disable); 
// 		
// 	}
// 	
// //------FLASH-写入LEN个字节---------
// void FLASH2_GPIOSPI_Write(unsigned int write_addr, unsigned char *desbuffer,unsigned int len) // reentrant//FLASH-写入LEN个字节
// { 
//    unsigned char addr1,addr2,addr3;
// 		addr1 = ((write_addr&0xff0000)>>16);
// 		addr2 = ((write_addr&0xff00)>>8);
// 		addr3 = ((write_addr&0xff));

//    FLASH2_CSSelect( Enable);                  	                   
// //  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
// //	  SPI1->DR = FLASH_CMD_WREN  ;     //  写使能 
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(FLASH_CMD_WREN);
// 	
// 	delay_cycle(2);  
// 	FLASH2_CSSelect(Disable);  
// 	
//    FLASH2_CSSelect( Enable);                  
// //  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
// //   SPI1->DR  = FLASH_CMD_PP; //写页
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(FLASH_CMD_PP);

// // 		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
// //	  SPI1->DR = addr1;
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(addr1);
// // 		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
// //	  SPI1->DR = addr2;
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(addr2);
// // 		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
// //	  SPI1->DR = addr3;
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(addr3);
// 	
//    while(len>0)
//     {
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(*desbuffer); //写数据
// 	  desbuffer++;
// 	  len--;
//      }
// 		 
//      delay_cycle (2);                        
//  FLASH2_CSSelect(Disable); 
// 		 
// //	FLASH2_IsBusy(1); 
// 	FLASH2_GPIOSPI_IsBusy();	 
// }
// //------------------------------------------------------------
// void FLASH2_GPIOSPI_IsBusy(void)// 
// {
// //   unsigned char i;

//   do
//    { 
// 	 FLASH2_CSSelect(Enable);   
// 		delay_cycle(10);  
// 		 
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(FLASH_CMD_RDSR);
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(0x55);
// 		 
// 		delay_cycle(10);  
// 	FLASH2_CSSelect(Disable); 
//      } 
// 		 while(((FLASH_u8temp) & 0x01) == 0x01); //Write In Progress Bit:"0" indicates the device is ready (default);"1" indicates a write cycle is in progress and the device is busy.

// }

// void FLASH2_GPIOSPI_SER(unsigned int u32_addr)  ////每次擦擦4K
// {
//    unsigned char addr1,addr2,addr3;
// 		addr1 = ((u32_addr&0xff0000)>>16);
// 		addr2 = ((u32_addr&0xff00)>>8);
// 		addr3 = ((u32_addr&0xff));
// 	                  
// 		FLASH2_CSSelect( Enable);                  	                   
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(FLASH_CMD_WREN);
// 		delay_cycle(2);  
// 		FLASH2_CSSelect(Disable);  
//     
// 		FLASH2_CSSelect(Enable);  
// 	
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(FLASH_CMD_SER);//         0xD7  //扇区擦除

// 		FLASH_u8temp = GPIOSPI_SR_BYTE(addr1);
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(addr2);
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(addr3);

// 		FLASH2_CSSelect(Disable); 
//  
// 	FLASH2_GPIOSPI_IsBusy();	 
// }
// void FLASH2_GPIOSPI_SE(unsigned int u32_addr)  ////每次擦擦64K
// {
//    unsigned char addr1,addr2,addr3;
// 		addr1 = ((u32_addr&0xff0000)>>16);
// 		addr2 = ((u32_addr&0xff00)>>8);
// 		addr3 = ((u32_addr&0xff));
// 	                  
// 		FLASH2_CSSelect( Enable);                  	                   
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(FLASH_CMD_WREN);
// 		delay_cycle(2);  
// 		FLASH2_CSSelect(Disable);  
//     
// 		FLASH2_CSSelect(Enable);  
// 	
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(FLASH_CMD_SE);//         0xD7  //扇区擦除

// 		FLASH_u8temp = GPIOSPI_SR_BYTE(addr1);
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(addr2);
// 		FLASH_u8temp = GPIOSPI_SR_BYTE(addr3);

// 		FLASH2_CSSelect(Disable); 
//  
// 	FLASH2_GPIOSPI_IsBusy();	 
// }

//*****************************************************************************
void SPI_Configuration(void)
{
	SPI1_Receive_Pointer = SPI_BUFFER;
	SPI1_Trans_Pointer = SPI_BUFFER;
	SPI1_Init();
	//SPI2_Init();
	SPI1_Receive_BufSize = 128;
	//SPI2_Receive_BufSize = 0;
}
/********************************END OF FILE***********************************/

