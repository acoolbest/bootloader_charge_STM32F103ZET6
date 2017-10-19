//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*******************************************
#include "stm32f10x.h"

  u8  *DMA1_Channel1_Pointer;
  u8   DMA1_Channel1_State;
  u8   DMA1_Channel1_Error;
  u32  DMA1_Channel1_Buffer;

  u8  *DMA2_Channel1_Pointer;
  u8   DMA2_Channel1_State;
  u8   DMA2_Channel1_Error;
  u32  DMA2_Channel1_Buffer;

//*******************************************
void DMA_Configuration(void)
{
//	DMA1_Channel1_Init();
	DMA1_Channel2_Init(SPI_BUFFER,16);
//	DMA2_Channel1_Init();
}
//*******************************************
void DMA1_Channel1_Init(void)     //
{
	DMA_InitTypeDef DMA_InitStructure;
	
	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		
	/* DMA channel1 configuration */
	DMA_DeInit(DMA1_Channel1);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;	 			//ADC地址
//	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;	//内存地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ADC_BUFFER;	//内存地址
	
// 		/*方向：从内存到外设*/		
// 		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 320;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//外设地址固定
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;  				//内存地址固定
// 		/*内存地址自增*/
 		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//半字
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;	//字
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;										//循环传输
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	/* Enable DMA channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
//	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
}
//*******************************************
void DMA1_Channel2_Init(unsigned char *buffer,unsigned int len)     //
{
	DMA_InitTypeDef DMA_InitStructure;
	
	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		
	/* DMA channel1 configuration */
	DMA_DeInit(DMA1_Channel2);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI1_DR_Address;	 			//SPIDR地址
//	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;	//内存地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)buffer;	//内存地址
	
// 		/*方向：从内存到外设*/		
// 		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = len;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//外设地址固定
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;  				//内存地址固定
// 		/*内存地址自增*/
 		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//半字递增半字外设
	//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;	//字
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	  //半字递增半字内存
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;										//非循环传输
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);
	
//   /* Enable SPI1 DMA RX request */
//   SPI1->CR2 |= 1<<0;                                                                 //接收缓冲区DMA使能
	
	/* Enable DMA channel2 */
	DMA_Cmd(DMA1_Channel2, ENABLE);
	
//	DMA_ITConfig(DMA1_Channel2,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
}
//*******************************************
void DMA2_Channel1_Init(void)
{
	
}
//*******************************************
void DMA1_Channel1_IRQHandler(void)
{
//DMA1_Channel1_IRQn
	DMA1_Channel1_State = 1;
	ADC1_State |= 1;
	DMA_ClearFlag(DMA1_FLAG_GL1);
	/* Enable ADC1 */
	ADC_Cmd(ADC1, DISABLE);
	/* 由于没有采用外部触发，所以使用软件触发ADC转换 */ 
//	ADC_SoftwareStartConvCmd(ADC1, DISABLE);
	
	GPIOC->ODR ^= GPIO_Pin_3; 
}
//*******************************************
void DMA1_Channel2_IRQHandler(void)
{
//DMA1_Channel1_IRQn
	DMA1_Channel1_State = 1;
	DMA_ClearFlag(DMA1_FLAG_GL1);   //清除所有标志 与DMA1_Channel2相关的GIF、TEIF、HTIF、CIF
	
    /* Enable SPI1 DMA RX request */
    SPI1->CR2 &= ~((u32)1<<0);                    //关闭接收缓冲区DMA使能
	
//	GPIOC->ODR ^= GPIO_Pin_3; 
}
/*******************************************************************************
* Function Name  : SPI1_DMA_Configuration
* Description    : 配置SPI1_RX的DMA通道2，SPI1_TX的DMA通道3
* Input          : None
* Output         : None
* Return         : None
* Attention             : 
*******************************************************************************/
// void SPI1_DMA_Configuration( void )
// {
//     DMA_InitTypeDef DMA_InitStructure;
//     
//     /* DMA1 Channel2 (triggered by SPI1 Rx event) Config */
//   DMA_DeInit(DMA1_Channel2);  
//   DMA_InitStructure.DMA_PeripheralBaseAddr = SPI1_DR_Address;                          //设置 SPI1 发送外设(0x4001300C) 地址(目的地址)
//   DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)SPI1_RX_Buff;                    //设置 SRAM 存储地址(目的地址)
//   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                                //传输方向 外设-内存
//   DMA_InitStructure.DMA_BufferSize = SPI1_ReciveBufferSize;                         //设置 SPI1 发送长度
//   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//   DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//   DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//   DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//   DMA_Init(DMA1_Channel2, &DMA_InitStructure);
//   
//   DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
//   /* Enable SPI1 DMA RX request */
//   SPI1->CR2 |= 1<<0;                                                                 //接收缓冲区DMA使能
//   DMA_Cmd(DMA1_Channel2, ENABLE);
//     
//     
//     /* DMA1 Channel3 (triggered by SPI1 Tx event) Config */
//   DMA_DeInit(DMA1_Channel3);  
//   DMA_InitStructure.DMA_PeripheralBaseAddr = SPI1_DR_Addr;                          //设置  接收外设(0x4001300C) 地址(源地址)
//   DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)SPI1_TX_Buff;                    //设置 SRAM 存储地址(源地址)
//   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                                //传输方向 内存-外设
//   DMA_InitStructure.DMA_BufferSize = SPI1_SendBufferSize;                           //设置 SPI1 接收长度
//   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                  //外设地址增量(不变)
//   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                           //内存地址增量(变化)
//   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;           //外设传输宽度(字节)
//   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                   //内存传输宽度(字节)
//   DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                     //传输方式,一次传输完停止,不重新加载
//   DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                           //中断方式-高(三级)
//   DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                      //内存到内存方式禁止
//   DMA_Init(DMA1_Channel3, &DMA_InitStructure);
//   
//   DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);                                   //开启 DMA1_Channel3 传输完成中断
//   DMA_ITConfig(DMA1_Channel3, DMA_IT_TE, ENABLE);                                   //开启 DMA1_Channel3 传输错误中断
//   /* Enable SPI1 DMA TX request */
//   SPI1->CR2 |= 1<<1;                                                                //发送缓冲区DMA使能
//   DMA_Cmd(DMA1_Channel3, DISABLE);                                                  //开启 DMA 通道 DMA1_Channel3
// }

/*******************************************************************************
* Function Name  : SPI1_Send
* Description    : SPI1的DMA方式发送
* Input          : SPI1_TX_Buff[SPI1_SendBufferSize]
* Output         : None
* Return         : None
* Attention             : 关闭DMA通道3之前必须等待TXE为1，等待忙标志为0
*******************************************************************************/
// void SPI1_Send( u8 *buff, u32 len )
// {
//     DMA1_Channel3->CPAR = SPI1_DR_Addr; //外设地址
//     DMA1_Channel3->CMAR = (u32) buff; //mem地址
//     DMA1_Channel3->CNDTR = len ; //传输长度
//     DMA1_Channel3->CCR = (0 << 14) | // 非存储器到存储器模式
//             (2 << 12) | // 通道优先级高
//             (0 << 11) | // 存储器数据宽度8bit
//             (0 << 10) | // 存储器数据宽度8bit
//             (0 <<  9) | // 外设数据宽度8bit
//             (0 <<  8) | // 外设数据宽度8bit
//             (1 <<  7) | // 存储器地址增量模式
//             (0 <<  6) | // 外设地址增量模式(不增)
//             (0 <<  5) | // 非循环模式
//             (1 <<  4) | // 从存储器读
//             (1 <<  3) | // 允许传输错误中断
//             (0 <<  2) | // 允许半传输中断
//             (1 <<  1) | // 允许传输完成中断
//             (1);        // 通道开启
// }

/*******************************************************************************
* Function Name  : SPI1_Recive
* Description    : SPI1的DMA方式接收
* Input          : None
* Output         : SPI1_RX_Buff[SPI1_ReciveBufferSize]
* Return         : None
* Attention             : 必须要先关闭通道2,然后再配置通道2的参数
*******************************************************************************/
void SPI1_DMARecive( u8 *buff, u32 len )
{
    DMA1_Channel2->CCR &= ~(1<<0);          //关闭DMA通道2
    
    /* Enable SPI1 DMA RX request */
    SPI1->CR2 |= 1<<0;                    //接收缓冲区DMA使能
	
    DMA1_Channel2->CPAR = SPI1_DR_Address; //外设地址
    DMA1_Channel2->CMAR = (uint32_t)buff; //mem地址
    DMA1_Channel2->CNDTR = len ; //传输长度
    DMA1_Channel2->CCR = (0 << 14) | // 非存储器到存储器模式
            (2 << 12) | // 通道优先级高
            (0 << 11) | // 存储器数据宽度8bit
            (0 << 10) | // 存储器数据宽度8bit
            (0 <<  9) | // 外设数据宽度8bit
            (0 <<  8) | // 外设数据宽度8bit
            (1 <<  7) | // 存储器地址增量模式
            (0 <<  6) | // 外设地址增量模式(不增)
            (0 <<  5) | // 非循环模式
            (0 <<  4) | // 传输方向 外设-内存
            (0 <<  3) | // 允许传输错误中断
            (0 <<  2) | // 允许半传输中断
            (1 <<  1) | // 允许传输完成中断
            (1);        // 通道开启
}
