//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*******************************************
#include "stm32f10x.h"

  u8  *DAC1_Pointer;
  u8   DAC1_State;
  u8   DAC1_Error;
  u16  DAC1_Buffer =0;
	
  u8  *DAC2_Pointer;
  u8   DAC2_State;
  u8   DAC2_Error;
  u16  DAC2_Buffer =0;
//*******************************************
void DAC_Configuration(void)
{
	DAC1_Init();
//	DAC2_Init();
}
//*******************************************
#if defined (dac_out_Normal)
void DAC1_Init(void)  //
{
	DAC_InitTypeDef DAC_InitType;
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,  ENABLE);	//使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA ,  ENABLE);	//使能时钟
//DAC_Channl_1输出固定为PA4	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //设置的模拟输入，这样不受外部影响
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);		   
		
	DAC_InitType.DAC_Trigger = DAC_Trigger_None; //不使用触发功能 TEN1=0
	DAC_InitType.DAC_WaveGeneration = DAC_WaveGeneration_None; //不使用波形发生器
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;//
	DAC_InitType.DAC_OutputBuffer = DAC_OutputBuffer_Enable; //输出有缓冲能增加驱动能力
	DAC_Init(DAC_Channel_1,&DAC_InitType);	//初始化DAC通道1
	DAC_Cmd(DAC_Channel_1,ENABLE);	//使能DAC1
	
	DAC_SetChannel1Data(DAC_Align_12b_R, 0xffff);//初始化DAC输出，这步可以没有	
//	DAC_GetDataOutputValue(DAC_Channel_1);
}
#endif
#if defined  (dac_out_Sin)    
//DMA方式  
//触发方式
void DAC1_Init(void)   
{
	DAC_InitTypeDef DAC_InitType;
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,  ENABLE);	//使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA ,  ENABLE);	//使能时钟
//DAC_Channl_1输出固定为PA4	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //设置的模拟输入，这样不受外部影响
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);		   
		
//	DAC_InitType.DAC_Trigger = DAC_Trigger_T2_TRGO; //不使用触发功能 TEN1=0
	DAC_InitType.DAC_Trigger = DAC_Trigger_Software; //不使用触发功能 TEN1=0
//	DAC_InitType.DAC_WaveGeneration = DAC_WaveGeneration_None; //不使用波形发生器
	DAC_InitType.DAC_WaveGeneration = DAC_WaveGeneration_Triangle; //使用波形发生器
//	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;//
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_2047;//设定三角波的幅值，0-4095。当为4095时最幅值为3.3v
	DAC_InitType.DAC_OutputBuffer = DAC_OutputBuffer_Enable; //输出有缓冲能增加驱动能力
	DAC_Init(DAC_Channel_1,&DAC_InitType);	//初始化DAC通道1
	DAC_Cmd(DAC_Channel_1,ENABLE);	//使能DAC1
	
	DAC_SetChannel1Data(DAC_Align_8b_R, 0xffff);//初始化DAC输出，这步可以没有
	
	DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);//
	
//	DAC_GetDataOutputValue(DAC_Channel_1);

}
#endif
#if defined  (dac_out_Triangle)
void DAC1_Init(void)  //有电压不归0的现象，原因还不明
{
	DAC_InitTypeDef DAC_InitType;
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,  ENABLE);	//使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA ,  ENABLE);	//使能时钟
//DAC_Channl_1输出固定为PA4	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //设置的模拟输入，这样不受外部影响
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);		   
		
	DAC_InitType.DAC_Trigger = DAC_Trigger_Software; //使用触发功能      
	DAC_InitType.DAC_WaveGeneration = DAC_WaveGeneration_Triangle; //使用波形发生器
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_2047;//设定三角波的幅值，0-4095。当为4095时最幅值为3.3v
	DAC_InitType.DAC_OutputBuffer = DAC_OutputBuffer_Enable; //输出有缓冲能增加驱动能力
	DAC_Init(DAC_Channel_1,&DAC_InitType);	//初始化DAC通道1
	DAC_Cmd(DAC_Channel_1,ENABLE);	//使能DAC1
	
	DAC_SetChannel1Data(DAC_Align_8b_R, 0xffff);//初始化DAC输出，这步可以没有	
	DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);//	
//	DAC_GetDataOutputValue(DAC_Channel_1);

}
#endif

//*******************************************
void DAC2_Init(void)
{
//DAC_Channl_1输出固定为PA5	
	//DAC_InitTypeDef DAC_InitType;
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,  ENABLE);	//使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA ,  ENABLE);	//使能时钟
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //设置的模拟输入，这样不受外部影响
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);		   
	#if 0
	DAC_InitType.DAC_Trigger = DAC_Trigger_None; //不使用触发功能 TEN1=0
	DAC_InitType.DAC_WaveGeneration = DAC_WaveGeneration_None; //不使用波形发生器
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
	DAC_InitType.DAC_OutputBuffer = DAC_OutputBuffer_Disable; //DAC1输出缓存关闭
	DAC_Init(DAC_Channel_2,&DAC_InitType);	//初始化DAC通道2
	#endif
	DAC_Cmd(DAC_Channel_2,ENABLE);	//使能DAC2
	
	DAC_SetChannel2Data(DAC_Align_12b_R, 0x000);//初始化DAC输出，这步可以没有
//	DAC_GetDataOutputValue(DAC_Channel_2);

}
