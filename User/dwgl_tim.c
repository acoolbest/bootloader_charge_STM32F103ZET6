//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*******************************************
#include "stm32f10x.h"
//变量
  u8  *TIM2_Pointer;
  u8   TIM2_State;
  u8   TIM2_Error;
  u16  TIM2_Buffer;
  u16  TIM2_Counter;

  u8  *TIM3_Pointer;
  u8   TIM3_State;
  u8   TIM3_Error;
  u16  TIM3_Buffer;
  u16  TIM3_Counter;

//*******************************************
void TIM_Configuration(void)
{
	TIM2_Init();
//	TIM3_Init();
	TIM2_Buffer =0;
	TIM3_Buffer =0;
}
//**********************************************************************
void TIM2_Init(void)  //
{
TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
GPIO_InitTypeDef GPIO_InitStructure;

   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
	 RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M
	GPIO_Init(GPIOA, &GPIO_InitStructure);
   //复位外设
   TIM_DeInit(TIM2);

   TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 0x00;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); // Time base configuration

	TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);

	TIM_SetCounter(TIM2, 0);
	TIM_Cmd(TIM2, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE);		/*先关闭等待使用*/  
}

//**********************************************************************
//  TIM_Period / Auto Reload Register(ARR) = 1000   TIM_Prescaler--71 
//  中断周期为 = 1/(72MHZ /72) * 1000 = 1ms
//  TIMxCLK/CK_PSC --> TIMxCNT --> TIM_Period(ARR) --> 中断 且TIMxCNT重置为0重新计数 
void TIM3_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		
		/* 设置TIM2CLK 为 72MHZ */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
    //TIM_DeInit(TIM2);
	
	/* 自动重装载寄存器周期的值(计数值) */
    TIM_TimeBaseStructure.TIM_Period=1000;
	
    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
	  /* 时钟预分频数为72 */
    TIM_TimeBaseStructure.TIM_Prescaler= 71;
	
		/* 对外部时钟进行采样的时钟分频,这里没有用到 */
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
		
    TIM_Cmd(TIM3, ENABLE);																		
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , DISABLE);		/*先关闭等待使用*/    
}


/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
	
}

/**
  * @brief  This function handles TIM3 interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET ) 
	{	
		GPIO_ResetBits(GPIOA, GPIO_Pin_11);
		TIM3_State = 1;
		TIM3_Counter++;
		TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);  		 
	}		 	
}
/*********************************************END OF FILE**********************/
