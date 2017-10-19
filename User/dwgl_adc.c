//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*******************************************
#include "stm32f10x.h"

u16 * ADC1_Pointer = NULL;
u16  ADC1_BufSize;
u16  ADC1_Length ;
u8   ADC1_State;
u8   ADC1_Error;
u16  ADC1_Buffer;

u16  *ADC2_Pointer;
u16  ADC2_BufSize;
u16  ADC2_Length ;
u8   ADC2_State;
u8   ADC2_Error;
u16  ADC2_Buffer;

u16 * ADC3_Pointer = NULL;

u8 ADC1_channel[18]={AN1_ADC1_CH8,AN2_ADC1_CH1,AN3_ADC1_CH13,AN7_ADC1_CH15,AN8_ADC1_CH9,AN9_ADC1_CH0};
static u8 ADC1_channel_index = 0;

u8 ADC3_channel[18]={AN4_ADC3_CH6,AN5_ADC3_CH8,AN6_ADC3_CH11,AN10_ADC3_CH7,AN11_ADC3_CH10,AN12_ADC3_CH12,INPUT_AD_ADC3_CH5};
static u8 ADC3_channel_index = 0;

#if  defined ADC1_init_Normal
//*******************************************
void ADC1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);

	/* Configure PC.0  as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);				// PC0,输入时不用设置速率

	/* ADC1 configuration */	
	ADC_DeInit(ADC1);   //首先复位ADC1。ADC的全部寄存器为缺省值 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;			//独立ADC模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ; 	 				//禁止扫描模式，扫描模式用于多通道采集
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			//开启连续转换模式，即不停地进行ADC转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//不使用外部触发转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//采集数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	 								//要转换的通道数目1
	ADC_Init(ADC1, &ADC_InitStructure);

	/*配置ADC时钟，为PCLK2的8分频，即9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	//	RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
	/*配置ADC1的通道11为55.	5个采样周期，序列为1 */ 
	//	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/*复位校准寄存器 */   
	ADC_ResetCalibration(ADC1);
	/*等待校准寄存器复位完成 */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* ADC校准 */
	ADC_StartCalibration(ADC1);
	/* 等待校准完成*/
	while(ADC_GetCalibrationStatus(ADC1));
	//开中断
	//	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);//开中断

	/* 由于没有采用外部触发，所以使用软件触发ADC转换 */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
#endif
#if  defined ADC1_init_DMA
//*******************************************
void ADC1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);

	/* Configure PC.0  as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);				// PC0,输入时不用设置速率

	/* ADC1 configuration */	
	ADC_DeInit(ADC1);   //首先复位ADC1。ADC的全部寄存器为缺省值 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;			//独立ADC模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ; 	 				//禁止扫描模式，扫描模式用于多通道采集
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			//开启连续转换模式，即不停地进行ADC转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//不使用外部触发转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//采集数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	 								//要转换的通道数目1
	ADC_Init(ADC1, &ADC_InitStructure);

	/*配置ADC时钟，为PCLK2的8分频，即9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	//	RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
	/*配置ADC1的通道11为55.	5个采样周期，序列为1 */ 
	//	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/*复位校准寄存器 */   
	ADC_ResetCalibration(ADC1);
	/*等待校准寄存器复位完成 */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* ADC校准 */
	ADC_StartCalibration(ADC1);
	/* 等待校准完成*/
	while(ADC_GetCalibrationStatus(ADC1));
	//开中断
	//	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);//开中断

	/* 由于没有采用外部触发，所以使用软件触发ADC转换 */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
#endif
#if  defined ADC1_init_DMAE
//*******************************************
void ADC1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);

	/* Configure PC.0  as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);				// PC0,输入时不用设置速率

	/* ADC1 configuration */	
	ADC_DeInit(ADC1);   //首先复位ADC1。ADC的全部寄存器为缺省值 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;			//独立ADC模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;			     //禁止扫描模式，扫描模式用于多通道采集
	//	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			//开启连续转换模式，即不停地进行ADC转换
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;			//开启连续转换模式，即不停地进行ADC转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//不使用外部触发转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//采集数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1; 								//要转换的通道数目1
	ADC_Init(ADC1, &ADC_InitStructure);

	/*配置ADC时钟，为PCLK2的8分频，即9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	//	RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
	/*配置ADC1的通道11为55.	5个采样周期，序列为1 */ 
	//	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_55Cycles5);
	ADC1_Length = 0;
	AINx_ADCch[16] = 5;
	ADC_RegularChannelConfig(ADC1, AINx_ADCch[AINx_ADCch[16]], 1, ADC_SampleTime_239Cycles5);
	//	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_239Cycles5);
	// 	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 2, ADC_SampleTime_239Cycles5);
	// 	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_239Cycles5);
	// 	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 4, ADC_SampleTime_239Cycles5);
	// 	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 5, ADC_SampleTime_239Cycles5);
	// 	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 6, ADC_SampleTime_239Cycles5);
	// 	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 7, ADC_SampleTime_239Cycles5);
	// 	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 8, ADC_SampleTime_239Cycles5);

	/* Enable ADC1 DMA */
	//	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/*复位校准寄存器 */   
	ADC_ResetCalibration(ADC1);
	/*等待校准寄存器复位完成 */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* ADC校准 */
	ADC_StartCalibration(ADC1);
	/* 等待校准完成*/
	while(ADC_GetCalibrationStatus(ADC1));
	//开中断
	//	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);//开中断

	/* 由于没有采用外部触发，所以使用软件触发ADC转换 */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
#endif

//*******************************************
static void ADC1_Init(void)
{
	//GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	/* Enable ADC1*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* ADC1 configuration */	
	ADC_DeInit(ADC1);													//首先复位ADC1。ADC的全部寄存器为缺省值 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;					//独立ADC模式
	//ADC_InitStructure.ADC_ScanConvMode = DISABLE;						//禁止扫描模式，扫描模式用于多通道采集
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	//ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;				//开启连续转换模式，即不停地进行ADC转换
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//不使用外部触发转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//采集数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;								//要转换的通道数目1
	ADC_Init(ADC1, &ADC_InitStructure);

	/*配置ADC时钟，为PCLK2的8分频，即9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);

	/*配置ADC1的通道11为55.	5个采样周期，序列为1 */ 
	//	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_55Cycles5);

	ADC_RegularChannelConfig(ADC1, ADC1_channel[ADC1_channel_index], 1, ADC_SampleTime_239Cycles5);

	/* Enable ADC1 DMA */
	//ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/*复位校准寄存器 */   
	ADC_ResetCalibration(ADC1);
	
	/*等待校准寄存器复位完成 */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* ADC校准 */
	ADC_StartCalibration(ADC1);
	
	/* 等待校准完成*/
	while(ADC_GetCalibrationStatus(ADC1));
	
	/* 开中断*/
	//ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

	/* 由于没有采用外部触发，所以使用软件触发ADC转换 */
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

#if 0
void ADC2_Init(void)
{

}
#endif

static void ADC3_Init(void)
{
	//GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	/* Enable ADC3*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

	/* ADC3 configuration */	
	ADC_DeInit(ADC3);													//首先复位ADC3。ADC的全部寄存器为缺省值 
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;					//独立ADC模式
	//ADC_InitStructure.ADC_ScanConvMode = DISABLE;						//禁止扫描模式，扫描模式用于多通道采集
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	//ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;				//开启连续转换模式，即不停地进行ADC转换
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//不使用外部触发转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//采集数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;								//要转换的通道数目1
	
	ADC_Init(ADC3, &ADC_InitStructure);

	/*配置ADC时钟，为PCLK2的8分频，即9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	
	/*配置ADC3的通道11为55.	5个采样周期，序列为1 */ 
	//ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 1, ADC_SampleTime_55Cycles5);
	
	ADC_RegularChannelConfig(ADC3, ADC3_channel[ADC3_channel_index], 1, ADC_SampleTime_239Cycles5);

	/* Enable ADC3 DMA */
	//ADC_DMACmd(ADC3, ENABLE);

	/* Enable ADC3 */
	ADC_Cmd(ADC3, ENABLE);

	/*复位校准寄存器 */   
	ADC_ResetCalibration(ADC3);
	
	/*等待校准寄存器复位完成 */
	while(ADC_GetResetCalibrationStatus(ADC3));

	/* ADC校准 */
	ADC_StartCalibration(ADC3);
	
	/* 等待校准完成*/
	while(ADC_GetCalibrationStatus(ADC3));
	
	/* 开中断*/
	//ADC_ITConfig(ADC3, ADC_IT_EOC, ENABLE);

	/* 由于没有采用外部触发，所以使用软件触发ADC转换 */
	ADC_SoftwareStartConvCmd(ADC3, ENABLE);
}

//***********************************************************************

void ADC1_2_IRQHandler(void)
{
	static u8 u8_sampling_times = 0;
	
	if(ADC_GetITStatus(ADC1, ADC_IT_EOC)!= RESET)
	{
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
		
		ADC1_Pointer[ADC1_channel[ADC1_channel_index]*ADC_SAMPLING_TIMES+u8_sampling_times] = ADC_GetConversionValue(ADC1);
		
		u8_sampling_times++;
		if(u8_sampling_times >= 3)
		{
			u8_sampling_times = 0;

			ADC1_channel_index++;
			if(ADC1_channel_index >= ADC1_ENABLE_CHANNEL_NUM) ADC1_channel_index = 0;
			
			ADC_RegularChannelConfig(ADC1, ADC1_channel[ADC1_channel_index], 1, ADC_SampleTime_239Cycles5);
		}
	}
}

void ADC3_IRQHandler(void)
{
	static u8 u8_sampling_times = 0;
	
	if(ADC_GetITStatus(ADC3, ADC_IT_EOC)!= RESET)
	{
		ADC_ClearITPendingBit(ADC3, ADC_IT_EOC);
		
		ADC3_Pointer[ADC3_channel[ADC3_channel_index]*ADC_SAMPLING_TIMES+u8_sampling_times] = ADC_GetConversionValue(ADC3);
		
		u8_sampling_times++;
		if(u8_sampling_times >= 3)
		{
			u8_sampling_times = 0;

			ADC3_channel_index++;
			if(ADC3_channel_index >= ADC3_ENABLE_CHANNEL_NUM) ADC3_channel_index = 0;
			
			ADC_RegularChannelConfig(ADC3, ADC3_channel[ADC3_channel_index], 1, ADC_SampleTime_239Cycles5);
		}
	}
}

void ADC_Configuration(void)
{
	ADC1_Pointer = ADC_BUFFER;
	ADC3_Pointer = ADC_BUFFER+ADC_BUFFER_SIZE/2;

	ADC1_Init();
	//ADC2_Init();
	ADC3_Init();
}
