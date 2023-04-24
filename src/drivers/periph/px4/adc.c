#include "adc.h"

void adc1_init(void)
{
	/* rcc initialization */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure= {
		.GPIO_Pin = GPIO_Pin_2,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_Mode = GPIO_Mode_AIN,
		.GPIO_PuPd  = GPIO_PuPd_NOPULL
	};
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure ADC1
    ADC_InitTypeDef ADC_InitStruct1 = {0};
    ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
    ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStruct);

    ADC_InitStruct1.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStruct1.ADC_ScanConvMode = DISABLE;
    ADC_InitStruct1.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStruct1.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct1.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct1.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStruct1);

	/* enable i2c */
	ADC_Cmd(ADC1,ENABLE);
    // Configure ADC channels
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_3Cycles);
    ADC_SoftwareStartConv(ADC1);

}


void adc2_init(void)
{
	/* rcc initialization */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure= {
		.GPIO_Pin = GPIO_Pin_3,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_Mode = GPIO_Mode_AIN,
		.GPIO_PuPd  = GPIO_PuPd_NOPULL
	};
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure ADC2
    ADC_InitTypeDef ADC_InitStruct1 = {0};
    ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
    ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStruct);

    ADC_InitStruct1.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStruct1.ADC_ScanConvMode = DISABLE;
    ADC_InitStruct1.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStruct1.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct1.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct1.ADC_NbrOfConversion = 1;
    ADC_Init(ADC2, &ADC_InitStruct1);

	/* enable i2c */
	ADC_Cmd(ADC2,ENABLE);
    // Configure ADC channels
    ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 1, ADC_SampleTime_3Cycles);
    ADC_SoftwareStartConv(ADC2);

}


