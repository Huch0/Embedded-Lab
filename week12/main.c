#include "stm32f10x.h"
#include "core_cm3.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "misc.h"
#include "lcd.h"
#include "touch.h"

#define LCD_TEAM_NAME_X 20
#define LCD_TEAM_NAME_Y 50
#define LCD_LUX_VAL_X 20
#define LCD_LUX_VAL_Y 110

// Todo
#define LUX_THRESHOLD 1500

void Init(void);
void RccInit(void);
void GpioInit(void);
void AdcInit(void);
void DMA_Configure(void);

// volatile unsigned 32bits
volatile uint32_t ADC_Value[1];

int main()
{
    Init();

    LCD_Clear(WHITE);

    while (1)
    {
        // Todo: 조도센서 값 확인후 일정 밝기 이상일 경우(핸드폰 라이트 비췄을때)에 배경 색 변경
        if ((uint32_t)ADC_Value[0] >= LUX_THRESHOLD)
            LCD_Clear(GRAY);
        else
            LCD_Clear(WHITE);
        // sensor value
        LCD_ShowNum(LCD_LUX_VAL_X, LCD_LUX_VAL_Y, ADC_Value[0], 4, BLUE, WHITE);
    }
}

void Init(void)
{
    SystemInit();
    RccInit();
    GpioInit();
    AdcInit();
    DMA_Configure();

    LCD_Init();
    Touch_Configuration();
    Touch_Adjust();
}

void RccInit(void)
{
    // Todo: DMA, ADC, port RCC init
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

void GpioInit(void)
{
    // Todo: 조도센서 GPIO Init
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void AdcInit(void)
{
    ADC_InitTypeDef ADC_InitStructure;

    // ADC1 Configuration, 조도센서 B0 가정
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE); // ADC1 enable
    ADC_ResetCalibration(ADC1);

    while (ADC_GetResetCalibrationStatus(ADC1))
        ;

    ADC_StartCalibration(ADC1);

    while (ADC_GetCalibrationStatus(ADC1))
        ;

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void DMA_Configure(void)
{
    DMA_InitTypeDef DMA_Instructure;

    DMA_DeInit(DMA1_Channel1);
    DMA_Instructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; // 어디에 있는걸 가져올지
    DMA_Instructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_Value[0]; // 가져온걸 어디에 쓸지
    DMA_Instructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_Instructure.DMA_BufferSize = 1;
    DMA_Instructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_Instructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_Instructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_Instructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_Instructure.DMA_Mode = DMA_Mode_Circular;
    DMA_Instructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_Instructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_Instructure);

    DMA_Cmd(DMA1_Channel1, ENABLE);
}
