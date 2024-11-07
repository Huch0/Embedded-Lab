#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "lcd.h"
#include "touch.h"

// 색상 배열 정의
int color[12] = {WHITE, CYAN, BLUE, RED, MAGENTA, LGRAY, GREEN, YELLOW, BROWN, BRRED, GRAY};

uint16_t new_x, new_y, prev_x, prev_y;
uint16_t value = 0; // value of photo resistor

void RCC_Configure(void);
void GPIO_Configure(void);
void ADC_Configure(void);
void NVIC_Configure(void);
// void ADC1_2_IRQHandler(void);
void Delay(void);

void RCC_Configure(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}
void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_ADC;

    // Initialize the GPIO pins using the structure 'GPIO_InitTypeDef' and the function 'GPIO_Init'
    GPIO_ADC.GPIO_Pin = GPIO_Pin_1;
    GPIO_ADC.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_ADC.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_ADC); // PB1: ADC_IN1
}

void NVIC_Configure(void)
{
    NVIC_InitTypeDef NVIC_ADC;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    // Initialize the NVIC using the structure 'NVIC_InitTypeDef' and the function 'NVIC_Init'

    NVIC_EnableIRQ(ADC1_2_IRQn);
    NVIC_ADC.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_ADC.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_ADC.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_ADC.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_ADC);
}

void ADC_Configure(void)
{
    ADC_InitTypeDef ADC_InitStructure;

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);

    while (ADC_GetResetCalibrationStatus(ADC1))
        ;

    ADC_StartCalibration(ADC1);

    while (ADC_GetCalibrationStatus(ADC1))
        ;

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void ADC1_2_IRQHandler(void)
{
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
    {
        value = ADC_GetConversionValue(ADC1);

        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }
}

int main()
{

    // LCD 관련 설정은 LCD_Init에 구현되어 있으므로 여기서 할 필요 없음
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    ADC_Configure();
    NVIC_Configure();
    // ------------------------------------

    LCD_Init();
    Touch_Configuration();
    Touch_Adjust();
    LCD_Clear(WHITE);

    LCD_ShowString(20, 50, "THU_TEAM02", GRAY, WHITE);

    while (1)
    {
        // TODO : LCD 값 출력 및 터치 좌표 읽기
        Touch_GetXY(&new_x, &new_y, 1);
        Convert_Pos(new_x, new_y, &prev_x, &prev_y);

        LCD_DrawCircle(prev_x, prev_y, 3);
        LCD_ShowNum(50, 80, prev_x, 4, GRAY, WHITE);
        LCD_ShowNum(50, 100, prev_y, 4, GRAY, WHITE);
        LCD_ShowNum(20, 120, value, 4, GRAY, WHITE);
    }

    return 0;
}
