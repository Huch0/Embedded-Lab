#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "lcd.h"
#include "touch.h"

int color[12] = {WHITE, CYAN, BLUE, RED, MAGENTA, LGRAY, GREEN, YELLOW, BROWN, BRRED, GRAY};
int btn = 0;
int Tim2_cnt = 0;
int flag_cnt5 = 0;
int motorAngle = 1000; // Initial motor angle
uint16_t value;

void RCC_Configure(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // TIM2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // TIM3
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); // LED
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // TIM2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // TIM3
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // Alternate Function IO
}

void GPIO_Configure(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // TIM2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // TIM3
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // LED1 and LED2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void delay(void) {
   int i;
   for (i = 0; i < 8000000; i++) {}
}

void PWM_Configure(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_TimeBaseStructure.TIM_Period = 20000; // 20ms
    TIM_TimeBaseStructure.TIM_Prescaler = 72;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = motorAngle;

    TIM_OC3Init(TIM3, &TIM_OCInitStructure);

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void TIM_Configure(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    TIM_TimeBaseStructure.TIM_Period = 10000; // 1s
    TIM_TimeBaseStructure.TIM_Prescaler = 7200;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void moveMotor() {
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = motorAngle;

    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
}

void TIM2_IRQHandler() {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        if (btn) {
            if (Tim2_cnt % 2 == 0) {
                GPIO_SetBits(GPIOD, GPIO_Pin_2); // LED1 ON
            } else {
                GPIO_ResetBits(GPIOD, GPIO_Pin_2); // LED1 OFF
            }

            if (Tim2_cnt % 5 == 0) {
                if (flag_cnt5 == 0) {
                    GPIO_SetBits(GPIOD, GPIO_Pin_3); // LED2 ON
                    flag_cnt5 = 1;
                } else {
                    GPIO_ResetBits(GPIOD, GPIO_Pin_3); // LED2 OFF
                    flag_cnt5 = 0;
                }
            }

            Tim2_cnt++;
            motorAngle += 100; // Increment motor angle
            if (motorAngle > 2000) motorAngle = 1000; // Reset angle if out of bounds
        } else {
            Tim2_cnt = 0;
            flag_cnt5 = 0;
            motorAngle -= 100; // Decrement motor angle
            if (motorAngle < 1000) motorAngle = 2000; // Reset angle if out of bounds
        }
        moveMotor(); // Update motor position
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

int main() {
    uint16_t x, y, nx, ny;

    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    TIM_Configure();
    PWM_Configure();
    NVIC_Configure();

    LCD_Init();
    Touch_Configuration();
    Touch_Adjust();
    LCD_Clear(WHITE);

    char str[11] = "THU_Team02";
    LCD_ShowString(0, 0, str, BLACK, WHITE);
    LCD_DrawRectangle(50, 50, 100, 100);
    LCD_ShowString(60, 65, "BUT", RED, WHITE);
    LCD_ShowString(0, 20, "OFF", RED, WHITE);

    while (1) {
        Touch_GetXY(&x, &y, 1);
        Convert_Pos(x, y, &nx, &ny);
        
        if (nx >= 50 && nx <= 100 && ny >= 50 && ny <= 100) {
            if (btn == 0) {
                btn = 1;
                LCD_ShowString(0, 20, " ON", RED, WHITE);
            } else {
                btn = 0;
                LCD_ShowString(0, 20, "OFF", RED, WHITE);
                GPIO_ResetBits(GPIOD, GPIO_Pin_2); // LED1 OFF
                GPIO_ResetBits(GPIOD, GPIO_Pin_3); // LED2 OFF
            }
        }
    }
}
