#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "lcd.h"
#include "touch.h"
#include "math.h"

#define LCD_TEAM_NAME_X 20
#define LCD_TEAM_NAME_Y 50
#define LCD_STATUS_X 20
#define LCD_STATUS_Y 70

#define LCD_BUTTON_X 30
#define LCD_BUTTON_Y 100
#define LCD_BUTTON_W 50
#define LCD_BUTTON_H 50

void Init(void);
void RccInit(void);
void GpioInit(void);
void TIM_Configure(void);
void NvicInit(void);
void ledToggle(int num);

const int color[12] = {WHITE, CYAN, BLUE, RED, MAGENTA, LGRAY, GREEN, YELLOW, BROWN, BRRED, GRAY};

// timer counter
int timer_counter = 0;
int led2_flag = 0;
// led on/off
char ledOn;
// motor set
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
// motor angle
int motorAngle = 0;
int motorDir = 0;

int main()
{
    uint16_t pos_x, pos_y;
    uint16_t pix_x, pix_y;

    Init();

    ledOn = 0;

    LCD_Clear(WHITE);

    // team name
    LCD_ShowString(LCD_TEAM_NAME_X, LCD_TEAM_NAME_Y, "THU_02", BLUE, WHITE);
    // button
    LCD_DrawRectangle(LCD_BUTTON_X, LCD_BUTTON_Y, LCD_BUTTON_X + LCD_BUTTON_W, LCD_BUTTON_Y + LCD_BUTTON_H);
    LCD_ShowString(LCD_BUTTON_X + (LCD_BUTTON_W / 2), LCD_BUTTON_Y + (LCD_BUTTON_H / 2), "BUT", RED, WHITE);

    while (1)
    {
        if (ledOn == 0)
        {
            LCD_ShowString(LCD_STATUS_X, LCD_STATUS_Y, "OFF", RED, WHITE);
            motorDir = 0;
        }
        else
        {
            LCD_ShowString(LCD_STATUS_X, LCD_STATUS_Y, "ON ", RED, WHITE);
            motorDir = 1;
        }
        // get touch coordinate
        Touch_GetXY(&pos_x, &pos_y, 1);
        Convert_Pos(pos_x, pos_y, &pix_x, &pix_y);

        if (
            pix_x >= LCD_BUTTON_X &&
            pix_x <= LCD_BUTTON_X + LCD_BUTTON_W &&
            pix_y >= LCD_BUTTON_Y &&
            pix_x <= LCD_BUTTON_Y + LCD_BUTTON_H)
        {
            ledOn = !ledOn;
        }
    }
}

void Init(void)
{
    SystemInit();
    RccInit();
    GpioInit();
    TIM_Configure();
    NvicInit();

    LCD_Init();
    Touch_Configuration();
    Touch_Adjust();

    GPIO_SetBits(GPIOD, GPIO_Pin_2);
    GPIO_SetBits(GPIOD, GPIO_Pin_3);
}

void RccInit(void)
{
    // Todo: Init LED, motor Port, Timer
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //TIM2
    // LED1: PD2, LED2: PD3
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    // PWM motor: PB0
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    // TIM2
    RCC_APB2PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    // TIM3
    RCC_APB2PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
}

void GpioInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // LED 1, LED2 Init
    // Todo
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // TIM2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PWM motor Init
    // Todo
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void TIM_Configure(void)
{
    // led toggle timer
    TIM_TimeBaseInitTypeDef TIM2_InitStructure;

    // period: 1s
    // frequency: 1Hz
    uint16_t prescale1 = (uint16_t)(SystemCoreClock / 10000);

    TIM2_InitStructure.TIM_Period = 10000;
    TIM2_InitStructure.TIM_Prescaler = prescale1;
    TIM2_InitStructure.TIM_ClockDivision = 0;
    TIM2_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM2, &TIM2_InitStructure);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // motor pwm timer
    TIM_TimeBaseInitTypeDef TIM3_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // period: 20ms
    // frequency: 50Hz
    // 1 / 72,000,000 * 72,000 * 20 = 20ms
    uint16_t prescale = (uint16_t)(SystemCoreClock / 1000000);

    TIM3_InitStructure.TIM_Period = 20000;
    TIM3_InitStructure.TIM_Prescaler = prescale;
    TIM3_InitStructure.TIM_ClockDivision = 0;
    TIM3_InitStructure.TIM_CounterMode = TIM_CounterMode_Down;

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 2000;

    TIM_OC3Init(TIM3, &TIM_OCInitStructure);

    TIM_TimeBaseInit(TIM3, &TIM3_InitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void NvicInit(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void moveMotor()
{
    // Todo: Adjust motorAngle

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = motorAngle;

    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        timer_counter++;
        moveMotor();
        // Todo: LED toggle

        if (ledOn == 1)
        {
            if (timer_counter % 2 == 0)
            {
                ledToggle(1); // LED1 ON
            }
            else
            {
                ledToggle(2); // LED1 OFF
            }
            if (timer_counter % 5 == 0)
            {
                if (led2_flag == 0)
                {
                    ledToggle(3); // LED2 ON
                    led2_flag = 1;
                }
                else
                {
                    ledToggle(4); // LED2 OFF
                    led2_flag = 0;
                }
            }
            timer_counter %= 10;

            motorAngle += 100;
            if (motorAngle > 2000)
                motorAngle = 1000;
        }
        else
        {
            ledToggle(0); // LED OFF
            led2_flag = 0;

            motorAngle -= 100;
            if (motorAngle < 1000)
                motorAngle = 2000;
        }

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

void ledToggle(int num)
{
    // LED toggle OFF
    if (num == 0)
    {
        GPIO_SetBits(GPIOD, GPIO_Pin_2);
        GPIO_SetBits(GPIOD, GPIO_Pin_3);
    }
    // LED 1 ON
    else if (num == 1)
        GPIO_ResetBits(GPIOD, GPIO_Pin_2);
    // LED 1 OFF
    else if (num == 2)
        GPIO_SetBits(GPIOD, GPIO_Pin_2);
    // LED 2 ON
    else if (num == 3)
        GPIO_ResetBits(GPIOD, GPIO_Pin_3);
    // LED 2 OFF
    else if (num == 4)
        GPIO_SetBits(GPIOD, GPIO_Pin_3);
}
