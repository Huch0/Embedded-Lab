#include "ultra_sonic.h"

void ultra_sonic_init(void)
{
    ultra_sonic_rcc_configure();
    ultra_sonic_gpio_configure();
    ultra_sonic_exti_configure();
    ultra_sonic_timer_configure();
}

void ultra_sonic_rcc_configure(void)
{
    RCC_APB2PeriphClockCmd(US_FRONT_ECHO_PORT, ENABLE);
    RCC_APB2PeriphClockCmd(US_FRONT_TRIG_PORT, ENABLE);
    RCC_APB2PeriphClockCmd(US_LEFT_ECHO_PORT, ENABLE);
    RCC_APB2PeriphClockCmd(US_LEFT_TRIG_PORT, ENABLE);
    RCC_APB2PeriphClockCmd(US_RIGHT_ECHO_PORT, ENABLE);
    RCC_APB2PeriphClockCmd(US_RIGHT_TRIG_PORT, ENABLE);
    RCC_APB2PeriphClockCmd(US_BACK_ECHO_PORT, ENABLE);
    RCC_APB2PeriphClockCmd(US_BACK_TRIG_PORT, ENABLE);

    RCC_APB1PeriphClockCmd(US_TIMER, ENABLE);
}

void ultra_sonic_gpio_configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = US_FRONT_TRIG_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(US_FRONT_TRIG_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = US_FRONT_ECHO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(US_FRONT_ECHO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = US_LEFT_TRIG_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(US_LEFT_TRIG_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = US_LEFT_ECHO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(US_LEFT_ECHO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = US_RIGHT_TRIG_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(US_RIGHT_TRIG_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = US_RIGHT_ECHO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(US_RIGHT_ECHO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = US_BACK_TRIG_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(US_BACK_TRIG_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = US_BACK_ECHO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(US_BACK_ECHO_PORT, &GPIO_InitStructure);
}