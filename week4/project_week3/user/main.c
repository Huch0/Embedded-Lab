#include "stm32f10x.h"

#define RCC_APB2ENR (*(volatile unsigned int *)0x40021018)

#define GPIOA_CRL (*(volatile unsigned int *)0x40010800)
#define GPIOA_IDR (*(volatile unsigned int *)0x40010808)

#define GPIOB_CRH (*(volatile unsigned int *)0x40010C04)
#define GPIOB_IDR (*(volatile unsigned int *)0x40010C08)

#define GPIOC_CRL (*(volatile unsigned int *)0x40011000)
#define GPIOC_CRH (*(volatile unsigned int *)0x40011004)
#define GPIOC_IDR (*(volatile unsigned int *)0x40011008)

#define GPIOD_CRL (*(volatile unsigned int *)0x40011400)
#define GPIOD_BSRR (*(volatile unsigned int *)0x40011410)
#define GPIOD_BRR (*(volatile unsigned int *)0x40011414)

void delay()
{
    for (int i = 0; i < 1000000; i++)
        ;
}

int main(void)
{

    // Key 1 PC4
    GPIOC_CRL &= 0xFFF0FFFF;
    GPIOC_CRL |= 0x00080000;
    // Key 2 PB10
    GPIOB_CRH &= 0xFFFFF0FF;
    GPIOB_CRH |= 0x00000800;
    // Key 3 PC13
    GPIOC_CRH &= 0xFF0FFFFF;
    GPIOC_CRH |= 0x00800000;
    // Key 4 PA0
    GPIOA_CRL &= 0xFFFFFFF0;
    GPIOA_CRL |= 0x00000008;

    // clock
    // PORT A, B, C, D ON
    RCC_APB2ENR |= 0x3C;

    // Relay Module
    // PD1, PD2
    GPIOD_CRL &= 0xFFFFF00F;
    GPIOD_CRL |= 0xFFFFF33F;

    while (1)
    {
        if ((GPIOC_IDR & 0x10) == 0) // Key1
        {
            GPIOD_BSRR |= 0x40002; // PD1 set, PD2 reset
        }
        else if ((GPIOB_IDR & 0x0400) == 0) // Key2
        {
            GPIOD_BSRR |= 0x20004; // PD1 reset, PD2 set
        }
        else if ((GPIOC_IDR & 0x2000) == 0) // key 3
        {
            GPIOD_BSRR |= 0x40002; // PD1 set, PD2 reset
            delay();
            delay();
            GPIOD_BSRR |= 0x20004; // PD1 reset, PD2 set
            delay();
            delay();
            GPIOD_BSRR |= 0x60000; // PD1 reset, PD2 reset
        }
        else if ((GPIOA_IDR & 0x01) == 0) // key 4
        {
            GPIOD_BSRR |= 0x60000; // PD1 reset, PD2 reset
        }
    }
    return 0;
}