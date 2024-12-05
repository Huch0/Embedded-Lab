#include "delay.h"

void default_delay(void)
{
    int i;
    for (i = 0; i < DELAY_TIME; i++)
    {
    }
}

void delay(int time)
{
    int i;
    for (i = 0; i < time; i++)
    {
    }
}