#include "motor.h"

void motor_init(void)
{
    motor_rcc_configure();
    motor_gpio_configure();
}

void motor_rcc_configure(void)
{
    RCC_APB2PeriphClockCmd(MOTOR_FL1_PORT, ENABLE);
    RCC_APB2PeriphClockCmd(MOTOR_FL2_PORT, ENABLE);
    RCC_APB2PeriphClockCmd(MOTOR_FR1_PORT, ENABLE);
    RCC_APB2PeriphClockCmd(MOTOR_FR2_PORT, ENABLE);
    RCC_APB2PeriphClockCmd(MOTOR_BL1_PORT, ENABLE);
    RCC_APB2PeriphClockCmd(MOTOR_BL2_PORT, ENABLE);
    RCC_APB2PeriphClockCmd(MOTOR_BR1_PORT, ENABLE);
    RCC_APB2PeriphClockCmd(MOTOR_BR2_PORT, ENABLE);
}

void motor_gpio_configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = MOTOR_FL1_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(MOTOR_FL1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MOTOR_FL2_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(MOTOR_FL2_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MOTOR_FR1_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(MOTOR_FR1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MOTOR_FR2_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(MOTOR_FR2_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MOTOR_BL1_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(MOTOR_BL1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MOTOR_BL2_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(MOTOR_BL2_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MOTOR_BR1_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(MOTOR_BR1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MOTOR_BR2_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(MOTOR_BR2_PORT, &GPIO_InitStructure);
}

void manual_control(void)
{
    if (motor_mode == MOTOR_AUTO)
    {
        // Raise error
    }
    // TODO
}

void auto_control(float f_distance, float l_distance, float r_distance, float b_distance)
{
    if (f_distance > SAFE_DISTANCE)
    {
        go_forward();
    }
    else if (l_distance > SAFE_DISTANCE)
    {
        turn_left();
    }
    else if (r_distance > SAFE_DISTANCE)
    {
        turn_right();
    }
    else if (b_distance > SAFE_DISTANCE)
    {
        go_backward();
    }
    else
    {
        stop();
    }
}

void go_forward(void)
{
    GPIO_SetBits(MOTOR_FL1_PORT, MOTOR_FL1_PIN);
    GPIO_ResetBits(MOTOR_FL2_PORT, MOTOR_FL2_PIN);

    GPIO_SetBits(MOTOR_FR1_PORT, MOTOR_FR1_PIN);
    GPIO_ResetBits(MOTOR_FR2_PORT, MOTOR_FR2_PIN);

    GPIO_SetBits(MOTOR_BL1_PORT, MOTOR_BL1_PIN);
    GPIO_ResetBits(MOTOR_BL2_PORT, MOTOR_BL2_PIN);

    GPIO_SetBits(MOTOR_BR1_PORT, MOTOR_BR1_PIN);
    GPIO_ResetBits(MOTOR_BR2_PORT, MOTOR_BR2_PIN);

    default_delay();
    stop();
}

void go_backward(void)
{
    GPIO_ResetBits(MOTOR_FL1_PORT, MOTOR_FL1_PIN);
    GPIO_SetBits(MOTOR_FL2_PORT, MOTOR_FL2_PIN);

    GPIO_ResetBits(MOTOR_FR1_PORT, MOTOR_FR1_PIN);
    GPIO_SetBits(MOTOR_FR2_PORT, MOTOR_FR2_PIN);

    GPIO_ResetBits(MOTOR_BL1_PORT, MOTOR_BL1_PIN);
    GPIO_SetBits(MOTOR_BL2_PORT, MOTOR_BL2_PIN);

    GPIO_ResetBits(MOTOR_BR1_PORT, MOTOR_BR1_PIN);
    GPIO_SetBits(MOTOR_BR2_PORT, MOTOR_BR2_PIN);

    default_delay();
    stop();
}

void turn_left(void)
{
    // Front Left: go forward
    GPIO_SetBits(MOTOR_FL1_PORT, MOTOR_FL1_PIN);
    GPIO_ResetBits(MOTOR_FL2_PORT, MOTOR_FL2_PIN);

    // Front Right: go backward
    GPIO_ResetBits(MOTOR_FR1_PORT, MOTOR_FR1_PIN);
    GPIO_SetBits(MOTOR_FR2_PORT, MOTOR_FR2_PIN);

    // Back Left: Stop
    GPIO_ResetBits(MOTOR_BL1_PORT, MOTOR_BL1_PIN);
    GPIO_ResetBits(MOTOR_BL2_PORT, MOTOR_BL2_PIN);

    // Back Right: go forward
    GPIO_SetBits(MOTOR_BR1_PORT, MOTOR_BR1_PIN);
    GPIO_ResetBits(MOTOR_BR2_PORT, MOTOR_BR2_PIN);

    default_delay();
    stop();
}

void turn_right(void)
{
    // Front Left: go backward
    GPIO_ResetBits(MOTOR_FL1_PORT, MOTOR_FL1_PIN);
    GPIO_SetBits(MOTOR_FL2_PORT, MOTOR_FL2_PIN);

    // Front Right: go forward
    GPIO_SetBits(MOTOR_FR1_PORT, MOTOR_FR1_PIN);
    GPIO_ResetBits(MOTOR_FR2_PORT, MOTOR_FR2_PIN);

    // Back Left: go forward
    GPIO_SetBits(MOTOR_BL1_PORT, MOTOR_BL1_PIN);
    GPIO_ResetBits(MOTOR_BL2_PORT, MOTOR_BL2_PIN);

    // Back Right: Stop
    GPIO_ResetBits(MOTOR_BR1_PORT, MOTOR_BR1_PIN);
    GPIO_ResetBits(MOTOR_BR2_PORT, MOTOR_BR2_PIN);

    default_delay();
    stop();
}

void stop(void)
{
    GPIO_ResetBits(MOTOR_FL1_PORT, MOTOR_FL1_PIN);
    GPIO_ResetBits(MOTOR_FL2_PORT, MOTOR_FL2_PIN);

    GPIO_ResetBits(MOTOR_FR1_PORT, MOTOR_FR1_PIN);
    GPIO_ResetBits(MOTOR_FR2_PORT, MOTOR_FR2_PIN);

    GPIO_ResetBits(MOTOR_BL1_PORT, MOTOR_BL1_PIN);
    GPIO_ResetBits(MOTOR_BL2_PORT, MOTOR_BL2_PIN);

    GPIO_ResetBits(MOTOR_BR1_PORT, MOTOR_BR1_PIN);
    GPIO_ResetBits(MOTOR_BR2_PORT, MOTOR_BR2_PIN);
}