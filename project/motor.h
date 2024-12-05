#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "delay.h"

// pin mapping
// motor front left | PD2 | PD3
#define MOTOR_FL1_PORT GPIOD
#define MOTOR_FL1_PIN GPIO_Pin_2
#define MOTOR_FL2_PORT GPIOD
#define MOTOR_FL2_PIN GPIO_Pin_3

// motor front right | PD4 | PD7
#define MOTOR_FR1_PORT GPIOD
#define MOTOR_FR1_PIN GPIO_Pin_4
#define MOTOR_FR2_PORT GPIOD
#define MOTOR_FR2_PIN GPIO_Pin_7

// motor back left | PD8 | PD9
#define MOTOR_BL1_PORT GPIOD
#define MOTOR_BL1_PIN GPIO_Pin_8
#define MOTOR_BL2_PORT GPIOD
#define MOTOR_BL2_PIN GPIO_Pin_9

// motor back right | PD14 | PD15
#define MOTOR_BR1_PORT GPIOD
#define MOTOR_BR1_PIN GPIO_Pin_14
#define MOTOR_BR2_PORT GPIOD
#define MOTOR_BR2_PIN GPIO_Pin_15

// motor mode
#define MOTOR_MANUAL 0
#define MOTOR_AUTO 1

extern int motor_mode;

void motor_init(void);
void motor_rcc_configure(void);
void motor_gpio_configure(void);

void manual_control(void);
void auto_control(float, float, float, float);

#define SAFE_DISTANCE 10

void go_forward(void);
void go_backward(void);
void turn_left(void);
void turn_right(void);
void stop(void);