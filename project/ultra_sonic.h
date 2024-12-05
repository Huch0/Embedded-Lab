#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

#define INF 1000000
extern float front_ultra_sonic_distance = INF;
extern float left_ultra_sonic_distance = INF;
extern float right_ultra_sonic_distance = INF;
extern float back_ultra_sonic_distance = INF;

void ultra_sonic_init(void);
void ultra_sonic_rcc_configure(void);
void ultra_sonic_gpio_configure(void);
void ultra_sonic_exti_configure(void);
void ultra_sonic_timer_configure(void);
void ultra_sonic_trigger(void);

// pin mapping
#define US_FRONT_TRIG_PORT GPIOB
#define US_FRONT_TRIG_PIN GPIO_Pin_0
#define US_FRONT_ECHO_PORT GPIOB
#define US_FRONT_ECHO_PIN GPIO_Pin_1

#define US_LEFT_TRIG_PORT GPIOB
#define US_LEFT_TRIG_PIN GPIO_Pin_2
#define US_LEFT_ECHO_PORT GPIOB
#define US_LEFT_ECHO_PIN GPIO_Pin_3

#define US_RIGHT_TRIG_PORT GPIOB
#define US_RIGHT_TRIG_PIN GPIO_Pin_4
#define US_RIGHT_ECHO_PORT GPIOB
#define US_RIGHT_ECHO_PIN GPIO_Pin_5

#define US_BACK_TRIG_PORT GPIOB
#define US_BACK_TRIG_PIN GPIO_Pin_6
#define US_BACK_ECHO_PORT GPIOB
#define US_BACK_ECHO_PIN GPIO_Pin_7

#define US_TIMER TIM2
