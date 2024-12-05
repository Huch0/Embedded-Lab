#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

// #include "sensor.h"
#include "motor.h"
#include "ultra_sonic.h"
// #include "bluetooth.h"

int main()
{
    SystemInit();

    motor_init();
    // SensorInit();
    // CameraInit();
    // BluetoothInit();

    while (1)
    {
        // char user_input = get_user_input();

        if (motor_mode == MOTOR_MANUAL)
        {
            // manual_control(user_input);
        }
        else if (motor_mode == MOTOR_AUTO)
        {
            auto_control(front_ultra_sonic_distance, left_ultra_sonic_distance, right_ultra_sonic_distance, back_ultra_sonic_distance);
        }
    }

    return 0;
}