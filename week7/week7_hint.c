
#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"

#include "misc.h"

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void EXTI_Configure(void);
void USART1_Init(void);
void NVIC_Configure(void);

void EXTI15_10_IRQHandler(void);

void Delay(void);

void sendDataUART1(uint16_t data);

//---------------------------------------------------------------------------------------------------

void RCC_Configure(void)
{
	// TODO: Enable the APB2 peripheral clock using the function 'RCC_APB2PeriphClockCmd'
	
	/* UART TX/RX port clock enable */
	// 포트A enable
	/* Button 1,2,3 port clock enable */
	// 포트B,C enable
	/* LED port clock enable */
	// 포트D enable
	/* USART1 clock enable */
	// USART1 enable
	/* Alternate Function IO clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	// TODO: Initialize the GPIO pins using the structure 'GPIO_InitTypeDef' and the function 'GPIO_Init'
	
    /* Button 1,2,3 pin setting */
    // PC4, PC13, PB10 설정

    /* LED pin setting*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
    /* UART pin setting */
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    // speed: 50, Mode: AF_PP 
    //RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    // speed: 50, Mode: IN_FLOATING or (IPD, IPU 둘다 설정)
	
}

void EXTI_Configure(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    // TODO: Select the GPIO pin (button) used as EXTI Line using function 'GPIO_EXTILineConfig'
    // TODO: Initialize the EXTI using the structure 'EXTI_InitTypeDef' and the function 'EXTI_Init'
	
    /* Button 1 */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Button 2 */
	// EXTI 10
    /* Button 3 */
    // EXTI 13 

    // NOTE: do not select the UART GPIO pin used as EXTI Line here
}

void USART1_Init(void)
{
	USART_InitTypeDef USART1_InitStructure;

	// Enable the USART1 peripheral
	USART_Cmd(USART1, ENABLE);
	
	// TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
    // BaudRate: 9600
    // WordLength: 8b
    // Stopbit: 1
    // Parity: No
    // Mode: Rx | Tx
    // HardwareFlowControl: None
    // USART_Init(???); 
	// TODO: Enable the USART1 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
	// USART_ITConfig(???);
}

void NVIC_Configure(void) {

    NVIC_InitTypeDef NVIC_InitStructure;
    
    // TODO: fill the arg you want
    // 선점을 위해 그룹 1이상 사용
    // PreemptionPriority와 SubPriority가 모두 같다면 새로운 interrupt가 항상 선점 <- 구현 목표
    // NVIC_PriorityGroupConfig(???);

    // TODO: Initialize the NVIC using the structure 'NVIC_InitTypeDef' and the function 'NVIC_Init'
	
    // Button1
    // EXTI4_IRQn

    // Button2,3
    // EXTI15_10_IRQn

    // UART1
    // 'NVIC_EnableIRQ' is only required for USART setting
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ???; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = ???; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void USART1_IRQHandler() {
	uint16_t word;
    if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET){
    	// the most recent received data by the USART1 peripheral
        word = USART_ReceiveData(USART1);

        // TODO implement
        /*  받은 (uint8_t)word가 'a'일때, 'b'일때 처리를 해줘야함.
            interrupt handler안에서는 최대한 빠른 동작을 수행해야함
            따라서 mode와 같은 전역 변수등을 설정하여 해당 변수를 수정한뒤,
            메인 루프에서 이것을 확인한 뒤 그에 맞는 동작을 수행하도록 구현하는 것이 좋음 */
        // clear 'Read data register not empty' flag
    	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    }
}

void EXTI15_10_IRQHandler(void) { // when the Button 2,3 is pressed

    if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == Bit_RESET) {
			// TODO implement
            // 위의 USART1_IRQHandler에서 'b'를 수신받았을때의 처리와 동일하게 수행
		}
		EXTI_ClearITPendingBit(EXTI_Line10);
	}
    if (EXTI_GetITStatus(EXTI_Line13) != RESET) {
		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET) {
			// TODO implement
            /*  USART로 메세지 전송
                하지만 interrupt handler안에서 바로 전송 X
                따라서 여기서도 USART 전송을 위한 플래그, 시그널 변수를 수정하고
                메인 루프에서 이 변수의 변화를 감지하면(if등) 메세지 전송을 수행하는 식으로 처리*/
		}
        EXTI_ClearITPendingBit(EXTI_Line13);
	}
}

// TODO: Button 1 interrupt handler implement, referent above function
void EXTI4_IRQHandler(void) {
    /*  위의 EXTI15_10_IRQHandler와 비슷한 처리
        USART1_IRQHandler에서 'a'를 수신받았을때의 처리와 동일하게 수행
     */

}

void Delay(void) {
	int i;
	for (i = 0; i < 2000000; i++) {}
}

void sendDataUART1(uint16_t data) {
	/* Wait till TC is set */
	while ((USART1->SR & USART_SR_TC) == 0);
	USART_SendData(USART1, data);
}

int main(void)
{

    SystemInit();

    RCC_Configure();

    GPIO_Configure();

    EXTI_Configure();

    USART1_Init();

    NVIC_Configure();

    while (1) {
    	// TODO: implement 
    	/*  현재 모드를 확인
            만약 순방향이라면 그에 맞는 led 키는 동작 수행
            역방향도 처리
            led를 순차적으로 키려면 모든 led를 끄고 해당하는 index led만 키는 방식으로 구현
            이 index의 증가, 감소는 위의 모드에 따라 다르게 결정됨
            USART 시그널 변수 확인 후 보내야한다면 메세지를 보내고 해당 변수 다시 초기화
        */

        /* Pesudo Code, mode, usart_signal은 전역변수로 두어 interrupt handler에서 수정
            if(mode == 'a') {
                // 순방향
                led_index = (led_index + 1) % 4;
            }
            else if(mode == 'b') {
                // 역방향
                if(led_index == 0) {
                    led_index = 4;
                }
                led_index = (led_index - 1) % 4;
            }
            
            // 모든 led 끄기
            turn_off_all_led();
            // 현재 index led만 키기
            turn_on_led(led_index);

            // if(usart_signal == true) {
                for (i = 0; i < buf_length; i++) {
                    sendDataUART1(buf[i]);
                }
                usart_signal = false;
            }
        */
    	// Delay
    	Delay();
    }
    return 0;
}
