#include "string.h"
#include "stm32h743xx.h"
#include "aux.h"

/*
 * example blinky program
 * please refer to the reference manual to understand what is going on
 * https://www.st.com/resource/en/reference_manual/dm00314099-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
 */

uint32_t volatile msTicks;// counter for systicks

// SysTick Interrupt Handler
void SysTick_Handler (void) {
	msTicks++;
}

void idle_delay(uint32_t ms){
	uint32_t cur = msTicks;
	while(msTicks - cur < ms);
}

void TIM4_IRQHandler(void){
	GPIOB->ODR ^= GPIO_ODR_OD14; // toggle bit 14
}

int main(void){
	// enable GPIOB clock
	// this can be found on page 454 from the refman
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
	// alternative
	//RCC->AHB4ENR |= ( 1 << 1); //GPIOB is bit 1

	// configure pin 0 to be output mode
	// this can be found on page 537 from the refman
	GPIOB->MODER &= ~GPIO_MODER_MODE0; //reset mode config to b00
	GPIOB->MODER |= (0b01 << GPIO_MODER_MODE0_Pos); //set mode config to b01
	// the following alternative works
	//GPIOB->MODER |=  (1 << 0);
	//GPIOB->MODER &= ~(1 << 1);

	//configure pin 14
	GPIOB->MODER &= ~GPIO_MODER_MODE14;
	GPIOB->MODER |= (0b01 << GPIO_MODER_MODE14_Pos); //set mode config to b01

	if (SysTick_Config (SystemCoreClock / 1000)) { // SysTick 1mSec
		//error, set bit and die
		GPIOB->BSRR = GPIO_BSRR_BS0;
		while(1);
	}

	//------------------------USART EXAMPLE-------------------------------------------
	//configure GPIOC pin 10 and 11 for alternate function mode (uart)
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;
	GPIOC->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11); //default 0000
	GPIOC->MODER |= (0b10 << GPIO_MODER_MODE10_Pos);
	GPIOC->MODER |= (0b10 << GPIO_MODER_MODE11_Pos);
	GPIOC->AFR[1] &= ~(GPIO_AFRH_AFSEL10 | GPIO_AFRH_AFSEL11);
	// page 541 - alternate function configuration (AF7)
	// AFR[1] is the AFRH
	GPIOC->AFR[1] |= (0b0111 << GPIO_AFRH_AFSEL10_Pos);
	GPIOC->AFR[1] |= (0b0111 << GPIO_AFRH_AFSEL11_Pos);

	// USART init
	RCC->APB1LENR |= RCC_APB1LENR_USART3EN;
	USART3->BRR = 0x1a0b; //see page 2058, stm32h743 defaults to internal clock @ 64MHz
	//enable TX, RX and UART
	USART3->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
	//read page 2048 for rx and tx operations
	//--------------------------------------------------------------------------------

	//------------------------TIMER INTERRUPT-----------------------------------------
	// enable the timer4 clock to be gated by the rcc apb4lenr bus
	RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;
	TIM4->PSC = 64000; //prescaler 64000 (max 65535) on 64MHz clock means timer tick at 1Khz
	TIM4->ARR = 1000; //timer interrupt trigger after 1000 ticks, so 1 second interrupt freq
	//TIM4->CR1 |= TIM_CR1_URS; //overflow/underflow interrupt, count down
	//TIM4->CR1 |= TIM_CR1_DIR; //count down (do not set this to count up)
	//TIM4->DIER |= TIM_DIER_UIE; // update interrupt ENABLED
	//TIM4->EGR |= TIM_EGR_UG; // update generation
	TIM4->CR1 |= TIM_CR1_CEN; //enable the counter

	//enable all interrupts
	//__disable_irq(); //disable all global interrupts
	////NVIC_SetPriority(TIM4_IRQn, 1); //0 is default and highest interrupt priority
	//NVIC_EnableIRQ(TIM4_IRQn); //enable timer 4 interrupts
	////re-enable all global interrupts
	//__enable_irq();

	//blink 10 times quickly
	for(int i=0;i<10;i++){
		GPIOB->BSRR = GPIO_BSRR_BS0; //set bit
		//GPIOB->ODR ^= GPIO_ODR_OD0; //toggle bit
		idle_delay(50);
		GPIOB->BSRR = GPIO_BSRR_BR0; //reset bit
		//GPIOB->ODR ^= GPIO_ODR_OD0; //toggle bit
		idle_delay(50);
	}

	// main loop
	//char temp[] = "hello world\n\r";
	//usart_getc(USART3);

	while(1){
		usart_printf(USART3,"%lu\n\r", TIM4->CNT);
	}
}
