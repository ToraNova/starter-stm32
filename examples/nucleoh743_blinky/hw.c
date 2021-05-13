/*
 * actual hardware implementation file
 * this should differ based on hardware
 *
 * for nucleoh743zi2
 *
 * author: toranova (chia_jason96@live.com)
 */

extern void sysclk_init(void);
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <inttypes.h>
#include "stm32h743xx.h"
#include "hw.h"

// local macro defines

// useful macros
#define SRM_EXPAND(ST, PIN) 	ST ? GPIO_BSRR_BS##PIN : GPIO_BSRR_BR##PIN

#define LOG_USART USART2
#define SER_USART USART3
#define LOGGER_BUFSZ 80

void TIM4_IRQHandler(void){
	if( TIM4->SR & TIM_SR_UIF ){
		TIM4->SR &= ~TIM_SR_UIF; //clear the update interrupt flag
		//GPIOE->ODR ^= GPIO_ODR_OD1; // toggle pin 1 on port E
		timer_expire_callback();
		return;
	}
}

void USART3_IRQHandler(void) {
    	/* Check for RX not empty interrupt */
	if( USART3->ISR & USART_ISR_RXNE_RXFNE ){
		//clear RXFNE flag
		//USART3->ICR |= USART_ICR_RXNE_RXFNE;
		serial_read_callback( (uint8_t *) &(SER_USART->RDR), 1 );
	}else if( USART3->ISR & USART_ISR_ORE ){
		//overrun error (when tx speed overwhelms the mcu receive speed)
		//use FIFO to overcome this
		USART3->ICR |= USART_ICR_ORECF;
		logger_printf("ua3 irq: overrun.\n");
	}
}

void hardware_init(void){
	// enable sysclk initialization function to be defined elsewhere
	// (if we require so, due to excessive length)
	// if undefined, the default function in aux.c is called (does nothing)
	sysclk_init();

	// enable GPIOB clock
	// this can be found on page 454 from the refman
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOEEN;
	// alternative
	//RCC->AHB4ENR |= ( 1 << 1); //GPIOB is bit 1

	// configure pin 0 to be output mode
	// this can be found on page 537 from the refman
	//reset mode config to b00 for pin 0 and 14 on port b
	GPIOB->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE14);
	GPIOB->MODER |= (0b01 << GPIO_MODER_MODE0_Pos); //set mode config to b01 (output)
	GPIOB->MODER |= (0b01 << GPIO_MODER_MODE14_Pos);
	// the following alternative works
	//GPIOB->MODER |=  (1 << 0);
	//GPIOB->MODER &= ~(1 << 1);

	//configure pin 14
	GPIOB->MODER &= ~GPIO_MODER_MODE14;
	GPIOB->MODER |= (0b01 << GPIO_MODER_MODE14_Pos); //set mode config to b01

	// enable GPIOE clock
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;
	GPIOE->MODER &= ~GPIO_MODER_MODE1;
	GPIOE->MODER |= (0b01 << GPIO_MODER_MODE1_Pos);

	if (SysTick_Config (SystemCoreClock / 1000)) { // SysTick 1mSec
		//error, set bit and die
		GPIOB->BSRR = GPIO_BSRR_BS14;
		while(1);
	}

	//------------------------USART EXAMPLE-------------------------------------------
	//enable usart3
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
	//enable TX, RX, RXNE interrupt and USART3 itself.
	USART3->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_UE;
	//read page 2048 for rx and tx operations

	// USART 2 gpio config
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;
	GPIOD->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6); //default 0000
	GPIOD->MODER |= (0b10 << GPIO_MODER_MODE5_Pos);
	GPIOD->MODER |= (0b10 << GPIO_MODER_MODE6_Pos);
	GPIOD->AFR[0] &= ~(GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL6);
	GPIOD->AFR[0] |= (0b0111 << GPIO_AFRL_AFSEL5_Pos);
	GPIOD->AFR[0] |= (0b0111 << GPIO_AFRL_AFSEL6_Pos);

	//enable usart2
	RCC->APB1LENR |= RCC_APB1LENR_USART2EN; //clock gating for USART
	USART2->BRR = (uint32_t) (64000000/9600); // baud rate of 9600 if APB clock is 64MHz
	// USART 2 is used for logging (no rx)
	USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
	//--------------------------------------------------------------------------------

	//------------------------TIMER INTERRUPT-----------------------------------------
	// enable the timer4 clock to be gated by the rcc apb4lenr bus
	RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;
	TIM4->PSC = 64000; //prescaler 64000 (max 65535) on 64MHz clock means timer tick at 1Khz
	TIM4->ARR = 100; //timer interrupt trigger after 1000 ticks, so 1 second interrupt freq
	TIM4->CR1 |= TIM_CR1_URS | TIM_CR1_DIR; //overflow/underflow interrupt, count down
	TIM4->DIER |= TIM_DIER_UIE; // update interrupt ENABLED
	TIM4->CR1 |= TIM_CR1_CEN; //enable the counter

	//disable all interrupts
	//__disable_irq();
	//NVIC_SetPriority(TIM4_IRQn, 1); //0 is default and highest interrupt priority
	NVIC_EnableIRQ(TIM4_IRQn); //enable timer 4 interrupts
	NVIC_EnableIRQ(USART3_IRQn); //enable usart3 interrupts
	//re-enable all global interrupts
	//__enable_irq();
}

// block execution until a character is successfully sent
// this is for internal use only
void uart_putc(USART_TypeDef *handle, uint8_t c){
	handle->TDR = c; // send it back out
	while(!(handle->ISR & USART_ISR_TC)); // wait for tx to be complete
}

void serial_write(uint8_t *buf, uint16_t len){
	for(size_t i=0; i<len;i++){
		uart_putc(SER_USART, buf[i]);
	}
}

void gpio_write(uint8_t arg, uint8_t state){
	//uint32_t sval = 0;
	switch(arg){
		case 0:
			GPIOB->BSRR |= SRM_EXPAND(state, 0);
			break;
		case 1:
			GPIOB->BSRR |= SRM_EXPAND(state, 14);
			break;
		case 2:
			GPIOE->BSRR |= SRM_EXPAND(state, 1);
			break;
	}
}

// format print a string and its arguments to a logger usart
// automatically prints a carriage return
void logger_printf(const char *msg, ...) {
	char buf[LOGGER_BUFSZ];
	va_list args;
	va_start(args,msg);
	vsprintf(buf,msg,args);
	for(size_t i=0; i<strlen(buf); i++){
		uart_putc(LOG_USART, buf[i]);
		if(buf[i] == '\n'){
			uart_putc(LOG_USART, '\r');
		}
	}
}

void logger_print32(uint32_t regval){
	logger_printf("[%08" PRIx32 "]", regval);
}

/*
 * block execution until a character is read from the recv buffer
 * not actually used. serves as reference only
uint8_t serial_getc(void){
	while(!(SER_USART->ISR & USART_ISR_RXNE_RXFNE));
	return SER_USART->RDR;
}
*/
