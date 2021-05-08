/*
 * example dma transfer program for nucleoh743zi2
 * please refer to the reference manual to understand what is going on
 * https://www.st.com/resource/en/reference_manual/dm00314099-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
 */

#include "stm32h743xx.h"
#include "aux.h"

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
	TIM4->SR &= ~TIM_SR_UIF; //clear the update interrupt flag
	GPIOE->ODR ^= GPIO_ODR_OD1; // toggle bit 14
	return;
}

void DMA1_Stream0_IRQHandler(void){
	// this interrupt could be caused by either a transfer complete interrupt
	// OR a transfer error interrupt, it is up to software to check the LISR
	// (or HISR) and decide
	if( DMA1->LISR & DMA_LISR_TEIF0 ){
		// transmission err.
		DMA1->LIFCR |= DMA_LIFCR_CTEIF0; // clear transmit complete flag
		GPIOB->BSRR = GPIO_BSRR_BS14; //set bit
	}else{
		// transfer is successful (this implies TX is done)
		DMA1->LIFCR |= DMA_LIFCR_CTCIF0; // clear transmit complete flag
		GPIOB->BSRR = GPIO_BSRR_BS0; //set bit
	}
}

//void DMA1_Stream1_IRQHandler(void){
//	if( DMA1->LISR & DMA_LISR_TEIF1 ){
//		// transmission err.
//		DMA1->LIFCR |= DMA_LIFCR_CTEIF1; // clear transmit complete flag
//		GPIOB->BSRR = GPIO_BSRR_BS14; //set bit
//	}else{
//		// transfer is successful (this implies RX is done)
//		DMA1->LIFCR |= DMA_LIFCR_CTCIF1; // clear transmit complete flag
//		GPIOB->BSRR = GPIO_BSRR_BS0; //set bit
//	}
//}

int main(void){
	// enable SRAM2 clock (used and accesible by DMA)
	//RCC->AHB2ENR |= RCC_AHB2ENR_SRAM2EN; //seems to be unecessary

	// enable GPIOB and GPIOE clock
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOEEN;
	GPIOB->MODER &= ~GPIO_MODER_MODE0; //reset mode config to b00
	GPIOB->MODER |= (0b01 << GPIO_MODER_MODE0_Pos); //set mode config to b01 (output)
	//configure pin 14 on gpiob
	GPIOB->MODER &= ~GPIO_MODER_MODE14;
	GPIOB->MODER |= (0b01 << GPIO_MODER_MODE14_Pos); //set mode config to b01 (output)

	GPIOE->MODER &= ~GPIO_MODER_MODE1;
	GPIOE->MODER |= (0b01 << GPIO_MODER_MODE1_Pos); //set mode config to b01 (output)

	if (SysTick_Config (SystemCoreClock / 1000)) { // SysTick 1mSec
		//error, set bit and die
		GPIOB->BSRR = GPIO_BSRR_BS14;
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

	//------------------------DMA (USART) CONFIGURATION-------------------------------
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; //clock gating for DMA1
	//configure a transfer buffer (USART), memory increment (as oppose to peripheral increment PINC)
	// also enable transfer complete interrupt, transfer error interrupt
	// dir 01 -> memory to peripheral (TX)
	DMA1_Stream0->CR |= DMA_SxCR_TRBUFF | DMA_SxCR_MINC | (0b01 << DMA_SxCR_DIR_Pos) | DMA_SxCR_TCIE | DMA_SxCR_TEIE;
	// dir 00 -> peripheral to memory (RX)
	//DMA1_Stream1->CR |= DMA_SxCR_TRBUFF | DMA_SxCR_MINC | (0b00 << DMA_SxCR_DIR_Pos) | DMA_SxCR_TCIE | DMA_SxCR_TEIE;
	// configure dmamux for respective dma requests
	//see page 694 and 695 (Table 121.)
	DMAMUX1_Channel0->CCR |= 46U; //enable channel 0 for usart3_tx_dma
	//DMAMUX1_Channel1->CCR |= 45U; //enable channel 1 for usart3_rx_dma

	// USART init
	RCC->APB1LENR |= RCC_APB1LENR_USART3EN;
	USART3->CR3 |= USART_CR3_DMAT; //enable DMA xmit only. (USART_CR3_DMAR)
	USART3->BRR = 0x1a0b; //see page 2058, stm32h743 defaults to internal clock @ 64MHz
	//enable TX, RX and UART
	USART3->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
	//read page 2048 for rx and tx operations
	//--------------------------------------------------------------------------------

	//------------------------TIMER INTERRUPT-----------------------------------------
	// enable the timer4 clock to be gated by the rcc apb4lenr bus
	RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;
	TIM4->PSC = 64000; //prescaler 64000 (max 65535) on 64MHz clock means timer tick at 1Khz
	TIM4->ARR = 500; //timer interrupt trigger after 1000 ticks, so 1 second interrupt freq
	TIM4->CR1 |= TIM_CR1_URS | TIM_CR1_DIR; //overflow/underflow interrupt, count down
	TIM4->DIER |= TIM_DIER_UIE; // update interrupt ENABLED
	TIM4->CR1 |= TIM_CR1_CEN; //enable the counter

	//disable all interrupts
	//__disable_irq();
	NVIC_EnableIRQ(DMA1_Stream0_IRQn); //enable dma1_stream0 interrupts
	//NVIC_EnableIRQ(DMA1_Stream1_IRQn); //enable dma1_stream1 interrupts
	NVIC_EnableIRQ(TIM4_IRQn); //enable timer 4 interrupts
	////re-enable all global interrupts
	//__enable_irq();
	//--------------------------------------------------------------------------------

	//blink 10 times quickly
	for(int i=0;i<10;i++){
		GPIOB->BSRR = GPIO_BSRR_BS14; //set bit
		idle_delay(50);
		GPIOB->BSRR = GPIO_BSRR_BR14; //reset bit
		idle_delay(50);
	}

	char test[] = "hello world!\r\n";
	usart_printf(USART3, test);

	test[0] = 'y';
	usart_dma10_printf(USART3, test);

	while(1){
	}
}
