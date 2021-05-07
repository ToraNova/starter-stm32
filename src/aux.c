#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <inttypes.h>
#include "stm32h743xx.h"
#include "aux.h"
#include "halper.h"

uint32_t volatile sys_msticks;// counter for systicks

uint32_t board_millis(void){
	return sys_msticks;
}

void board_init(void){

	sysclk_pll_hse_init();

	//enable all the gpio clocks
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN;
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOEEN | RCC_AHB4ENR_GPIOGEN;
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOHEN | RCC_AHB4ENR_GPIOIEN | RCC_AHB4ENR_GPIOJEN;

	// enable GPIOB and GPIOE clock
	GPIOB->MODER &= ~GPIO_MODER_MODE0; //reset mode config to b00
	GPIOB->MODER |= (0b01 << GPIO_MODER_MODE0_Pos); //set mode config to b01 (output)
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
	//USART3->BRR = 0x1a0b; //see page 2058, stm32h743 defaults to internal clock @ 64MHz
	// usart on 84MHz
	USART3->BRR = (uint32_t) (84000000 / 9600);
	//enable TX, RX and UART
	USART3->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
	//read page 2048 for rx and tx operations
	//--------------------------------------------------------------------------------

	//------------------------TIMER INTERRUPT-----------------------------------------
	// enable the timer4 clock to be gated by the rcc apb4lenr bus
	RCC->APB1LENR |= RCC_APB1LENR_TIM4EN; //APB1 is on 168MHz
	TIM4->PSC = 7200; //prescaler 64000 (max 65535) on 64MHz clock means timer tick at 1Khz
	TIM4->ARR = 23333; //timer interrupt trigger after 1000 ticks, so 1 second interrupt freq
	TIM4->CR1 |= TIM_CR1_URS | TIM_CR1_DIR; //overflow/underflow interrupt, count down
	TIM4->DIER |= TIM_DIER_UIE; // update interrupt ENABLED
	//TIM4->CR1 |= TIM_CR1_CEN; //enable the counter

	//-----------------------USB clock configuration----------------------------------
	// stm32h7xx_hal_rcc.h line 7534 and 277
	SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVQ1EN); //set PLLCFGR DIVQEN
	// stm32h7xx_hal_rcc_ex.h line 3076
        MODIFY_REG(RCC->D2CCIP2R, RCC_D2CCIP2R_USBSEL, (uint32_t)RCC_D2CCIP2R_USBSEL_0);
	// TODO: more configuration on the RCC initialization

	//-----------------------USB IO configuration----------------------------------
	GPIOA->MODER &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10 | GPIO_MODER_MODE11 | GPIO_MODER_MODE12);
	GPIOA->MODER |= (0b00 << GPIO_MODER_MODE9_Pos);
	GPIOA->MODER |= (0b10 << GPIO_MODER_MODE10_Pos);
	GPIOA->MODER |= (0b10 << GPIO_MODER_MODE11_Pos);
	GPIOA->MODER |= (0b10 << GPIO_MODER_MODE12_Pos);
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL10 | GPIO_AFRH_AFSEL11 | GPIO_AFRH_AFSEL12);
	GPIOA->AFR[1] |= (0b1010 << GPIO_AFRH_AFSEL10_Pos); //AF10
	GPIOA->AFR[1] |= (0b1010 << GPIO_AFRH_AFSEL11_Pos); //AF10
	GPIOA->AFR[1] |= (0b1010 << GPIO_AFRH_AFSEL12_Pos); //AF10
	GPIOA->OTYPER &= ~(GPIO_AFRH_AFSEL10);
	GPIOA->OTYPER |= (GPIO_OTYPER_OT10);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED11 | GPIO_OSPEEDR_OSPEED12);
	GPIOA->OSPEEDR |= (0b10 << GPIO_OSPEEDR_OSPEED10_Pos);
	GPIOA->OSPEEDR |= (0b10 << GPIO_OSPEEDR_OSPEED11_Pos);
	GPIOA->OSPEEDR |= (0b10 << GPIO_OSPEEDR_OSPEED12_Pos);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD10);
	GPIOA->PUPDR |= (0b01 << GPIO_PUPDR_PUPD10_Pos); //pull up for pin 10

	// Enable VBUS sense (B device) via pin PA9
	// THE ORDER IS IMPORTANT!!!!!
  	SET_BIT (PWR->CR3, PWR_CR3_USB33DEN); //enable usb voltage detector
	RCC->AHB1ENR |= RCC_AHB1ENR_USB2OTGHSEN; //enable usb otg full speed clock
	USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_VBDEN;

	//disable all interrupts
	//__disable_irq();
	NVIC_EnableIRQ(DMA1_Stream0_IRQn); //enable dma1_stream0 interrupts
	//NVIC_EnableIRQ(DMA1_Stream1_IRQn); //enable dma1_stream1 interrupts
	NVIC_EnableIRQ(TIM4_IRQn); //enable timer 4 interrupts
	//NVIC_EnableIRQ(OTG_FS_IRQn); //enable otg interrupts
	////re-enable all global interrupts
	//__enable_irq();
	//--------------------------------------------------------------------------------
}

// SysTick Interrupt Handler
void SysTick_Handler (void) {
	sys_msticks++;
}

void idle_delay(uint32_t ms){
	uint32_t cur = sys_msticks;
	while(sys_msticks - cur < ms);
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

void DMA1_Stream1_IRQHandler(void){
	if( DMA1->LISR & DMA_LISR_TEIF1 ){
		// transmission err.
		DMA1->LIFCR |= DMA_LIFCR_CTEIF1; // clear transmit complete flag
		GPIOB->BSRR = GPIO_BSRR_BS14; //set bit
	}else{
		// transfer is successful (this implies RX is done)
		DMA1->LIFCR |= DMA_LIFCR_CTCIF1; // clear transmit complete flag
		GPIOB->BSRR = GPIO_BSRR_BS0; //set bit
	}
}

// create a buffer in sram2, buffer is uninitialized and filled with random values
// add the following onto the linker script to allow buf to be initialized on SRAM2
// this is because originally, all vars are on DTCMRAM (domain 1) but DMA has no access
// there. see the following (page 103)
// refer to linker_scripts/stm32h743xx_flash_sram2.ld
// https://community.st.com/s/question/0D50X00009XkeWd/stm32h743-nucleo-dma-transfer-error
// https://community.st.com/s/question/0D50X00009XkXEHSA3/stm32h7stm32h743-adc-with-dma
// https://stackoverflow.com/questions/65577391/stm32-create-ram-section
// https://embedds.com/programming-stm32-discovery-using-gnu-tools-startup-code/
// https://electronics.stackexchange.com/questions/389830/tim2-dma-configuration-for-stm32h7
/*
   .ram2 (NOLOAD) :
   {
   KEEP(*(.sram2))
   } > RAM_D2
   */
char __attribute__ ((section(".sram2"))) dma10_buf[128];

void usart_dma10_printf(USART_TypeDef *usart, const char *msg,...){
	va_list args;
	va_start(args,msg);
	vsprintf(dma10_buf,msg,args);
	size_t mlen = strlen(dma10_buf);

	while (DMA1_Stream0->CR & DMA_SxCR_EN); //ensure EN bit is cleared
	usart->ICR |= USART_ICR_TCCF; //clear the transmission complete flag

	//configure peripheral address
	DMA1_Stream0->M0AR = (uint32_t) dma10_buf;
	DMA1_Stream0->PAR  = (uint32_t) &usart->TDR;
	DMA1_Stream0->NDTR = mlen >= 128 ? 128 : mlen;
	//DMA1->LIFCR |= (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0); //clear all DMA flags (not needed, since the flags SHOULD be cleared in the interrupt anyways)
	DMA1_Stream0->CR |= DMA_SxCR_EN; // start the transfer
	// EN bit is auto-cleared by hardware when transmission is done
	// alternatively we cna clear it in an interrupt
}

// format print a string and its arguments to a USART handler
void usart_printf(USART_TypeDef *usart, const char *msg, ...) {
	char buf[80];
	va_list args;
	va_start(args,msg);
	vsprintf(buf,msg,args);
	for(size_t i=0; i<strlen(buf); i++){
		usart->TDR = buf[i];  //send it back out
		while (!(usart->ISR & USART_ISR_TC)); //wait for TX to be complete
	}
	return;
}

void usart_print32(USART_TypeDef *usart, uint32_t regval){
	usart_printf(usart,"[%08" PRIx32 "]", regval);
	return;
}

//block execution until a character is read from the USART handler
char usart_getc(USART_TypeDef *usart){
	while(!(usart->ISR & USART_ISR_RXNE_RXFNE));
	return usart->RDR;
}

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void)
{

}

