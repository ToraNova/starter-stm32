#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <inttypes.h>
#include "stm32h743xx.h"
#include "aux.h"
#include "halper.h"
#include "tusb.h"

uint32_t volatile sys_msticks;// counter for systicks

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
uint8_t __attribute__ ((section(".sram2"))) dma_tx_buf[SERIAL_WRITE_MAX_LEN];
uint8_t __attribute__ ((section(".sram2"))) dma_rx_buf[SERIAL_RXBUF_LEN];

uint32_t board_millis(void){
	return sys_msticks;
}

void board_init(void){

	sysclk_pll_hse_init();

	//enable all the peripheral clocks
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN;
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOEEN | RCC_AHB4ENR_GPIOGEN;
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOHEN | RCC_AHB4ENR_GPIOIEN | RCC_AHB4ENR_GPIOJEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; //clock gating for DMA1
	RCC->APB1LENR |= RCC_APB1LENR_USART3EN | RCC_APB1LENR_USART2EN; //clock gating for USART
	RCC->APB1LENR |= RCC_APB1LENR_TIM4EN; //APB1 is on 168MHz (clock gating for TIM4)
	RCC->AHB1ENR |= RCC_AHB1ENR_USB2OTGHSEN; //enable usb otg full speed clock

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

	//------------------------USART-------------------------------------------
	// USART 2 gpio config
	GPIOD->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6); //default 0000
	GPIOD->MODER |= (0b10 << GPIO_MODER_MODE5_Pos);
	GPIOD->MODER |= (0b10 << GPIO_MODER_MODE6_Pos);
	GPIOD->AFR[0] &= ~(GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL6);
	GPIOD->AFR[0] |= (0b0111 << GPIO_AFRL_AFSEL5_Pos);
	GPIOD->AFR[0] |= (0b0111 << GPIO_AFRL_AFSEL6_Pos);

	// USART 3 gpio config
	GPIOC->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11); //default 0000
	GPIOC->MODER |= (0b10 << GPIO_MODER_MODE10_Pos);
	GPIOC->MODER |= (0b10 << GPIO_MODER_MODE11_Pos);
	GPIOC->AFR[1] &= ~(GPIO_AFRH_AFSEL10 | GPIO_AFRH_AFSEL11);
	GPIOC->AFR[1] |= (0b0111 << GPIO_AFRH_AFSEL10_Pos);
	GPIOC->AFR[1] |= (0b0111 << GPIO_AFRH_AFSEL11_Pos);

	//------------------------DMA (USART) CONFIGURATION-------------------------------
	//configure a transfer buffer (USART), memory increment (as oppose to peripheral increment PINC)
	// also enable transfer complete interrupt, transfer error interrupt
	// dir 01 -> memory to peripheral (TX)
	DMA1_Stream0->CR |= DMA_SxCR_TRBUFF | DMA_SxCR_MINC | (0b01 << DMA_SxCR_DIR_Pos) | DMA_SxCR_TCIE | DMA_SxCR_TEIE;
	// dir 00 -> peripheral to memory (RX)
	// also uses circular mode
	DMA1_Stream1->CR |= DMA_SxCR_CIRC | DMA_SxCR_TRBUFF | DMA_SxCR_MINC | (0b00 << DMA_SxCR_DIR_Pos) | DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_HTIE;
	// we configure the Stream1 address since it will always be enabled
	DMA1_Stream1->M0AR = (uint32_t) dma_rx_buf;
	DMA1_Stream1->PAR  = (uint32_t) &(USART3->RDR);
	DMA1_Stream1->NDTR = SERIAL_RXBUF_LEN;

	// configure dmamux for respective dma requests
	//see page 694 and 695 (Table 121.)
	DMAMUX1_Channel1->CCR |= 45U; //enable channel 1 for usart3_rx_dma
	DMAMUX1_Channel0->CCR |= 46U; //enable channel 0 for usart3_tx_dma

	// USART init
	USART3->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR; //enable DMA xmit and recv
	USART3->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE;
	USART3->BRR = (uint32_t) (84000000 / 9600);

	//set USART3 FIFO
  	MODIFY_REG(USART3->CR3, USART_CR3_RXFTCFG, 0x00000004U << USART_CR3_RXFTCFG_Pos);
  	MODIFY_REG(USART3->CR3, USART_CR3_TXFTCFG, 0x00000004U << USART_CR3_TXFTCFG_Pos);

	USART2->BRR = (uint32_t) (84000000 / 9600);
	USART2->CR1 |= USART_CR1_TE | USART_CR1_UE; // USART 2 is used for logging (no rx)
	//--------------------------------------------------------------------------------

	//------------------------TIMER INTERRUPT-----------------------------------------
	// enable the timer4 clock to be gated by the rcc apb4lenr bus
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
	USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_VBDEN;

	//disable all interrupts
	//__disable_irq();
	NVIC_EnableIRQ(USART3_IRQn); //enable dma1_stream0 interrupts
	NVIC_EnableIRQ(DMA1_Stream0_IRQn); //enable dma1_stream0 interrupts
	NVIC_EnableIRQ(DMA1_Stream1_IRQn); //enable dma1_stream1 interrupts
	NVIC_EnableIRQ(TIM4_IRQn); //enable timer 4 interrupts
	//NVIC_EnableIRQ(OTG_FS_IRQn); //enable otg interrupts
	////re-enable all global interrupts
	//__enable_irq();
	//--------------------------------------------------------------------------------

	DMA1_Stream1->CR |= DMA_SxCR_EN; // start dma rx
	USART3->CR1 |= USART_CR1_UE; //enable usart3
	GPIOB->BSRR = GPIO_BSRR_BR0; //set bit
}

void TIM4_IRQHandler(void){
	TIM4->SR &= ~TIM_SR_UIF; //clear the update interrupt flag
	GPIOE->ODR ^= GPIO_ODR_OD1; // toggle bit 14
	return;
}

void USART3_IRQHandler(void) {
    	/* Check for IDLE line interrupt */
	if( USART3->ISR & USART_ISR_IDLE ){
		USART3->ICR |= USART_ICR_IDLECF; //clear IDLE line flag
		serial_read_check();
	}
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
	}
}

void DMA1_Stream1_IRQHandler(void){
	if( DMA1->LISR & DMA_LISR_TEIF1 ){
		// transmission err.
		DMA1->LIFCR |= DMA_LIFCR_CTEIF1; // clear transmit complete flag
		GPIOB->BSRR = GPIO_BSRR_BS14; //set bit
	}else if( DMA1->LISR & DMA_LISR_HTIF1 ){
		//half complete interrupt
		DMA1->LIFCR |= DMA_LIFCR_CHTIF1; // clear transmit complete flag
		serial_read_check();
	}else{
		// transfer is successful (this implies RX is done)
		DMA1->LIFCR |= DMA_LIFCR_CTCIF1; // clear transmit complete flag
		serial_read_check();
	}
	logger_printf("d11 irq.\n\r");
}

void serial_write(uint8_t *buf, uint16_t len){
	memcpy(dma_tx_buf, buf, len);
	while (DMA1_Stream0->CR & DMA_SxCR_EN); //ensure EN bit is cleared
	USART3->ICR |= USART_ICR_TCCF; //clear the transmission complete flag
	//DMA1->LIFCR |= (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0); //clear all DMA flags (not needed, since the flags SHOULD be cleared in the interrupt anyways)
	DMA1_Stream0->M0AR = (uint32_t) dma_tx_buf;
	DMA1_Stream0->PAR  = (uint32_t) &USART3->TDR;
	DMA1_Stream0->NDTR = len > SERIAL_WRITE_MAX_LEN ? SERIAL_WRITE_MAX_LEN : len;
	DMA1_Stream0->CR |= DMA_SxCR_EN; // start the transfer
	// EN bit is auto-cleared by hardware when transmission is done
	// alternatively we cna clear it in an interrupt
}

// format print a string and its arguments to a logger usart
void logger_printf(const char *msg, ...) {
	char buf[80];
	va_list args;
	va_start(args,msg);
	vsprintf(buf,msg,args);
	for(size_t i=0; i<strlen(buf); i++){
		USART2->TDR = buf[i];  //send it back out
		while (!(USART2->ISR & USART_ISR_TC)); //wait for TX to be complete
	}
	return;
}

void logger_print32(uint32_t regval){
	logger_printf("[%08" PRIx32 "]", regval);
	return;
}

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void)
{

}

// Despite being call USB2_OTG
// OTG_FS is marked as RHPort0 by TinyUSB to be consistent across stm32 port
void OTG_FS_IRQHandler(void)
{
	tud_int_handler(0);
}

// SysTick Interrupt Handler
void SysTick_Handler (void) {
	sys_msticks++;
}

void idle_delay(uint32_t ms){
	uint32_t cur = sys_msticks;
	while(sys_msticks - cur < ms);
}

void serial_read_check(void) {
	static size_t old_pos;
	size_t pos;

	/* Calculate current position in buffer */
	pos = SERIAL_RXBUF_LEN - READ_BIT( DMA1_Stream1->NDTR, DMA_SxNDT);
	if (pos != old_pos) {                       /* Check change in received data */
		if (pos > old_pos) {                    /* Current position is over previous one */
			/* We are in "linear" mode */
			/* Process data directly by subtracting "pointers" */
			serial_read_callback(&dma_rx_buf[old_pos], pos - old_pos);
		} else {
			/* We are in "overflow" mode */
			/* First process data to the end of buffer */
			serial_read_callback(&dma_rx_buf[old_pos], SERIAL_RXBUF_LEN - old_pos);
			/* Check and continue with beginning of buffer */
			if (pos > 0) {
				serial_read_callback(&dma_rx_buf[0], pos);
			}
		}
		old_pos = pos;                          /* Save current position as old */
	}
}

