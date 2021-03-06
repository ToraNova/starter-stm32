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
#include "stm32h7xx.h"
#include "stm32h743xx.h"
#include "hw.h"
#include "tusb.h"

// local macro defines

// useful macros
#define SRM_EXPAND(ST, PIN) 	ST ? GPIO_BSRR_BS##PIN : GPIO_BSRR_BR##PIN

#define LOG_USART USART2
#define SER_USART USART3
#define LOGGER_BUFSZ 128

#define DMA10_BUFSZ 2000
#define DMA11_BUFSZ 2000

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
uint8_t __attribute__ ((section(".sram2"))) dma10_buf[DMA10_BUFSZ];
uint8_t __attribute__ ((section(".sram2"))) dma11_buf[DMA11_BUFSZ];

void serial_read_check(void);

void TIM4_IRQHandler(void){
	if( TIM4->SR & TIM_SR_UIF ){
		TIM4->SR &= ~TIM_SR_UIF; //clear the update interrupt flag
		//GPIOE->ODR ^= GPIO_ODR_OD1; // toggle pin 1 on port E
		timer_expire_callback();
		return;
	}
}

void USART3_IRQHandler(void) {
    	/* Check for RX idle interrupt */
	if ( USART3->ISR & USART_ISR_IDLE ){
		USART3->ICR |= USART_ICR_IDLECF; //clear IDLE line flag
		serial_read_check();
		//logger_printf("us3 irq idle detect.\n");
	}else if( USART3->ISR & USART_ISR_ORE ){
		//overrun error (when tx speed overwhelms the mcu receive speed)
		//use FIFO to overcome this
		USART3->ICR |= USART_ICR_ORECF;
		logger_printf("ua3 irq: overrun.\n");
	}
}

void hardware_init(void){
	sysclk_init();

	//enable all the peripheral clocks
	RCC->AHB4ENR |=   RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN
			| RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOEEN | RCC_AHB4ENR_GPIOGEN
 			| RCC_AHB4ENR_GPIOHEN | RCC_AHB4ENR_GPIOIEN | RCC_AHB4ENR_GPIOJEN;

	// set pin 0 and 14 on GPIOB and pin 1 on GPIOE to output mode
  	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE0,  	0b01 << GPIO_MODER_MODE0_Pos);
  	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE14, 	0b01 << GPIO_MODER_MODE14_Pos);
  	MODIFY_REG(GPIOE->MODER, GPIO_MODER_MODE1, 	0b01 << GPIO_MODER_MODE1_Pos);

	if (SysTick_Config (SystemCoreClock / 1000)) { // SysTick 1mSec
		//error, set bit and die
		GPIOB->BSRR = GPIO_BSRR_BS14;
		while(1);
	}

	// usart2 gpio config
	GPIOD->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6); //default 0000
	GPIOD->MODER |= (0b10 << GPIO_MODER_MODE5_Pos);
	GPIOD->MODER |= (0b10 << GPIO_MODER_MODE6_Pos);
	GPIOD->AFR[0] &= ~(GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL6);
	GPIOD->AFR[0] |= (0b0111 << GPIO_AFRL_AFSEL5_Pos);
	GPIOD->AFR[0] |= (0b0111 << GPIO_AFRL_AFSEL6_Pos);

	// usart3 gpio config
	GPIOC->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11); //default 0000
	GPIOC->MODER |= (0b10 << GPIO_MODER_MODE10_Pos);
	GPIOC->MODER |= (0b10 << GPIO_MODER_MODE11_Pos);
	GPIOC->AFR[1] &= ~(GPIO_AFRH_AFSEL10 | GPIO_AFRH_AFSEL11);
	GPIOC->AFR[1] |= (0b0111 << GPIO_AFRH_AFSEL10_Pos);
	GPIOC->AFR[1] |= (0b0111 << GPIO_AFRH_AFSEL11_Pos);

	// usbotg gpio config
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

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; //clock gating for DMA1

	// configure dmamux for respective dma requests (DMA1 for USART3 TX/RX)
	//see page 694 and 695 (Table 121.)
	DMAMUX1_Channel1->CCR |= 45U; //enable channel 1 for usart3_rx_dma
	DMAMUX1_Channel0->CCR |= 46U; //enable channel 0 for usart3_tx_dma

	// using DMA1 Stream0 for TX on uart3
	// dir is set to 01:MEMORY TO PERIPHERAL (cpu to the uart peripheral)
	// TRBUFF is needed for bufferrable transfers, MINC to enable memory address increment
	// TCIE and TEIE are transfer complete and error flags (enabled)
	DMA1_Stream0->CR |= DMA_SxCR_TRBUFF | DMA_SxCR_MINC | (0b01 << DMA_SxCR_DIR_Pos) | DMA_SxCR_TCIE | DMA_SxCR_TEIE;
	DMA1_Stream0->M0AR = (uint32_t) dma10_buf; //memory address
	DMA1_Stream0->PAR  = (uint32_t) &(USART3->TDR); //peripheral address (uart TX register)
	// NDTR is not set for dma_tx because we don't know how long the buffer to transfer is.

	// using DMA1 Stream1 for RX on uart3
	// dir is set to 00:PERIPHERAL TO MEMORY (uart peripheral to a buffer)
	// setup circular mode, transfer complete, error and half transfer interrupt
	// circular mode will loop over the buffer (dma11_buf) when NDTR hits 0
	DMA1_Stream1->CR |= DMA_SxCR_CIRC | DMA_SxCR_TRBUFF | DMA_SxCR_MINC | (0b00 << DMA_SxCR_DIR_Pos) | DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_HTIE;
	DMA1_Stream1->M0AR = (uint32_t) dma11_buf; //memory address
	DMA1_Stream1->PAR  = (uint32_t) &(USART3->RDR); //peripheral address (uart RX register)
	DMA1_Stream1->NDTR = DMA11_BUFSZ;

	RCC->APB1LENR |= RCC_APB1LENR_USART3EN | RCC_APB1LENR_USART2EN; //clock gating for USART

	// usart3 init (with DMA tx and rx)
	USART3->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR; //enable DMA xmit and recv
	//enable transmit, receive, idle detection interrupt and FIFO
	USART3->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_FIFOEN;
	USART3->BRR = (uint32_t) (84000000/1152000); //baud rate of 9600 for 84MHz clock

	// configure TX and RX FIFO for usart3 (7/8 depth)
  	MODIFY_REG(USART3->CR3, USART_CR3_RXFTCFG, 0x00000004U << USART_CR3_RXFTCFG_Pos);
  	MODIFY_REG(USART3->CR3, USART_CR3_TXFTCFG, 0x00000004U << USART_CR3_TXFTCFG_Pos);

	DMA1_Stream1->CR |= DMA_SxCR_EN; // start dma rx
	USART3->CR1 |= USART_CR1_UE; //enable usart3

	USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
	USART2->BRR = (uint32_t) (84000000/9600); // baud rate of 9600 if APB clock is 84MHz

	//------------------------TIMER INTERRUPT-----------------------------------------
	// enable the timer4 clock to be gated by the rcc apb4lenr bus
	RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;
	TIM4->PSC = 64000; //prescaler 64000 (max 65535) on 168MHz
	// 168000000 / 64000 = 2625 (1Hz)
	TIM4->ARR = (uint32_t)(2625*(0.1)); //timer interrupt trigger after 2625 ticks, so 0.1 second int (every 100 ms)
	TIM4->CR1 |= TIM_CR1_URS | TIM_CR1_DIR; //overflow/underflow interrupt, count down
	TIM4->DIER |= TIM_DIER_UIE; // update interrupt ENABLED
	TIM4->CR1 |= TIM_CR1_CEN; //enable the counter

	// USB OTG Clock
	SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVQ1EN); //set PLLCFGR DIVQEN
	// stm32h7xx_hal_rcc_ex.h line 3076
        MODIFY_REG(RCC->D2CCIP2R, RCC_D2CCIP2R_USBSEL, (uint32_t)RCC_D2CCIP2R_USBSEL_0);

	// Enable VBUS sense (B device) via pin PA9
	// THE ORDER IS IMPORTANT!!!!!
  	SET_BIT (PWR->CR3, PWR_CR3_USB33DEN); //enable usb voltage detector
	RCC->AHB1ENR |= RCC_AHB1ENR_USB2OTGHSEN; //enable usb otg full speed clock
	USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_VBDEN;

	//disable all interrupts
	//__disable_irq();
	//NVIC_SetPriority(TIM4_IRQn, 1); //0 is default and highest interrupt priority
	NVIC_EnableIRQ(USART3_IRQn); //enable dma1_stream0 interrupts
	NVIC_EnableIRQ(DMA1_Stream0_IRQn); //enable dma1_stream0 interrupts
	NVIC_EnableIRQ(DMA1_Stream1_IRQn); //enable dma1_stream1 interrupts
	NVIC_EnableIRQ(TIM4_IRQn); //enable timer 4 interrupts
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
	if( len > DMA10_BUFSZ ){
		logger_printf("serial_write: length overflow.\n");
		len = DMA10_BUFSZ;
	}
	memcpy(dma10_buf, buf, len);
	while ((DMA1_Stream0->CR & DMA_SxCR_EN )); //ensure EN bit is cleared
	__disable_irq();
	SER_USART->ICR |= USART_ICR_TCCF; //clear the transmission complete flag
	//DMA1->LIFCR |= (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0); //clear all DMA flags (not needed, since the flags SHOULD be cleared in the interrupt anyways)
	DMA1_Stream0->NDTR = len;
	DMA1_Stream0->CR |= DMA_SxCR_EN; // start the transfer
	__enable_irq();
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

void DMA1_Stream0_IRQHandler(void){
	// this interrupt could be caused by either a transfer complete interrupt
	// OR a transfer error interrupt, it is up to software to check the LISR
	// (or HISR) and decide
	if( DMA1->LISR & DMA_LISR_TEIF0 ){
		// transmission err.
		DMA1->LIFCR |= DMA_LIFCR_CTEIF0; // clear transmit complete flag
		GPIOB->BSRR = GPIO_BSRR_BS14; //set bit
		logger_printf("d10 irq transfer error.\n");
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
		logger_printf("d11 irq transfer error.\n");
	}else if( DMA1->LISR & DMA_LISR_HTIF1 ){
		//half complete interrupt
		DMA1->LIFCR |= DMA_LIFCR_CHTIF1; // clear transmit complete flag
		serial_read_check();
	}else{
		// transfer is successful (this implies RX is done)
		DMA1->LIFCR |= DMA_LIFCR_CTCIF1; // clear transmit complete flag
		serial_read_check();
	}
}

// serial_read_check
// original author: MaJerle
// https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
void serial_read_check(void) {
	static size_t old_pos;
	size_t pos;
	// Calculate current position in buffer
	pos = DMA11_BUFSZ - READ_BIT( DMA1_Stream1->NDTR, DMA_SxNDT);
	if (pos != old_pos) { // Check change in received data
		if (pos > old_pos) { // Current position is over previous one */
			/* We are in "linear" mode */
			/* Process data directly by subtracting "pointers" */
			serial_read_callback(&dma11_buf[old_pos], pos - old_pos);
		} else {
			/* We are in "overflow" mode */
			/* First process data to the end of buffer */
			serial_read_callback(&dma11_buf[old_pos], DMA11_BUFSZ - old_pos);
			/* Check and continue with beginning of buffer */
			if (pos > 0) {
				serial_read_callback(&dma11_buf[0], pos);
			}
		}
		old_pos = pos; // Save current position as old
	}
}

/*
 * block execution until a character is read from the recv buffer
 * not actually used. serves as reference only
uint8_t serial_getc(void){
	while(!(SER_USART->ISR & USART_ISR_RXNE_RXFNE));
	return SER_USART->RDR;
}
*/

void irq_disable(void){
	__disable_irq();
}
void irq_enable(void){
	__enable_irq();
}

// Despite being call USB2_OTG
// OTG_FS is marked as RHPort0 by TinyUSB to be consistent across stm32 port
void OTG_FS_IRQHandler(void)
{
	tud_int_handler(0);
}

// Despite being call USB2_OTG
// OTG_HS is marked as RHPort1 by TinyUSB to be consistent across stm32 port
void OTG_HS_IRQHandler(void)
{
	tud_int_handler(1);
}
