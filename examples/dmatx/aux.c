#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <inttypes.h>
#include "stm32h743xx.h"
#include "aux.h"

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void)
{

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
