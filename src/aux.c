#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "stm32h743xx.h"
#include "aux.h"

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void)
{

}

// format print a string and its arguments to a USART handler
void usart_printf(USART_TypeDef *usart,const char *msg, ...) {
	char buf[80];
	va_list args;
	va_start(args,msg);
	vsprintf(buf,msg,args);
	for(size_t i=0; i<strlen(buf); i++){
		usart->TDR = buf[i];  //send it back out
		while (!(usart->ISR & USART_ISR_TC)); //wait for TX to be complete
	}
}

//block execution until a character is read from the USART handler
char usart_getc(USART_TypeDef *usart){
	while(!(usart->ISR & USART_ISR_RXNE_RXFNE));
	return usart->RDR;
}
