#ifndef __AUX_H__
#define __AUX_H__

#ifdef __cplusplus
extern "C" {
#endif

void usart_printf(USART_TypeDef *usart,const char *msg, ...) __attribute__ ((format (printf, 2, 3)));
char usart_getc(USART_TypeDef *usart);

#ifdef __cplusplus
}
#endif

#endif
