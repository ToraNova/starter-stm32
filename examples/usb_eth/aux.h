#ifndef __AUX_H__
#define __AUX_H__

#ifdef __cplusplus
extern "C" {
#endif

void board_init(void);
uint32_t board_millis(void);
void idle_delay(uint32_t ms);
void usart_dma10_printf(USART_TypeDef *usart, const char *msg,...) __attribute__ ((format (printf, 2, 3)));
void usart_printf(USART_TypeDef *usart,const char *msg, ...) __attribute__ ((format (printf, 2, 3)));
void usart_print32(USART_TypeDef *usart, uint32_t regval);
char usart_getc(USART_TypeDef *usart);

uint32_t sysclk_pll_hse_freq(void);
void sysclk_pll_hse_init(void);

#ifdef __cplusplus
}
#endif

#endif
