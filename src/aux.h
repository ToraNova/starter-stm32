#ifndef __AUX_H__
#define __AUX_H__

#include <stdint.h>
#define SERIAL_WRITE_MAX_LEN 2048
#define SERIAL_RXBUF_LEN 2048

#ifdef __cplusplus
extern "C" {
#endif

void logger_printf(const char *msg, ...) __attribute__ ((format (printf, 1, 2)));
void logger_print32(uint32_t regval);


extern void tim4_expire_callback(void);

void serial_write(uint8_t *buf, uint16_t len);
extern void serial_read_callback(const uint8_t *buf, uint16_t len);
void serial_read_check(void);

void board_init(void);
uint32_t board_millis(void);
void idle_delay(uint32_t ms);
void board_gpioctl(uint8_t, uint8_t);

uint32_t sysclk_pll_hse_freq(void);
void sysclk_pll_hse_init(void);

#ifdef __cplusplus
}
#endif

#endif
