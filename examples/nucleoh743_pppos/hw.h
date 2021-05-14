/*
 * hardware implementation headers - separating the hardware implementation
 * and the application (main.c) should not contain hardware dependent code!
 * this way, application can be easily developed on multiple boards, and
 * the devtask can be split into hardware (writing the hw.c file)
 * and application (writing the main.c file)
 *
 * this file should provide the absolute utility function required to use a mcu
 * basic necessities are logging, usart_write/read callbacks, gpio control and a timer callback function and a system time function
 * EACH OF THIS FUNCTION WILL DIFFER BASED ON THEIR ACTUAL HARDWARE IMPLEMENTATION!
 *
 * an initialization function should also be available for app dev to call (hardware_init)
 */

#ifndef __HW_H__
#define __HW_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void hardware_init(void);
uint32_t sysclk_millis(void); // get system elapsed time in millis since boot
void idle_delay(uint32_t ms); // do nothing and delay

void serial_write(uint8_t *buf, uint16_t len); // write to serial out
extern void serial_read_callback(const uint8_t *buf, uint16_t len); // called when serial has input

extern void timer_expire_callback(void); // called when timer expires

// control gpio (set/reset)
void gpio_write(uint8_t arg, uint8_t state);

// logger print functions (for debugging/ monitoring)
void logger_printf(const char *msg, ...) __attribute__ ((format (printf, 1, 2)));
void logger_print32(uint32_t regval);

#ifdef __cplusplus
}
#endif

#endif
