/*
 * hardware independent auxiliary code to help the dev process
 *
 * author: toranova (chia_jason96@live.com)
 */

#include "hw.h"

uint32_t volatile systicks_ms;// counter for systicks

// SysTick Interrupt Handler
void SysTick_Handler (void) {
	systicks_ms++;
}

uint32_t sysclk_millis(void){
	return systicks_ms;
}

void idle_delay(uint32_t ms){
	uint32_t cur = systicks_ms;
	while(systicks_ms - cur < ms);
}

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void) {
}

__attribute__((weak)) void sysclk_init(void){

}
