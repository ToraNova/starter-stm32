/*
 * example blinky program with usart.
 * please refer to the reference manual to understand what is going on
 * https://www.st.com/resource/en/reference_manual/dm00314099-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
 *
 * author: toranova (chia_jason96@live.com)
 */

#include "hw.h"
#include "string.h"

static uint8_t gstate;

// called when serial has input
void serial_read_callback(const uint8_t *buf, uint16_t len){
	(void) len;
	logger_printf("serial recv [%02X]\n",buf[0]); // this hardware impl receives one by one
}

// called when timer expires
void timer_expire_callback(void){
	gpio_write(2, gstate);
	gstate = !gstate; //toggle
}

int main(void){

	hardware_init(); //initialize hardware peripherals
	gstate = 0;

	char tmp[] = "serial ok.\n\r";
	serial_write( (uint8_t *) tmp, strlen(tmp) );
	logger_printf("logger ok. elapsed millis: %lu\n", sysclk_millis());

	while(1){
		gpio_write(0, 1);
		idle_delay(500);
		gpio_write(0, 0);
		idle_delay(500);
	}
}
