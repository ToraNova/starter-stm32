/*
 * example blinky program with usart.
 * please refer to the reference manual to understand what is going on
 * https://www.st.com/resource/en/reference_manual/dm00314099-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
 *
 * author: toranova (chia_jason96@live.com)
 */

#include "hw.h"
#include "string.h"
#include "httpd.h"
#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "netif/ppp/ppp.h"
#include "netif/ppp/pppos.h"
#include "lwrb.h"

static uint8_t gstate;
static struct netif pppos_netif;
static ppp_pcb *ppp;
static lwrb_t ringbuf;
static uint8_t ringbuf_data[2048];

static const ip_addr_t ppp_thr_ipaddr = IPADDR4_INIT_BYTES(10,0,0,1); //their ip
static const ip_addr_t ppp_our_ipaddr = IPADDR4_INIT_BYTES(10,0,0,2); //our ip

// called when serial has input
void serial_read_callback(const uint8_t *buf, uint16_t len){
	/* Write data to receive buffer */
	lwrb_write(&ringbuf, buf, len);
}

// Callback used by ppp connection
uint32_t ppp_output_callback(ppp_pcb *pcb, const void *data, uint32_t len, void *ctx){
	LWIP_UNUSED_ARG(pcb);
	LWIP_UNUSED_ARG(ctx);
	//idle_delay(20); // if using 9600, this 20ms delay seems necessary!
	idle_delay(5); // required to serve the http packet correctly on 1152000
  	serial_write((uint8_t *)data, len);
	return len;
}

// called when timer expires
void timer_expire_callback(void){
	gpio_write(2, gstate);
	gstate = !gstate; //toggle
}

//callback when link status changed
void ppp_link_status_callback(ppp_pcb *pcb, int err_code, void *ctx){
	struct netif *pppif = ppp_netif(pcb);
	LWIP_UNUSED_ARG(ctx);
	(void) pppif;
	switch(err_code) {
		case PPPERR_NONE:               /* No error. */
			logger_printf("ppp_link_status_cb: PPPERR_NONE.\n");
			//logger_printf("dev ipv4addr = %s.\n", ip4addr_ntoa(netif_ip4_addr(pppif)));
			//logger_printf("rem ipv4addr = %s.\n", ip4addr_ntoa(netif_ip4_gw(pppif)));
			//logger_printf("netmask      = %s.\n", ip4addr_ntoa(netif_ip4_netmask(pppif)));
			break;
		case PPPERR_PARAM:             /* Invalid parameter. */
			logger_printf("ppp_link_status_cb: PPPERR_PARAM.\n");
			break;
		case PPPERR_OPEN:              /* Unable to open PPP session. */
			logger_printf("ppp_link_status_cb: PPPERR_OPEN.\n");
			break;
		case PPPERR_DEVICE:            /* Invalid I/O device for PPP. */
			logger_printf("ppp_link_status_cb: PPPERR_DEVICE.\n");
			break;
		case PPPERR_ALLOC:             /* Unable to allocate resources. */
			logger_printf("ppp_link_status_cb: PPPERR_ALLOC.\n");
			break;
		case PPPERR_USER:              /* User interrupt. */
			logger_printf("ppp_link_status_cb: PPPERR_USER.\n");
			break;
		case PPPERR_CONNECT:           /* Connection lost. */
			logger_printf("ppp_link_status_cb: PPPERR_CONNECT.\n");
			break;
		case PPPERR_AUTHFAIL:          /* Failed authentication challenge. */
			logger_printf("ppp_link_status_cb: PPPERR_AUTHFAIL.\n");
			break;
		case PPPERR_PROTOCOL:          /* Failed to meet protocol. */
			logger_printf("ppp_link_status_cb: PPPERR_PROTOCOL.\n");
			break;
		case PPPERR_PEERDEAD:          /* Connection timeout. */
			logger_printf("ppp_link_status_cb: PPPERR_PEERDEAD.\n");
			break;
		case PPPERR_IDLETIMEOUT:       /* Idle Timeout. */
			logger_printf("ppp_link_status_cb: PPPERR_IDLETIMEOUT.\n");
			break;
		case PPPERR_CONNECTTIME:       /* PPPERR_CONNECTTIME. */
			logger_printf("ppp_link_status_cb: PPPERR_CONNECTTIME.\n");
			break;
		case PPPERR_LOOPBACK:          /* Connection timeout. */
			logger_printf("ppp_link_status_cb: PPPERR_LOOPBACK.\n");
			break;
		default:
			logger_printf("ppp_link_status_cb: unknown errno %d.\n", err_code);
			break;
	}
}

void network_init(void) {
	struct netif *netif;
	err_t err;

	lwip_init();
	// init pppos
	netif = &pppos_netif;
	ppp = pppos_create(netif, ppp_output_callback, ppp_link_status_callback, NULL);
	if (ppp == NULL ){
		logger_printf("pppos creation failed.\n.");
	}

	ppp_set_ipcp_ouraddr(ppp, &ppp_our_ipaddr);
	ppp_set_ipcp_hisaddr(ppp, &ppp_thr_ipaddr);
	//ppp_set_silent(ppp, 1);
	//ppp_set_passive(ppp, 1);
	ppp_set_default(ppp); //set ppp as default route

	err = ppp_connect(ppp,0);
	if (err != ERR_OK) {
		logger_printf("pppd connect failed.\n.");
	}
}

int main(void){

	uint8_t pack[PPP_MAXMRU]; //packet buffer

	hardware_init(); //initialize hardware peripherals
	lwrb_init(&ringbuf, ringbuf_data, sizeof(ringbuf_data));
	gstate = 0;

	network_init();

	httpd_init(); //init http server
	logger_printf("setup ok. elapsed millis: %lu\n", sysclk_millis());

	size_t rblen = 0;
	while(1){
		rblen = lwrb_read(&ringbuf, pack, PPP_MAXMRU);
		if( rblen > 0 ){
			pppos_input(ppp, pack, rblen);
		}
	}
}

/* lwip has provision for using a mutex, when applicable */
sys_prot_t sys_arch_protect(void) {
	return 0;
}
void sys_arch_unprotect(sys_prot_t pval) {
	(void)pval;
}
