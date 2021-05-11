#include "aux.h"
#include "pppd.h"

void ppp_link_status_cb(ppp_pcb *pcb, int err_code, void *ctx){
	struct netif *pppif = ppp_netif(pcb);
	LWIP_UNUSED_ARG(ctx);
	switch(err_code) {
		case PPPERR_NONE:               /* No error. */
			logger_printf("ppp_link_status_cb: PPPERR_NONE.\n\r");
			logger_printf("dev_ipv4addr = %s.\n\r",\
					ip4addr_ntoa(netif_ip4_addr(pppif)));
			logger_printf("rem_ipv4addr = %s.\n\r",\
					ip4addr_ntoa(netif_ip4_gw(pppif)));
			logger_printf("netmask      = %s.\n\r",\
					ip4addr_ntoa(netif_ip4_netmask(pppif)));
			break;
		case PPPERR_PARAM:             /* Invalid parameter. */
			logger_printf("ppp_link_status_cb: PPPERR_PARAM.\n\r");
			break;
		case PPPERR_OPEN:              /* Unable to open PPP session. */
			logger_printf("ppp_link_status_cb: PPPERR_OPEN.\n\r");
			break;
		case PPPERR_DEVICE:            /* Invalid I/O device for PPP. */
			logger_printf("ppp_link_status_cb: PPPERR_DEVICE.\n\r");
			break;
		case PPPERR_ALLOC:             /* Unable to allocate resources. */
			logger_printf("ppp_link_status_cb: PPPERR_ALLOC.\n\r");
			break;
		case PPPERR_USER:              /* User interrupt. */
			logger_printf("ppp_link_status_cb: PPPERR_USER.\n\r");
			break;
		case PPPERR_CONNECT:           /* Connection lost. */
			logger_printf("ppp_link_status_cb: PPPERR_CONNECT.\n\r");
			break;
		case PPPERR_AUTHFAIL:          /* Failed authentication challenge. */
			logger_printf("ppp_link_status_cb: PPPERR_AUTHFAIL.\n\r");
			break;
		case PPPERR_PROTOCOL:          /* Failed to meet protocol. */
			logger_printf("ppp_link_status_cb: PPPERR_PROTOCOL.\n\r");
			break;
		case PPPERR_PEERDEAD:          /* Connection timeout. */
			logger_printf("ppp_link_status_cb: PPPERR_PEERDEAD.\n\r");
			break;
		case PPPERR_IDLETIMEOUT:       /* Idle Timeout. */
			logger_printf("ppp_link_status_cb: PPPERR_IDLETIMEOUT.\n\r");
			break;
		case PPPERR_CONNECTTIME:       /* PPPERR_CONNECTTIME. */
			logger_printf("ppp_link_status_cb: PPPERR_CONNECTTIME.\n\r");
			break;
		case PPPERR_LOOPBACK:          /* Connection timeout. */
			logger_printf("ppp_link_status_cb: PPPERR_LOOPBACK.\n\r");
			break;
		default:
			logger_printf("ppp_link_status_cb: unknown errno %d.\n\r", err_code);
			break;
	}
}

// Callback used by ppp connection
u32_t ppp_output_cb(ppp_pcb *pcb, const void *data, u32_t len, void *ctx){
	LWIP_UNUSED_ARG(pcb);
	LWIP_UNUSED_ARG(ctx);
	if(len > SERIAL_WRITE_MAX_LEN){
		logger_printf("pppos_output_cb: overflow warning.\n\r");
	}
	serial_write( (uint8_t *) data, len);
	logger_printf("pppos_output_cb: sent (%ld).\n\r", len);
	return len;
}
