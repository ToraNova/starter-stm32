/*
 * A PPPOS 2 USBRNDIS example using crude network address translation
 *
 * author: toranova (chia_jason96@live.com)
 */
#include "hw.h"
#include "tusb.h"

#include "dhserver.h"
#include "dnserver.h"
#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "lwrb.h"
#include "netif/ppp/ppp.h"
#include "netif/ppp/pppos.h"
#include "nat/nat.h"
#include "httpd.h"

/* lwip context */
static struct netif usbet_netif;
static struct netif pppos_netif;
static ppp_pcb *ppp;
static struct nat_rule usb2ppp_rule; //nat

/* shared between tud_network_recv_cb() and service_traffic() */
static struct pbuf *received_frame;
// buffers
static uint8_t gstate;
static uint8_t ringbuf_data[6000];
static uint8_t pack[PPP_MAXMRU]; //packet buffer
static lwrb_t ringbuf;
static uint16_t gtc;

/* this is used by this code, ./class/net/net_driver.c, and usb_descriptors.c */
/* ideally speaking, this should be generated from the hardware's unique ID (if available) */
/* it is suggested that the first byte is 0x02 to indicate a link-local address */
const uint8_t tud_network_mac_address[6] = {0x02,0x02,0x84,0x6A,0x96,0x00};

/* network parameters of this MCU */
static const ip_addr_t ipaddr  = IPADDR4_INIT_BYTES(192, 168, 7, 1);
static const ip_addr_t netmask = IPADDR4_INIT_BYTES(255, 255, 255, 0);
static const ip_addr_t gateway = IPADDR4_INIT_BYTES(0, 0, 0, 0);

// ppp network param
static const ip_addr_t ppp_thr_ipaddr = IPADDR4_INIT_BYTES(10,0,0,1); //their ip
static const ip_addr_t ppp_our_ipaddr = IPADDR4_INIT_BYTES(10,0,0,2); //our ip

/* database IP addresses that can be offered to the host; this must be in RAM to store assigned MAC addresses */
static dhcp_entry_t entries[] = {
	/* mac ip address                          lease time */
	{ {0}, IPADDR4_INIT_BYTES(192, 168, 7, 2), 24 * 60 * 60 },
	{ {0}, IPADDR4_INIT_BYTES(192, 168, 7, 3), 24 * 60 * 60 },
	{ {0}, IPADDR4_INIT_BYTES(192, 168, 7, 4), 24 * 60 * 60 },
};

static const dhcp_config_t dhcp_config = {
	.router = IPADDR4_INIT_BYTES(0, 0, 0, 0),  /* router address (if any) */
	.port = 67,                                /* listen port */
	.dns = IPADDR4_INIT_BYTES(192, 168, 7, 1), /* dns server (if any) */
	"usb",                                     /* dns suffix */
	TU_ARRAY_SIZE(entries),                    /* num entry */
	entries                                    /* entries */
};

// called when serial has input
void serial_read_callback(const uint8_t *buf, uint16_t len){
	/* Write data to receive buffer */
	//logger_printf("pppos_in: ");
	//for(uint32_t i=0;i<len;i++){
	//	logger_printf("%02X", buf[i]);
	//}
	//logger_printf("\n");
	lwrb_write(&ringbuf, buf, len);
}

// Callback used by ppp connection
uint32_t ppp_output_callback(ppp_pcb *pcb, const void *data, uint32_t len, void *ctx){
	LWIP_UNUSED_ARG(pcb);
	LWIP_UNUSED_ARG(ctx);
	//idle_delay(20); // if using 9600, this 20ms delay seems necessary!
	//idle_delay(50); // required to serve the http packet correctly on 1152000
	//logger_printf("pppos_out: ");
	//for(uint32_t i=0;i<len;i++){
	//	logger_printf("%02X", ((uint8_t *)data)[i]);
	//}
	//logger_printf("\n");
  	serial_write((uint8_t *)data, len);
	return len;
}

// called when timer expires
void timer_expire_callback(void){
	gtc++;
	if(gtc >= (uint16_t)(LWIP_NAT_TICK_PERIOD_MS / 100) ){
		gpio_write(2, gstate);
		gstate = !gstate; //toggle
		nat_timer_tick(); //period of this should be 1ms LWIP_NAT_TICK_PERIOD_MS
		gtc = 0;
	}
}

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


static err_t linkoutput_fn(struct netif *netif, struct pbuf *p) {
	(void)netif; //unused
	while(1) {
		if (!tud_ready()) return ERR_USE;

		if (tud_network_can_xmit()) {
			tud_network_xmit(p, 0 /* unused for this example */);
			return ERR_OK;
		}

		tud_task(); // handle task in while looping
	}
}

static err_t output_fn(struct netif *netif, struct pbuf *p, const ip_addr_t *addr) {
	return etharp_output(netif, p, addr);
}

static err_t netif_init_cb(struct netif *netif) {
	LWIP_ASSERT("netif != NULL", (netif != NULL));
	netif->mtu = CFG_TUD_NET_MTU;
	netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP | NETIF_FLAG_UP;
	netif->state = NULL;
	netif->name[0] = 'E';
	netif->name[1] = 'X';
	netif->linkoutput = linkoutput_fn;
	netif->output = output_fn;
	return ERR_OK;
}

static void network_init(void) {
	struct netif *netif = &usbet_netif;
	err_t err;

	lwip_init();

	/* the lwip virtual MAC address must be different from the host's; to ensure this, we toggle the LSbit */
	netif->hwaddr_len = sizeof(tud_network_mac_address);
	memcpy(netif->hwaddr, tud_network_mac_address, sizeof(tud_network_mac_address));
	netif->hwaddr[5] ^= 0x01;

	netif = netif_add(netif, &ipaddr, &netmask, &gateway, NULL, netif_init_cb, ip_input);
	//netif_set_default(netif);

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

	// NAT setup
	struct nat_rule *rule = &usb2ppp_rule;
	rule->inp = &usbet_netif;
	rule->outp = &pppos_netif;
	err = nat_rule_add(rule);
	if (err != ERR_OK){
		logger_printf("nat setup failed.\n");
	}
}

/* handle any DNS requests from dns-server */
bool dns_query_proc(const char *name, ip_addr_t *addr) {
	if (0 == strcmp(name, "tiny.usb"))
	{
		*addr = ipaddr;
		return true;
	}
	return false;
}

bool tud_network_recv_cb(const uint8_t *src, uint16_t size) {
	/* this shouldn't happen, but if we get another packet before
	   parsing the previous, we must signal our inability to accept it */
	if (received_frame) return false;

	if (size) {
		struct pbuf *p = pbuf_alloc(PBUF_RAW, size, PBUF_POOL);

		if (p) {
			/* pbuf_alloc() has already initialized struct; all we need to do is copy the data */
			memcpy(p->payload, src, size);

			/* store away the pointer for service_traffic() to later handle */
			received_frame = p;
		}
	}

	return true;
}

uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg) {
	struct pbuf *p = (struct pbuf *)ref;
	struct pbuf *q;
	uint16_t len = 0;

	(void)arg; /* unused for this example */

	/* traverse the "pbuf chain"; see ./lwip/src/core/pbuf.c for more info */
	for(q = p; q != NULL; q = q->next) {
		memcpy(dst, (char *)q->payload, q->len);
		dst += q->len;
		len += q->len;
		if (q->len == q->tot_len) break;
	}

	return len;
}

static void service_traffic(void) {
	size_t rblen = 0;

	/* handle any packet received by tud_network_recv_cb() */
	if (received_frame) {
		ethernet_input(received_frame, &usbet_netif);
		//pbuf_free(received_frame);
		received_frame = NULL;
		tud_network_recv_renew();
	}

	// pppos_input called in main thread if PPP_INPROC_IRQ_SAFE == 1
	// see https://www.nongnu.org/lwip/2_0_x/group__ppp.html
	rblen = lwrb_read(&ringbuf, pack, PPP_MAXMRU);
	if( rblen > 0){
		if(rblen > PPP_MAXMRU){
			logger_printf("ppp_input: length overflow.\n");
		}
		pppos_input(ppp, pack, rblen);
	}
}

void tud_network_init_cb(void)
{
	/* if the network is re-initializing and we have a leftover packet, we must do a cleanup */
	if (received_frame) {
		pbuf_free(received_frame);
		received_frame = NULL;
	}
}

int main(void) {
	//initialize hardware peripherals
	gtc = 0;
	hardware_init();
	lwrb_init(&ringbuf, ringbuf_data, sizeof(ringbuf_data));
	gstate = 0;

	/* initialize TinyUSB */
	tusb_init();

	//uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  	//serial_write(data, 8);
	//while(1);

	/* initialize lwip, dhcp-server, dns-server, and http */
	network_init();
	while (!netif_is_up(&usbet_netif));
	while (!netif_is_up(&pppos_netif));
	//while (dhserv_init(&dhcp_config) != ERR_OK);
	//while (dnserv_init(&ipaddr, 53, dns_query_proc) != ERR_OK);
	httpd_init();

	logger_printf("setup ok. elapsed millis: %lu\n", sysclk_millis());
	//logger_printf("pbuf_link_hlen: %lu\n", PBUF_LINK_HLEN);

	while (1) {
		sys_check_timeouts();
		tud_task();
		service_traffic();
	}

	return 0;
}

/* lwip has provision for using a mutex, when applicable */
sys_prot_t sys_arch_protect(void) {
	return 0;
}
void sys_arch_unprotect(sys_prot_t pval) {
	(void)pval;
}

