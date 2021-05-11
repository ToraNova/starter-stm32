/*
 * example usbeth http server
 * please refer to the tinyusb net_lwip example
 */

#include "stm32h743xx.h"
#include "aux.h"
#include "tusb.h"

#include "dhserver.h"
#include "dnserver.h"
#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "httpd.h"
#include "pppd.h"

/* lwip context */
static struct netif usbet_netif;
static struct netif pppos_netif;
static ppp_pcb *ppp;

void serial_read_callback(const uint8_t *buf, uint16_t len){
	logger_printf("pppos_read_ci: received (%u).\n\r", len);
	//logger_printf("sercb: received (%u) [", len);
	//for(uint16_t i=0; i<len; i++){
	//	logger_printf("%02x",buf[i]);
	//}
	//logger_printf("].\r\n");
	pppos_input(ppp, buf, len);
}

void tim4_expire_callback(void){
	// problem - won't reply to ping
	logger_printf("pppd phase %02X.\r\n", ppp->phase);
}

/* shared between tud_network_recv_cb() and service_traffic() */
static struct pbuf *received_frame;

/* this is used by this code, ./class/net/net_driver.c, and usb_descriptors.c */
/* ideally speaking, this should be generated from the hardware's unique ID (if available) */
/* it is suggested that the first byte is 0x02 to indicate a link-local address */
const uint8_t tud_network_mac_address[6] = {0x02,0x02,0x84,0x6A,0x96,0x00};

/* network parameters of this MCU */
static const ip_addr_t ipaddr  = IPADDR4_INIT_BYTES(192, 168, 7, 1);
static const ip_addr_t netmask = IPADDR4_INIT_BYTES(255, 255, 255, 0);
static const ip_addr_t gateway = IPADDR4_INIT_BYTES(0, 0, 0, 0);

static const ip_addr_t pppos_our_ipaddr = IPADDR4_INIT_BYTES(10,0,0,2); //our ip
static const ip_addr_t pppos_thr_ipaddr = IPADDR4_INIT_BYTES(10,0,0,1); //their ip

/* database IP addresses that can be offered to the host; this must be in RAM to store assigned MAC addresses */
static dhcp_entry_t entries[] =
{
	/* mac ip address                          lease time */
	{ {0}, IPADDR4_INIT_BYTES(192, 168, 7, 2), 24 * 60 * 60 },
	{ {0}, IPADDR4_INIT_BYTES(192, 168, 7, 3), 24 * 60 * 60 },
	{ {0}, IPADDR4_INIT_BYTES(192, 168, 7, 4), 24 * 60 * 60 },
};

static const dhcp_config_t dhcp_config =
{
	.router = IPADDR4_INIT_BYTES(0, 0, 0, 0),  /* router address (if any) */
	.port = 67,                                /* listen port */
	.dns = IPADDR4_INIT_BYTES(192, 168, 7, 1), /* dns server (if any) */
	"usb",                                     /* dns suffix */
	TU_ARRAY_SIZE(entries),                    /* num entry */
	entries                                    /* entries */
};
static err_t linkoutput_fn(struct netif *netif, struct pbuf *p)
{
	(void)netif;

	for (;;)
	{
		/* if TinyUSB isn't ready, we must signal back to lwip that there is nothing we can do */
		if (!tud_ready())
			return ERR_USE;

		/* if the network driver can accept another packet, we make it happen */
		if (tud_network_can_xmit())
		{
			tud_network_xmit(p, 0 /* unused for this example */);
			return ERR_OK;
		}

		/* transfer execution to TinyUSB in the hopes that it will finish transmitting the prior packet */
		tud_task();
	}
}

static err_t output_fn(struct netif *netif, struct pbuf *p, const ip_addr_t *addr)
{
	return etharp_output(netif, p, addr);
}

static err_t netif_init_cb(struct netif *netif)
{
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

static void init_lwip(void)
{
	struct netif *netif;
	err_t err;

	/* the lwip virtual MAC address must be different from the host's; to ensure this, we toggle the LSbit */
	netif = &usbet_netif;
	netif->hwaddr_len = sizeof(tud_network_mac_address);
	memcpy(netif->hwaddr, tud_network_mac_address, sizeof(tud_network_mac_address));
	netif->hwaddr[5] ^= 0x01;
	netif = netif_add(netif, &ipaddr, &netmask, &gateway, NULL, netif_init_cb, ip_input);
	//netif_set_default(netif);

	lwip_init();
	// init pppos
	netif = &pppos_netif;
	ppp = pppos_create(netif, ppp_output_cb, ppp_link_status_cb, NULL);
	if (ppp == NULL ){
		logger_printf("pppos creation failed.\r\n.");
	}

	ppp_set_ipcp_ouraddr(ppp, &pppos_our_ipaddr);
	ppp_set_ipcp_hisaddr(ppp, &pppos_thr_ipaddr);
	ppp_set_silent(ppp, 1);
	ppp_set_default(ppp); //set ppp as default route

	err = ppp_connect(ppp,0);
	if (err != ERR_OK) {
		logger_printf("pppd connect failed.\r\n.");
	}

}

/* handle any DNS requests from dns-server */
bool dns_query_proc(const char *name, ip_addr_t *addr)
{
	if (0 == strcmp(name, "tiny.usb"))
	{
		*addr = ipaddr;
		return true;
	}
	return false;
}

bool tud_network_recv_cb(const uint8_t *src, uint16_t size)
{
	/* this shouldn't happen, but if we get another packet before
	   parsing the previous, we must signal our inability to accept it */
	if (received_frame) return false;

	if (size)
	{
		struct pbuf *p = pbuf_alloc(PBUF_RAW, size, PBUF_POOL);

		if (p)
		{
			/* pbuf_alloc() has already initialized struct; all we need to do is copy the data */
			memcpy(p->payload, src, size);

			/* store away the pointer for service_traffic() to later handle */
			received_frame = p;
		}
	}

	return true;
}

uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg)
{
	struct pbuf *p = (struct pbuf *)ref;
	struct pbuf *q;
	uint16_t len = 0;

	(void)arg; /* unused for this example */

	/* traverse the "pbuf chain"; see ./lwip/src/core/pbuf.c for more info */
	for(q = p; q != NULL; q = q->next)
	{
		memcpy(dst, (char *)q->payload, q->len);
		dst += q->len;
		len += q->len;
		if (q->len == q->tot_len) break;
	}

	return len;
}

static void service_traffic(void)
{
	/* handle any packet received by tud_network_recv_cb() */
	if (received_frame)
	{
		ethernet_input(received_frame, &usbet_netif);
		pbuf_free(received_frame);
		received_frame = NULL;
		tud_network_recv_renew();
	}

	sys_check_timeouts();
}

void tud_network_init_cb(void)
{
	/* if the network is re-initializing and we have a leftover packet, we must do a cleanup */
	if (received_frame)
	{
		pbuf_free(received_frame);
		received_frame = NULL;
	}
}

int main(void){

	board_init();
	//blink 10 times quickly
	for(int i=0;i<10;i++){
		board_gpioctl(1, 1); //set bit
		idle_delay(50);
		board_gpioctl(1, 0); //unset bit
		idle_delay(50);
	}
	// observation: when usb is plugged in, DMA fails
	logger_printf("board init and logger ok.\r\n");
	serial_write((uint8_t *)"usart3 ok.\r\n", 12);

	tusb_init();
	logger_printf("tinyusb init ok.\r\n");

	/* initialize lwip, dhcp-server, dns-server, and http */
	init_lwip();
	while (!netif_is_up(&usbet_netif));
	while (dhserv_init(&dhcp_config) != ERR_OK);
	while (dnserv_init(&ipaddr, 53, dns_query_proc) != ERR_OK);
	logger_printf("dhserver init ok.\r\n");

	httpd_init();
	logger_printf("httpd init ok.\r\n");

	while (1)
	{
		tud_task();
		service_traffic();
		if(ppp->phase == PPP_PHASE_DEAD){
			//attempt to reconnect
			logger_printf("pppd attempt to reconnect.\r\n.");
			err = ppp_connect(ppp,0);
			if (err != ERR_OK) {
				logger_printf("pppd reconnect failed.\r\n.");
			}

		}
	}
}

/* lwip has provision for using a mutex, when applicable */
sys_prot_t sys_arch_protect(void)
{
	return 0;
}
void sys_arch_unprotect(sys_prot_t pval)
{
	(void)pval;
}

/* lwip needs a millisecond time source, and the TinyUSB board support code has one available */
uint32_t sys_now(void)
{
	return board_millis();
}

uint32_t sys_jiffies(void)
{
	return board_millis();
}
