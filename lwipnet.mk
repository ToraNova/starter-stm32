# user CFLAGS
CFLAGS += \
  -DPBUF_POOL_SIZE=2 \
  -DTCP_WND=2*TCP_MSS \
  -DHTTPD_USE_CUSTOM_FSDATA=0 \

SRC_C += \
	lib/lwip/src/api/err.c \
	lib/lwip/src/core/altcp.c \
	lib/lwip/src/core/altcp_alloc.c \
	lib/lwip/src/core/altcp_tcp.c \
	lib/lwip/src/core/def.c \
	lib/lwip/src/core/dns.c \
	lib/lwip/src/core/inet_chksum.c \
	lib/lwip/src/core/init.c \
	lib/lwip/src/core/ip.c \
	lib/lwip/src/core/mem.c \
	lib/lwip/src/core/memp.c \
	lib/lwip/src/core/netif.c \
	lib/lwip/src/core/pbuf.c \
	lib/lwip/src/core/raw.c \
	lib/lwip/src/core/stats.c \
	lib/lwip/src/core/sys.c \
	lib/lwip/src/core/tcp.c \
	lib/lwip/src/core/tcp_in.c \
	lib/lwip/src/core/tcp_out.c \
	lib/lwip/src/core/timeouts.c \
	lib/lwip/src/core/udp.c \
	lib/lwip/src/core/ipv4/autoip.c \
	lib/lwip/src/core/ipv4/dhcp.c \
	lib/lwip/src/core/ipv4/etharp.c \
	lib/lwip/src/core/ipv4/icmp.c \
	lib/lwip/src/core/ipv4/igmp.c \
	lib/lwip/src/core/ipv4/ip4.c \
	lib/lwip/src/core/ipv4/ip4_addr.c \
	lib/lwip/src/core/ipv4/ip4_frag.c \
	lib/lwip/src/core/ipv4/acd.c \
	lib/lwip/src/netif/ethernet.c \
	lib/lwip/src/netif/bridgeif.c \
	lib/lwip/src/netif/slipif.c \
	lib/lwip/src/netif/ppp/ppp.c \
	lib/lwip/src/netif/ppp/pppos.c \
	lib/lwip/src/netif/ppp/magic.c \
	lib/lwip/src/netif/ppp/lcp.c \
	lib/lwip/src/netif/ppp/eap.c \
	lib/lwip/src/netif/ppp/auth.c \
	lib/lwip/src/netif/ppp/ipcp.c \
	lib/lwip/src/netif/ppp/fsm.c \
	lib/lwip/src/netif/ppp/ccp.c \
	lib/lwip/src/netif/ppp/vj.c \
	lib/lwip/src/netif/ppp/mppe.c \
	lib/lwip/src/netif/ppp/utils.c \
	lib/lwip/src/netif/ppp/chap-new.c \
	lib/lwip/src/netif/ppp/chap-md5.c \
	lib/lwip/src/netif/ppp/chap_ms.c \
	lib/lwip/src/netif/ppp/polarssl/sha1.c \
	lib/lwip/src/netif/ppp/polarssl/arc4.c \
	lib/lwip/src/netif/ppp/polarssl/des.c \
	lib/lwip/src/netif/ppp/polarssl/md4.c \
	lib/lwip/src/netif/ppp/polarssl/md5.c \
	lib/lwip/src/netif/ppp/pppcrypt.c \
	lib/lwip/src/apps/http/httpd.c \
	lib/lwip/src/apps/http/fs.c \
	lib/networking/dhserver.c \
	lib/networking/dnserver.c \

INC += \
	lib/lwip/src/include \
	lib/lwip/src/include/ipv4 \
	lib/lwip/src/include/lwip/apps \
	lib/networking
