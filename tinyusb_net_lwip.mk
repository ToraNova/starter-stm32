# user CFLAGS
CFLAGS += \
  -DPBUF_POOL_SIZE=2 \
  -DTCP_WND=2*TCP_MSS \
  -DHTTPD_USE_CUSTOM_FSDATA=0 \
  -DCFG_TUSB_MCU=OPT_MCU_STM32H7 \
  -DBOARD_DEVICE_RHPORT_NUM=$(PORT)

SRC_C += \
	libs/tinyusb/src/portable/st/synopsys/dcd_synopsys.c \
	libs/tinyusb/src/tusb.c \
	libs/tinyusb/src/common/tusb_fifo.c \
	libs/tinyusb/src/device/usbd.c \
	libs/tinyusb/src/device/usbd_control.c \
	libs/tinyusb/src/class/audio/audio_device.c \
	libs/tinyusb/src/class/cdc/cdc_device.c \
	libs/tinyusb/src/class/dfu/dfu_rt_device.c \
	libs/tinyusb/src/class/hid/hid_device.c \
	libs/tinyusb/src/class/midi/midi_device.c \
	libs/tinyusb/src/class/msc/msc_device.c \
	libs/tinyusb/src/class/net/net_device.c \
	libs/tinyusb/src/class/usbtmc/usbtmc_device.c \
	libs/tinyusb/src/class/vendor/vendor_device.c \
	libs/lwip/src/core/altcp.c \
	libs/lwip/src/core/altcp_alloc.c \
	libs/lwip/src/core/altcp_tcp.c \
	libs/lwip/src/core/def.c \
	libs/lwip/src/core/dns.c \
	libs/lwip/src/core/inet_chksum.c \
	libs/lwip/src/core/init.c \
	libs/lwip/src/core/ip.c \
	libs/lwip/src/core/mem.c \
	libs/lwip/src/core/memp.c \
	libs/lwip/src/core/netif.c \
	libs/lwip/src/core/pbuf.c \
	libs/lwip/src/core/raw.c \
	libs/lwip/src/core/stats.c \
	libs/lwip/src/core/sys.c \
	libs/lwip/src/core/tcp.c \
	libs/lwip/src/core/tcp_in.c \
	libs/lwip/src/core/tcp_out.c \
	libs/lwip/src/core/timeouts.c \
	libs/lwip/src/core/udp.c \
	libs/lwip/src/core/ipv4/autoip.c \
	libs/lwip/src/core/ipv4/dhcp.c \
	libs/lwip/src/core/ipv4/etharp.c \
	libs/lwip/src/core/ipv4/icmp.c \
	libs/lwip/src/core/ipv4/igmp.c \
	libs/lwip/src/core/ipv4/ip4.c \
	libs/lwip/src/core/ipv4/ip4_addr.c \
	libs/lwip/src/core/ipv4/ip4_frag.c \
	libs/lwip/src/netif/ethernet.c \
	libs/lwip/src/netif/slipif.c \
	libs/lwip/src/apps/http/httpd.c \
	libs/lwip/src/apps/http/fs.c \
	libs/networking/dhserver.c \
	libs/networking/dnserver.c \
	libs/networking/rndis_reports.c

INC += \
	libs/tinyusb/src \
	libs/lwip/src/include \
	libs/lwip/src/include/ipv4 \
	libs/lwip/src/include/lwip/apps \
	libs/networking
