/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Simon Goldschmidt
 *
 */
#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#define NO_SYS_NO_TIMERS		0
#define SYS_LIGHTWEIGHT_PROT		0
#define NO_SYS                          1

#define MEM_LIBC_MALLOC			1
#define MEMP_MEM_MALLOC			1
#define MEM_ALIGNMENT			4

#define MEM_SIZE                        (16*1024*1024)
#define MEMP_NUM_UDP_PCB		1024
#define MEMP_NUM_TCP_PCB		1024
#define MEMP_NUM_TCP_PCB_LISTEN		1024
#define MEMP_NUM_TCP_SEG		8192
#define MEMP_NUM_REASSDATA		256
#define MEMP_NUM_FRAG_PBUF		1024
#define MEMP_NUM_TCPIP_MSG_API		1024
#define MEMP_NUM_TCPIP_MSG_INPKT	1024

#define LWIP_RAW                        0
#define LWIP_NETCONN                    0
#define LWIP_SOCKET                     0
#define LWIP_DHCP                       0
#define LWIP_ICMP                       1
#define LWIP_UDP                        1
#define LWIP_TCP                        1
#define ETH_PAD_SIZE                    0
#define LWIP_IP_ACCEPT_UDP_PORT(p)      ((p) == PP_NTOHS(67))

#define TCP_MSS                         (1500 /*mtu*/ - 20 /*iphdr*/ - 20 /*tcphhr*/)
#define TCP_SND_BUF                     (2 * TCP_MSS)

#define ETHARP_SUPPORT_STATIC_ENTRIES   1

#define LWIP_HTTPD_CGI                  0
#define LWIP_HTTPD_SSI                  0
#define LWIP_HTTPD_SSI_INCLUDE_TAG      0

//#define LWIP_SINGLE_NETIF             1
#define IP_FORWARD                      1

// using NAT support by Russ Dill
struct pbuf;
struct netif;
extern int ip4_input_nat(struct pbuf *p, struct netif *inp);
#define LWIP_HOOK_IP4_INPUT		ip4_input_nat
#define LWIP_NAT 			1
#define LWIP_NAT_ICMP 			1
#define LWIP_NAT_ICMP_IP		1

#define PPP_SUPPORT 			1
#define PPPOS_SUPPORT 			1
//#define PPP_INPROC_IRQ_SAFE 		1
#define VJ_SUPPORT 			1
#define CCP_SUPPORT 			1
#define MPPE_SUPPORT 			1
#define LCP_ECHOINTERVAL 		10
#define LCP_MAXECHOFAILS 		3

#define LWIP_DEBUG                      1
#define PPP_DEBUG 			LWIP_DBG_ON //required to keep ppp running (FIX ME!)
//#define IP_DEBUG 			LWIP_DBG_ON
#define NAT_DEBUG			LWIP_DBG_ON
//#define TCP_DEBUG 			LWIP_DBG_ON
//#define NETIF_DEBUG 			LWIP_DBG_ON
//#define ICMP_DEBUG 			LWIP_DBG_ON

#define LWIP_DBG_MIN_LEVEL              LWIP_DBG_LEVEL_ALL
//#define LWIP_DBG_MIN_LEVEL              LWIP_DBG_LEVEL_WARNING

#endif /* __LWIPOPTS_H__ */
