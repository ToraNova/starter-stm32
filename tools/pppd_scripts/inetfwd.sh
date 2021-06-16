#!/bin/sh

# internet forwarding example
# net0 - interface to receive internet from
# ppp0 - interface to give internet to
sysctl net.ipv4.ip_forward=1

# flush all rules
iptables -X
iptables -F
iptables -t nat -X
iptables -t nat -F

iptables -t nat -A POSTROUTING -o net0 -j MASQUERADE
iptables -A FORWARD -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
iptables -A FORWARD -i ppp0 -o net0 -j ACCEPT
iptables -t mangle -A FORWARD -o ppp0 -p tcp -m tcp --tcp-flags SYN,RST SYN -j TCPMSS --clamp-mss-to-pmtu
