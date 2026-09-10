#pragma once

#include "esp_netif.h"

void ldn_udp_init(esp_netif_t *netif, const uint8_t host[6]);
bool ldn_udp_command(const char *line, bool connected);
void ldn_udp_poll(bool connected);
void ldn_udp_stop(void);
/* Standalone brain hooks: direct datagram send + report the session IPs (octets). */
void ldn_udp_send(const uint8_t dst_ip[4], const uint8_t *data, int len);
bool ldn_udp_ips(uint8_t our_ip[4], uint8_t host_ip[4]);
