/* ldn_brain.h — the standalone trade brain: cport/sim.c driven by the ESP's own LDN I/O.
 * Once started (after the LDN join + IPs are up), it runs the Pia handshake, NI registration and
 * RFU slot exchange with the Switch on-chip — no PC in the trade path. The GBA-facing engine is
 * wired in ldn_brain.c (Task 4: the Pico UART RFU relay). */
#pragma once
#include <stdint.h>
#include <stdbool.h>

bool ldn_brain_active(void);
/* ssid(16), our_ip(4), host_ip(4) octets, our_mac(6)=station MAC, host_mac(6)=Switch BSSID.
 * Returns 0 on success. */
int  ldn_brain_start(const uint8_t ssid[16], const uint8_t our_ip[4], const uint8_t host_ip[4],
                     const uint8_t our_mac[6], const uint8_t host_mac[6], const char *joiner_name);
void ldn_brain_stop(void);
/* Feed one inbound LDN datagram (from ldn_udp_poll). src_ip = 4 octets. */
void ldn_brain_on_datagram(const uint8_t src_ip[4], const uint8_t *data, int len);
/* Advance one VBlank; call at ~60 Hz. */
void ldn_brain_tick(void);
