/* ldn_pico.h — the UART link to the Pico (GBA Wireless Adapter impersonator).
 *
 * Pico -> ESP: raw u32-LE WAP packet stream (uart_link::send32). header = (0x9966<<16)|(size<<8)|cmd,
 *              then `size` data words. cmd 0x25 (SendDataAndWait) carries the GBA's outgoing 14B slot.
 * ESP -> Pico: AA55 frames the Pico's relay.rs parses:
 *              AA 55 | type | len | payload[len] | xor_csum.
 *              T_PEER(0x01) = peer_id u32 + 6-word beacon (28B); T_RECV_SLOT(0x02) = slot u32 words;
 *              T_NO_PEER(0x00).
 *
 * Wiring: ESP GPIO(TX) -> Pico GP1 (RX); ESP GPIO(RX) <- Pico GP0 (TX); common GND. 115200 8N1. */
#pragma once
#include <stdint.h>
#include <stdbool.h>

void ldn_pico_init(void);
/* Drain the UART, parse the WAP stream, queue the GBA's outgoing slots. Call frequently. */
void ldn_pico_poll(void);
/* Pop the GBA's next outgoing slot (7 u16 words). Returns 1 if one was available, else 0. */
int  ldn_pico_take_gba_slot(uint16_t words[7]);
/* Send the Switch's received slot down to the GBA (AA55 T_RECV_SLOT). */
void ldn_pico_send_slot(const uint8_t slot14[14]);
/* Advertise a discoverable peer so the GBA connects (AA55 T_PEER). */
void ldn_pico_send_peer(uint32_t peer_id, const uint32_t beacon6[6]);
void ldn_pico_send_no_peer(void);
/* Advertise the default captured peer/beacon so the GBA lists it as a joinable group. */
void ldn_pico_advertise_peer(void);
void ldn_pico_stats(uint32_t out[5]);  /* {wap_seen, slots_seen(0x25), slots_taken, recv_slots_sent, peer_sent} */
/* Number of Connect (0x1f) commands the GBA has issued. Increments when the player selects our
 * advertised peer — the signal to actually join the Switch (not merely that the GBA is active). */
uint32_t ldn_pico_connect_count(void);
/* Count of a given RFU command byte the GBA has issued (diagnostic: what mode/flow is the GBA in?). */
uint32_t ldn_pico_cmd_count(uint8_t cmd);
/* {rx_bytes, peer_commits, peer_present} the Pico relay reports back over the link (diagnostic). */
void ldn_pico_diag(uint32_t out[3]);
/* DIAGNOSTIC: send T_BOOTSEL to reboot the Pico into USB bootloader (RPI-RP2). Confirms ESP->Pico TX. */
void ldn_pico_tx_bootsel_test(void);
