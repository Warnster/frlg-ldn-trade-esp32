/* gba_packet — the Gen 3 link packet engine: the handshake -> CRC -> 8-word-command framing that
 * rides on top of the raw 16-bit link transfers (gba_link.h) and carries the LINKCMD/block traffic
 * the trade sections speak.
 *
 * SYNCHRONOUS PORT of Celio-Firmware's src/layers/packetLayer.{hpp,cpp}. Celio's PacketLayer is
 * interrupt-driven (PIO ISR -> onReceive/onTransmit/onTransiveDone) and hands finished command
 * packets to a thread through k_sem. Our core1 is a bare polling loop, so this is restructured as a
 * plain state machine with no RTOS primitives: the driver calls gba_packet_next_tx() to get the
 * word to send, does one gba_link_transceive16(), then calls gba_packet_on_transceive() to advance
 * the state — and checks the return for "a full 8-word command packet just completed". Identical
 * protocol and CRC to Celio; only the control flow is inverted (poll instead of ISR+semaphore).
 *
 * FRAMING (from packetLayer.cpp onTransiveDone):
 *   - state HANDSHAKE: exchange handshake words until either side sends LINK_MASTER_HANDSHAKE, then
 *     advance to CRC. (The handshake word we send is chosen by the handshake sub-state:
 *     disabled -> LINK_HANDSHAKE_DISABLE, enabled -> LINK_SLAVE_HANDSHAKE, connect -> LINK_MASTER_HANDSHAKE.)
 *   - state CRC: exchange one word (we transmit the running CRC of the previous command packet),
 *     then advance to COMMAND and reset CRC to 0.
 *   - state COMMAND: exchange exactly 8 words; each adds (tx+rx) to the CRC; after the 8th, a
 *     command packet is COMPLETE (rx_cmd[0..7] holds it) and we return to CRC.
 *   So the wire is: <handshake...> then repeating { <crc word> <8 command words> }.
 *
 * CRC is a plain 16-bit sum of every command word sent and received (matches Celio). */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "gba_trade_proto.h"   /* LINK_*_HANDSHAKE */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { GBA_PKT_HANDSHAKE = 0, GBA_PKT_CRC, GBA_PKT_COMMAND } gba_pkt_state_t;

/* Handshake sub-state = which handshake word we present (Celio HandShakeState). */
typedef enum {
    GBA_HS_DISABLED = 0,   /* -> LINK_HANDSHAKE_DISABLE */
    GBA_HS_ENABLED,        /* -> LINK_SLAVE_HANDSHAKE  (we are a ready child) */
    GBA_HS_CONNECT,        /* -> LINK_MASTER_HANDSHAKE (drive the link into command phase) */
} gba_hs_state_t;

/* Inter-word timing the link layer should hold after each transfer (Celio's PacketLayer constants,
 * in its timingUs units). */
#define GBA_PKT_TIMING_HANDSHAKE         30097u
#define GBA_PKT_TIMING_COMMAND_BYTES      1378u
#define GBA_PKT_TIMING_BETWEEN_COMMANDS  12953u

/* The caller supplies the next COMMAND word to transmit (Celio TransiveStruct.transive). Returning
 * LINKCMD_EMPTY/0 is the idle filler. ctx is the trade-section state. */
typedef uint16_t (*gba_pkt_tx_cb)(void *ctx);

typedef struct {
    gba_pkt_state_t state;
    gba_hs_state_t  hs_state;

    uint16_t crc;                 /* running sum; Celio seeds the first with LINK_SLAVE_HANDSHAKE */
    int      cmd_index;           /* 0..8 received command words this packet */

    uint16_t rx_cmd[8];           /* the last fully-received command packet */
    uint16_t tx_cmd[8];           /* what we sent alongside it */

    uint16_t last_rx_handshake;
    uint16_t last_tx_handshake;

    uint32_t timing_us;           /* inter-word gap for the NEXT transfer (feed to gba_link) */
    bool     idle;                /* handler signalled it has nothing left to send */

    gba_pkt_tx_cb tx_cb;
    void         *tx_ctx;
} gba_packet_t;

void gba_packet_init(gba_packet_t *p);

/* Handshake progression (Celio enableHandshake/connectHandshake). */
static inline void gba_packet_enable_handshake(gba_packet_t *p)  { p->hs_state = GBA_HS_ENABLED; }
static inline void gba_packet_connect_handshake(gba_packet_t *p) { p->hs_state = GBA_HS_CONNECT; }

/* Install the command-word source (the current trade section's transive handler). */
void gba_packet_set_tx(gba_packet_t *p, gba_pkt_tx_cb cb, void *ctx);

/* The word to present on the NEXT transfer, given the current state. Has the side effect of pulling
 * one word from tx_cb in the COMMAND state (so call exactly once per transfer, before transceive). */
uint16_t gba_packet_next_tx(gba_packet_t *p);

/* Feed the (rx, tx) pair of one completed transfer; advances the state machine.
 * Returns true iff an 8-word command packet just completed — rx_cmd[0..7] is then valid. */
bool gba_packet_on_transceive(gba_packet_t *p, uint16_t rx, uint16_t tx);

#ifdef __cplusplus
}
#endif
