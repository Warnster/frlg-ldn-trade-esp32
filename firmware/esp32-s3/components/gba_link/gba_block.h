/* gba_block — block transfer over the packet engine: the INIT_BLOCK + CONT_BLOCK chunking the
 * trade uses to move a LinkPlayer record, a 200-byte party chunk, etc. across 8-word command
 * packets. Port of Celio-Firmware src/callbacks/blockCommand.cpp (sender) + the receive-side
 * reassembly the trade sections do inline (tradeConnection.cpp partnerPartyConstruct).
 *
 * Wire form: one INIT_BLOCK packet { LINKCMD_INIT_BLOCK, declaredSize, 0x80, 0,0,0,0,0 } then
 * ceil(declaredSize/14) CONT_BLOCK packets, each { LINKCMD_CONT_BLOCK, <14 payload bytes> }.
 * Source bytes shorter than declaredSize are zero-padded out to declaredSize.
 *
 * Pure logic, no hardware/RTOS — instance-based (Celio used file globals; a struct is cleaner and
 * host-testable). Plugs into gba_packet: the sender's word fn is the packet tx_cb; after each
 * completed packet, call the chunk fn to stage the next and learn when the transfer is done. */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "gba_trade_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GBA_BLOCK_CHUNK_BYTES 14   /* payload bytes per CONT_BLOCK packet (words[1..7]) */

/* ---- sender ---- */
typedef struct {
    const uint8_t *src;
    uint16_t src_size;      /* real source bytes still to copy (may run out before block_max) */
    uint16_t pos;           /* bytes consumed from src */
    uint16_t block_max;     /* declared block bytes still to send (zero-pads past src_size) */
    int      index;         /* 0..7 word within the current packet */
    bool     init_sent;     /* false until the INIT packet's 8 words are all emitted */
    bool     complete;
    uint16_t init_pkt[8];
    uint16_t content_pkt[8];
} gba_block_tx_t;

/* Begin sending `size` bytes of `src`, declaring `block_max` bytes to the peer. */
void     gba_block_tx_setup(gba_block_tx_t *b, const void *src, uint16_t size, uint16_t block_max);
/* The next word to transmit (use as the gba_packet tx_cb). */
uint16_t gba_block_tx_word(gba_block_tx_t *b);
/* Call after each completed command packet to stage the next chunk; returns true when the whole
 * block has been sent. */
bool     gba_block_tx_chunk(gba_block_tx_t *b);
static inline bool gba_block_tx_done(const gba_block_tx_t *b) { return b->complete; }

/* ---- receiver (reassembler) ---- */
typedef struct {
    uint8_t *dst;
    uint16_t cap;           /* dst capacity */
    uint16_t declared;      /* size from the INIT packet */
    uint16_t got;           /* bytes appended so far */
    bool     in_block;
    bool     complete;
} gba_block_rx_t;

void gba_block_rx_init(gba_block_rx_t *r, void *dst, uint16_t cap);
/* Feed one received 8-word command packet (rx_cmd[0..7]). Returns true when a full declared block
 * has been reassembled into dst (got >= declared). */
bool gba_block_rx_feed(gba_block_rx_t *r, const uint16_t cmd[8]);

#ifdef __cplusplus
}
#endif
