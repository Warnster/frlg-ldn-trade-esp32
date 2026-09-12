/* gba_block.c — see gba_block.h. Port of Celio blockCommand.cpp (+ inline reassembly). Pure logic. */
#include "gba_block.h"
#include <string.h>

/* ---- sender (Celio blockCommandSetup / blockCommandTransive / blockCommandChunk) ------------- */

void gba_block_tx_setup(gba_block_tx_t *b, const void *src, uint16_t size, uint16_t block_max)
{
    memset(b, 0, sizeof(*b));
    b->src       = (const uint8_t *)src;
    b->src_size  = size;
    b->block_max = block_max;
    /* INIT packet: { INIT_BLOCK, declaredSize, 0x80, 0,0,0,0,0 } */
    b->init_pkt[0] = LINKCMD_INIT_BLOCK;
    b->init_pkt[1] = block_max;
    b->init_pkt[2] = 0x80;
    b->content_pkt[0] = LINKCMD_CONT_BLOCK;
}

uint16_t gba_block_tx_word(gba_block_tx_t *b)
{
    uint16_t ret = b->init_sent ? b->content_pkt[b->index] : b->init_pkt[b->index];
    b->index++;
    if (b->index == 8) {
        b->index = 0;
        b->init_sent = true;       /* after the 8 INIT words, all later packets are CONT */
    }
    return ret;
}

bool gba_block_tx_chunk(gba_block_tx_t *b)
{
    if (b->block_max == 0) { b->complete = true; return true; }
    if (!b->init_sent) return false;       /* INIT packet not fully emitted yet */

    /* stage the next 14-byte CONT payload (words[1..7]); zero-pad past the real source */
    memset((uint8_t *)b->content_pkt + 2, 0x00, GBA_BLOCK_CHUNK_BYTES);
    uint16_t chunk     = b->src_size  < GBA_BLOCK_CHUNK_BYTES ? b->src_size  : GBA_BLOCK_CHUNK_BYTES;
    uint16_t max_chunk = b->block_max < GBA_BLOCK_CHUNK_BYTES ? b->block_max : GBA_BLOCK_CHUNK_BYTES;
    if (chunk > 0)
        memcpy((uint8_t *)b->content_pkt + 2, b->src + b->pos, chunk);
    b->src_size  -= chunk;
    b->block_max -= max_chunk;
    b->pos       += chunk;
    return false;
}

/* ---- receiver / reassembler (Celio tradeConnection partnerPartyConstruct, generalised) ------- */

void gba_block_rx_init(gba_block_rx_t *r, void *dst, uint16_t cap)
{
    memset(r, 0, sizeof(*r));
    r->dst = (uint8_t *)dst;
    r->cap = cap;
}

bool gba_block_rx_feed(gba_block_rx_t *r, const uint16_t cmd[8])
{
    if (cmd[0] == LINKCMD_INIT_BLOCK) {
        r->declared = cmd[1];
        r->got      = 0;
        r->in_block = true;
        r->complete = false;
        return false;
    }
    if (cmd[0] == LINKCMD_CONT_BLOCK && r->in_block) {
        /* 14 payload bytes live in words[1..7] (byte offset 2 of the packet). */
        const uint8_t *payload = (const uint8_t *)cmd + 2;
        for (int i = 0; i < GBA_BLOCK_CHUNK_BYTES && r->got < r->declared && r->got < r->cap; i++)
            r->dst[r->got++] = payload[i];
        if (r->got >= r->declared) {
            r->in_block = false;
            r->complete = true;
            return true;
        }
    }
    return false;
}
