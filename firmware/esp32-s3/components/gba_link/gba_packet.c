/* gba_packet.c — see gba_packet.h. Synchronous port of Celio PacketLayer (packetLayer.{hpp,cpp}).
 * Pure logic, no hardware and no RTOS — links into the firmware AND compiles/host-tests standalone. */
#include "gba_packet.h"
#include <stddef.h>   /* NULL */

void gba_packet_init(gba_packet_t *p)
{
    for (unsigned i = 0; i < sizeof(*p); i++) ((uint8_t *)p)[i] = 0;
    p->state     = GBA_PKT_HANDSHAKE;
    p->hs_state  = GBA_HS_DISABLED;
    p->crc       = LINK_SLAVE_HANDSHAKE;   /* Celio: "first crc is always handshake" */
    p->timing_us = GBA_PKT_TIMING_HANDSHAKE;
    p->idle      = true;
    p->last_rx_handshake = LINK_HANDSHAKE_DISABLE;
    p->last_tx_handshake = LINK_HANDSHAKE_DISABLE;
}

void gba_packet_set_tx(gba_packet_t *p, gba_pkt_tx_cb cb, void *ctx)
{
    p->tx_cb  = cb;
    p->tx_ctx = ctx;
    p->idle   = false;
}

/* Celio transmitHandshake(). */
static uint16_t handshake_word(gba_hs_state_t s)
{
    switch (s) {
        case GBA_HS_DISABLED: return LINK_HANDSHAKE_DISABLE;
        case GBA_HS_ENABLED:  return LINK_SLAVE_HANDSHAKE;
        case GBA_HS_CONNECT:  return LINK_MASTER_HANDSHAKE;
    }
    return 0xDEAD;
}

/* Celio onTransmit(): pick the word to send for the current state. In COMMAND it pulls one word
 * from the handler; CRC accumulation is done in on_transceive (which has both tx and rx). */
uint16_t gba_packet_next_tx(gba_packet_t *p)
{
    switch (p->state) {
        case GBA_PKT_HANDSHAKE:
            return handshake_word(p->hs_state);
        case GBA_PKT_CRC:
            return p->crc;
        case GBA_PKT_COMMAND: {
            uint16_t tx = (p->tx_cb != NULL) ? p->tx_cb(p->tx_ctx) : 0x0000;
            return tx;
        }
    }
    return 0x0000;
}

bool gba_packet_on_transceive(gba_packet_t *p, uint16_t rx, uint16_t tx)
{
    bool packet_complete = false;

    switch (p->state) {
        case GBA_PKT_HANDSHAKE:
            /* onReceive + onTransiveDone(handshake) */
            p->last_rx_handshake = rx;
            p->last_tx_handshake = tx;
            p->timing_us = GBA_PKT_TIMING_HANDSHAKE;
            if (rx == LINK_MASTER_HANDSHAKE || tx == LINK_MASTER_HANDSHAKE) {
                p->state     = GBA_PKT_CRC;
                p->timing_us = GBA_PKT_TIMING_COMMAND_BYTES;
            }
            break;

        case GBA_PKT_CRC:
            /* the CRC word was just transmitted (next_tx returned p->crc); start a fresh command
             * packet. onTransiveDone(crc): -> command, crc = 0. */
            p->state     = GBA_PKT_COMMAND;
            p->crc       = 0x0000;
            p->cmd_index = 0;
            break;

        case GBA_PKT_COMMAND:
            /* onReceive(command): store rx, accumulate; plus tx accumulates (Celio sums both). */
            if (p->cmd_index < 8) {
                p->rx_cmd[p->cmd_index] = rx;
                p->tx_cmd[p->cmd_index] = tx;
                p->crc = (uint16_t)(p->crc + rx);
                p->crc = (uint16_t)(p->crc + tx);
                p->cmd_index++;
            }
            /* onTransiveDone(command): timing + completion. */
            if (p->cmd_index == 7) {
                p->timing_us = GBA_PKT_TIMING_BETWEEN_COMMANDS;
            }
            if (p->cmd_index == 8) {
                p->cmd_index    = 0;
                p->state        = GBA_PKT_CRC;
                p->timing_us    = GBA_PKT_TIMING_COMMAND_BYTES;
                packet_complete = true;
            }
            break;
    }

    return packet_complete;
}
