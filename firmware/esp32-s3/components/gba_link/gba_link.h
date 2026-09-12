/* gba_link — abstract physical-link interface for the Gen 3 link-CABLE cart side (docs/25).
 *
 * This is the seam between the hardware-dependent physical layer (the 16-bit UART-framed
 * multiplayer transfer on core1's GPIO — to be written, task #9) and the hardware-INDEPENDENT
 * packet/trade logic above it (gba_packet.c, the trade sections). Everything above this header is
 * portable and host-testable; only the implementation of these functions touches pins.
 *
 * Mirrors Celio-Firmware's src/layers/linkLayer.h, but RESTRUCTURED for our runtime: Celio drives
 * the PIO from interrupts and hands completed transfers to a thread via semaphores. Our core1 is a
 * bare, non-preemptible polling loop (no RTOS, no ISRs — see gba_spi.c). So instead of callbacks +
 * semaphores, the link exposes ONE synchronous call — exchange a single 16-bit word — and the
 * caller (gba_packet's step loop) drives the sequencing. Same protocol, inverted control flow.
 *
 * PROTOCOL: GBA multiplayer serial, 16-bit words, UART-framed at 115200 baud (~8.68us/bit). The
 * parent (master) drives SC; data flows SO->SI around the 2-unit ring. At 115200 baud there are
 * ~2000 core1 cycles per bit, so a bare-metal bit-bang clocks it with comfortable margin (unlike
 * the 2MHz RFU path). See docs/25.
 *
 * PIN MAPPING: pending hardware confirmation (task #8). Our cut cable is wired (gba_spi.c) SC=GPIO12,
 * SO=GPIO11 (GBA->us in), SI=GPIO13 (us->GBA out), SD=GPIO14 — the same SI/SO data lines multiplayer
 * mode uses, so the existing wiring is expected to suffice; the physical-layer impl will confirm. */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GBA_LINK_DISABLED = 0,
    GBA_LINK_MASTER,      /* we drive SC (Celio's proven trade config) */
    GBA_LINK_SLAVE,       /* cart drives SC; we sample (force-cart-master fallback) */
} gba_link_mode_t;

/* Bring the link up in the given mode. Configures pins/clock; idempotent. */
void gba_link_init(gba_link_mode_t mode);
void gba_link_set_mode(gba_link_mode_t mode);
gba_link_mode_t gba_link_get_mode(void);

/* Exchange ONE 16-bit word with the cart and block until the transfer completes.
 *   tx        = the word we present (our SO)
 *   *rx       = the word the cart presented (its SO -> our SI)
 *   timing_us = inter-word gap to hold AFTER this transfer before the next (the packet layer sets
 *               this from its state — handshake vs command-byte vs between-command spacing).
 * Returns false if the cart's clock never came / the transfer timed out (master: we time out
 * waiting for the ready handshake; slave: SC never toggled). The caller decides how to recover. */
bool gba_link_transceive16(uint16_t tx, uint16_t *rx, uint32_t timing_us);

#ifdef __cplusplus
}
#endif
