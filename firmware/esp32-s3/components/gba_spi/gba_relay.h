/* gba_relay — the shared-internal-SRAM relay between core1 (the bare-metal GBA-adapter loop,
 * gba_spi_core1_entry) and core0 (the LDN/Switch side). Replaces the old ESP<->Pico UART bridge
 * now that both sides live on one chip (docs/16-single-chip-trade-plan.md, Phase 1).
 *
 * Implements the ENTIRE gba_wap_io callback surface (get_peer/peer_id/take_slot/put_slot) plus the
 * optional on_raw_command observer — these 5 callbacks are the full interface gba_wap.c ever
 * touches (confirmed by exploration: every other command is answered from gba_wap_io's own local
 * router state, no backend involved). See gba_wap.h for the callback contracts this implements.
 *
 * Design: internal SRAM on the S3 is NOT per-core cached, so there is no cache-coherency problem —
 * only ordering (compiler reordering, and making sure a multi-word payload is fully written before
 * the reader notices it changed). Each direction is a single-slot "mailbox" (not a queue — mirrors
 * the two-chip system's `relay.rs` design, which was proven sufficient: the GBA sends at most one
 * slot per VBlank, and each side only ever cares about the LATEST value, not history) using a
 * classic release/acquire sequence-number pattern: writer fills the payload with plain (non-atomic)
 * stores, THEN atomically bumps a sequence counter with release order; the reader atomically loads
 * the sequence counter with acquire order FIRST, and only trusts the payload if the sequence
 * changed. The C11 memory model guarantees the plain payload writes are visible after the matching
 * acquire load, even though the payload fields themselves aren't atomic — this is the standard
 * "message passing through one atomic" idiom, correct and lock-free.
 *
 * CORE-1 SIDE CONSTRAINT (gba_core1.c's own rules, restated here because it's easy to violate by
 * accident when adding new shared state): every function called from core1 (the 4 gba_wap_io
 * callbacks + on_raw_command) must be IRAM_ATTR, must never allocate, and must NEVER block/spin
 * waiting on core0 — core1's watchdogs are disabled, so nothing will ever recover a core1 hang.
 * "Nothing new since last time" is always a valid, cheap, immediate answer.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "gba_wap.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Generous headroom for a single mailbox slot's word count. The real GBA RFU slot is small (a few
 * words), but relay.rs's proven two-chip design capped its buffer at 14 words — match that. */
#define GBA_RELAY_SLOT_WORDS 14

/* Zero the shared structure. Call ONCE from core0, BEFORE gba_core1_start(). Not safe to call
 * concurrently with core1 running (it isn't yet at this point in boot, by construction). */
void gba_relay_init(void);

/* Fill a gba_wap_io with the relay-backed callbacks + ctx=NULL (relay state is one static
 * instance, not per-context) and zero the struct's local router-state fields. Pass the result to
 * gba_spi_set_core1_io() BEFORE gba_core1_start() — exactly the same wiring point the stub
 * providers use in gba_spi_selftest(). */
void gba_relay_fill_io(gba_wap_io *io);

/* ---- core0-side API — call ONLY from core0 (an LDN/FreeRTOS task), never from core1 ---- */

/* Publish (peer_id != 0) or clear (peer_id == 0) the Switch peer's identity + 6-word FRLG beacon.
 * The GBA discovers this via BroadcastReadPoll/BroadcastReadEnd and connects via Connect/
 * IsConnecting/FinishConnecting — gba_wap_respond ALREADY implements the 3-poll "still connecting"
 * fakeout internally using its own io->connecting_polls/connected fields, so this call only needs
 * to publish presence, not run a connection state machine. */
void gba_relay_set_peer(uint32_t peer_id, const uint32_t beacon[6]);
void gba_relay_clear_peer(void);

/* Pop the GBA's latest outgoing slot (published via SendDataAndWait/put_slot) if a new one has
 * arrived since the last call. Returns the word count (0 = nothing new since last call). */
int gba_relay_take_gba_slot(uint32_t *out, int max);

/* Publish a slot for the GBA to receive via ReceiveData/ReceiveDataAndWaitResponse (take_slot). */
void gba_relay_put_switch_slot(const uint32_t *data, int len);

/* Diagnostics: the most recently decoded Setup(0x17) role byte and Broadcast(0x16) activity byte,
 * captured passively via on_raw_command — lets core0 tell whether the GBA is asking for the TRADE
 * counter (role 0x3f host / 0x3c join, activity 0x04) vs the union-room lobby (0x40), without
 * gba_wap.c needing any room-specific logic. `*_seen` is a generation counter (0 = never observed;
 * compare across calls to detect a fresh observation, same seqlock-lite idea as the slot mailboxes). */
typedef struct {
    uint8_t  setup_role;
    uint32_t setup_raw0;       /* the full data[0] word the role byte was extracted from — the real
                                * trade-counter Setup payload has never been captured before (only
                                * the union-room lobby's has), so this lets us confirm/correct the
                                * byte-position assumption from a live run instead of guessing again */
    uint32_t setup_seen;
    uint8_t  broadcast_activity;
    uint32_t broadcast_raw[6]; /* the full 6-word beacon the activity byte was extracted from */
    uint32_t broadcast_seen;
} gba_relay_room_info_t;
void gba_relay_get_room_info(gba_relay_room_info_t *out);

/* Activity counters for the core0 LDN join-driving gate (docs/16 Phase 2). Monotonic; core0 samples
 * them for edge detection. wap_seen = any GBA command (the "GBA alive in wireless mode" signal);
 * connect_count = Connect(0x1f) count (player selected our advertised peer = join trigger);
 * send_count = SendDataWait(0x25) count (trade slots flowing). */
uint32_t gba_relay_wap_seen(void);
uint32_t gba_relay_connect_count(void);
uint32_t gba_relay_send_count(void);
uint32_t gba_relay_cmd_count(uint8_t cmd);   /* per-command-byte count (full flow visibility) */

/* The GBA cartridge's own in-game identity (TID + 8-byte FRLG-charmap trainer name), decoded from
 * its Broadcast(0x16). Returns false until the GBA has broadcast it. Present this to the Switch as
 * the joining player instead of the hardcoded "EMU". */
bool gba_relay_get_gba_identity(uint16_t *tid, uint8_t name8[8]);

/* Build the PARENT UNI sub-frame (3-byte parent LLSF + 70-byte gRecvCmds table: row0 = the
 * Switch's slot, row1 = the GBA's own slot echoed back). Returns the word count written (19 for a
 * full frame). Call from core1; IRAM-safe. The GBA pulls this via ReceiveData(0x26) after a
 * wake-up notification — see gba_relay_take_parent_frame (docs/22). */
int gba_relay_build_parent_frame(uint32_t *out, int max_words);

/* Join-status NI sender diagnostics (docs/23): stage 0-4 = serving that subframe of the
 * JOIN_GROUP_OK sequence, 5 = NI complete (UNI streaming); acks = child LLSF acks consumed;
 * childf = child LLSF subframes seen in its sends. */
uint32_t gba_relay_ni_stage(void);
uint32_t gba_relay_ni_acks(void);
uint32_t gba_relay_ni_childf(void);
uint32_t gba_relay_ni_lasthw(void);   /* raw halfword of last child LLSF subframe (decode check) */

/* ---- core1-side data-phase helpers (docs/22 wake-word model) ----
 * fresh():        peek-only — has core0 published a Switch slot not yet consumed? The clock-master
 *                 wait loop polls this to decide when to send the 0x99660028 wake word.
 * wait_timeout_us: the waiting-state idle-wake timeout, from Setup(0x17) bits 0-7 (16.6ms frames;
 *                 500ms fallback when unset) — on expiry the wake word is 0x99660027 instead.
 * take_parent_frame: the full 0x26 response — count-header word + parent frame (20 words), or a
 *                 lone zero-count word when nothing fresh. Consumes the mailbox. */
bool     gba_relay_switch_slot_fresh(void);
uint32_t gba_relay_wait_timeout_us(void);
int      gba_relay_take_parent_frame(uint32_t *out, int max_words);

#ifdef __cplusplus
}
#endif
