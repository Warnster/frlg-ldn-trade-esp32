/* ldn_gba.c — SINGLE-CHIP drop-in replacement for ldn_pico.c (docs/16 Phase 2, 2026-09-11).
 *
 * Implements the EXACT ldn_pico_* API (ldn_pico.h) so the LDN join FSM (ldn_probe.c) and the trade
 * brain (ldn_brain.c) stay byte-identical — but backed by the on-chip core1<->core0 shared-SRAM
 * relay (gba_relay, components/gba_spi) instead of a UART link to a separate Pico. Selected at build
 * time by CONFIG_LDN_GBA_SINGLE_CHIP (mutually exclusive with ldn_pico.c in CMakeLists).
 *
 * The GBA link itself runs on bare-metal core1 (gba_spi_core1_entry, proven 2026-09-11: 9,278
 * clean commands in the Union Room). core0 (this file) only moves slots/peer across the relay
 * mailboxes; it never touches the wire. gba_relay + gba_spi_init + gba_core1_start are done by
 * app_main BEFORE run_private_join (see ldn_probe.c), exactly where ldn_pico_init used to sit.
 *
 * SLOT FORMAT — the one detail that must match ldn_pico.c byte-for-byte (project rule: match the
 * bytes, don't infer). Both transports carry the SAME underlying bytes; they differ only in packing:
 *   - Pico: the WAP UART stream is u32-LE; ldn_pico reads the SendDataAndWait(0x25) payload as raw
 *     LE bytes, takes the first 14, and re-reads them as 7 u16-LE (ldn_pico.c parse_wap).
 *   - Relay: gba_relay hands core0 the 0x25 data as u32 WORDS (the exact values the GBA clocked,
 *     the same values the Pico's send32 serialized). So to reproduce the Pico's 7 u16 we LE-
 *     serialize those words, truncate to 14 bytes, and re-read as 7 u16-LE — identical result.
 *   The reverse (Switch slot -> GBA) mirrors ldn_pico_send_slot: 14 bytes padded to 16 = 4 u32-LE
 *   words handed to gba_relay_put_switch_slot. */
#include "sdkconfig.h"
#if CONFIG_LDN_PROBE_CONTROL_PORT && CONFIG_LDN_GBA_SINGLE_CHIP
#include <string.h>
#include "ldn_pico.h"          /* the API we implement */
#include "gba_relay.h"         /* the on-chip relay we implement it over */

/* Same real captured JP-FireRed beacon + peer id as ldn_pico.c / ldn_brain.c, activity byte 0x04
 * (ACTIVITY_TRADE). Kept identical so the GBA-facing behaviour is unchanged from the proven path. */
static const uint32_t PEER_ID = 0x00002abc;
static const uint32_t PEER_BEACON[6] = {
    0x0f820002, 0x0000c979, 0x00000000, 0xe3000004, 0xffc2cdbb, 0x00000000 };

/* core0-side diagnostics mirroring ldn_pico's, so the existing PICO_LINK/CMD_HIST prints keep working. */
static uint32_t s_slots_taken, s_recv_sent, s_peer_sent;

/* Host Switch's real in-game identity, decoded from the LDN advert (ldn_pico_set_peer_identity). */
static uint16_t s_peer_tid;
static uint8_t  s_peer_name[8];
static bool     s_peer_id_valid;

void ldn_pico_set_peer_identity(uint16_t tid, const uint8_t name8[8])
{
    s_peer_tid = tid;
    memcpy(s_peer_name, name8, 8);
    s_peer_id_valid = true;
}

/* Build the 6-word (24-byte) GBA parent-candidate beacon carrying `tid` + `name8` (FRLG charmap,
 * copied straight through — the advert record uses the same charmap as the GBA uname). Layout +
 * checksum per pokefirered librfu_rfu.c rfu_REQ_configGameData (see ldn_pico.h / the research notes):
 * word0=serialNo|compat, word1 low16=TID, word3 b0=activity(TRADE) b3=checksum, word4/5=uname[8].
 * checksum = ~(sum gname[0..7] + sum uname[0..7]) & 0xFF. */
static void build_gba_beacon(uint16_t tid, const uint8_t name8[8], uint32_t beacon6[6])
{
    uint8_t b[24] = {0};
    b[0] = 0x02; b[1] = 0x00;                 /* serialNo = RFU_SERIAL_GAME */
    b[2] = 0x82; b[3] = 0x0f;                 /* gname[0:2] compatibility (version/language) */
    b[4] = tid & 0xFF; b[5] = (tid >> 8) & 0xFF;   /* gname[2:4] in-game TID (LE) */
    b[12] = 0x04;                             /* gname[10] activity = ACTIVITY_TRADE (GBA lists it at the trade counter) */
    for (int i = 0; i < 8; i++) b[16 + i] = name8[i];   /* uname[8] = trainer name */
    uint8_t cs = 0;
    for (int i = 2; i < 10; i++) cs += b[i];   /* gname[0..7] = b[2..9] */
    for (int i = 16; i < 24; i++) cs += b[i];  /* uname[0..7] = b[16..23] */
    b[15] = (uint8_t)~cs;                       /* gname[13] checksum */
    for (int k = 0; k < 6; k++)
        beacon6[k] = (uint32_t)b[4*k] | ((uint32_t)b[4*k+1] << 8) |
                     ((uint32_t)b[4*k+2] << 16) | ((uint32_t)b[4*k+3] << 24);
}

void ldn_pico_init(void)
{
    /* No-op: the relay + core1 are brought up by app_main (gba_relay_init + gba_spi_init +
     * gba_spi_set_core1_io + gba_core1_start) before run_private_join, mirroring how ldn_pico_init
     * set up its UART before the loop. Kept as a symbol so ldn_probe.c's call site is unchanged. */
}

/* The relay's mailboxes are updated by core1 autonomously; there is nothing to "drain" on core0.
 * Kept as a no-op so ldn_probe.c / ldn_brain.c call sites stay identical. */
void ldn_pico_poll(void) { }

int ldn_pico_take_gba_slot(uint16_t words[7])
{
    uint32_t w32[GBA_RELAY_SLOT_WORDS];
    int n = gba_relay_take_gba_slot(w32, GBA_RELAY_SLOT_WORDS);
    if (n <= 0) return 0;
    /* LE-serialize the data words, take the first 14 bytes, re-read as 7 u16-LE (ldn_pico parity). */
    uint8_t raw[16] = {0};
    int nb = n * 4; if (nb > 14) nb = 14;
    for (int i = 0; i < nb; i++) raw[i] = (uint8_t)(w32[i >> 2] >> (8 * (i & 3)));
    for (int i = 0; i < 7; i++) words[i] = (uint16_t)(raw[2 * i] | (raw[2 * i + 1] << 8));
    s_slots_taken++;
    return 1;
}

void ldn_pico_send_slot(const uint8_t slot14[14])
{
    /* 14 bytes -> 16 (4 u32-LE) exactly like ldn_pico_send_slot's pad-to-16, then publish to core1. */
    uint8_t buf[16] = {0};
    memcpy(buf, slot14, 14);
    uint32_t w32[4];
    for (int k = 0; k < 4; k++)
        w32[k] = (uint32_t)buf[4*k] | ((uint32_t)buf[4*k+1] << 8) |
                 ((uint32_t)buf[4*k+2] << 16) | ((uint32_t)buf[4*k+3] << 24);
    gba_relay_put_switch_slot(w32, 4);
    s_recv_sent++;
}

void ldn_pico_send_peer(uint32_t peer_id, const uint32_t beacon6[6])
{
    gba_relay_set_peer(peer_id, beacon6);
}

void ldn_pico_send_no_peer(void) { gba_relay_clear_peer(); }

void ldn_pico_advertise_peer(void)
{
    uint32_t beacon6[6];
    if (s_peer_id_valid) build_gba_beacon(s_peer_tid, s_peer_name, beacon6);
    else memcpy(beacon6, PEER_BEACON, sizeof(beacon6));   /* fallback: hardcoded placeholder */
    gba_relay_set_peer(PEER_ID, beacon6);
    s_peer_sent++;
}

void ldn_pico_stats(uint32_t out[5])
{
    /* {wap_seen, slots_seen(0x25), slots_taken, recv_slots_sent, peer_sent} — same shape as ldn_pico. */
    out[0] = gba_relay_wap_seen();
    out[1] = gba_relay_send_count();
    out[2] = s_slots_taken;
    out[3] = s_recv_sent;
    out[4] = s_peer_sent;
}

uint32_t ldn_pico_connect_count(void) { return gba_relay_connect_count(); }

/* Per-command histogram: the relay only tracks the few commands the gate cares about, so answer
 * those exactly and 0 for the rest (the CMD_HIST print degrades gracefully — the join-driving
 * bytes 0x1f/0x25 are accurate; the purely-diagnostic ones read 0 on the single-chip path). */
uint32_t ldn_pico_cmd_count(uint8_t cmd) { return gba_relay_cmd_count(cmd); }

bool ldn_pico_get_gba_identity(uint16_t *tid, uint8_t name8[8])
{
    return gba_relay_get_gba_identity(tid, name8);
}

/* No separate relay chip to report back — the two-chip PICO_RX diagnostic is not meaningful here.
 * Surface the room-info decode instead (setup/broadcast generation counts) so the line still has
 * signal: {broadcast_seen, setup_seen, peer_present-ish}. */
void ldn_pico_diag(uint32_t out[3])
{
    gba_relay_room_info_t ri;
    gba_relay_get_room_info(&ri);
    out[0] = ri.broadcast_seen;
    out[1] = ri.setup_seen;
    out[2] = ri.broadcast_activity;
}

void ldn_pico_tx_bootsel_test(void) { /* no Pico to reboot on the single chip */ }
#endif
