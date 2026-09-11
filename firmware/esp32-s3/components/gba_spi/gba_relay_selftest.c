/* gba_relay_selftest.c — Phase 1 of docs/16-single-chip-trade-plan.md (gba-switch-bridge repo).
 *
 * Goal: prove the NEW shared-SRAM relay plumbing end-to-end against a REAL cartridge, in the
 * TRADE counter (not the union-room lobby this session's earlier work proved out), WITHOUT real
 * Wi-Fi yet — that's Phase 2/3. This isolates "did we build the relay right" from "does the radio
 * work", matching the project's established go/no-go gate style.
 *
 * What it does: brings up the bare-metal core1 GBA adapter exactly like gba_spi_selftest() does,
 * but wired to gba_relay's REAL callbacks instead of the always-empty stub, with a synthetic
 * Switch peer seeded immediately (the same peer_id/beacon shape already proven against a real
 * cartridge in the union-room fake-peer capture — see host/pico_host.py's PeerSim in
 * gba-switch-bridge, and ldn_brain.c's BRAIN_BEACON for the trade-room activity byte). Whatever
 * the GBA sends via SendDataAndWait is echoed straight back as the "Switch" slot, so
 * ReceiveData/ReceiveDataAndWaitResponse has something to return — a loopback that proves the
 * full round trip works before any real Switch is involved.
 *
 * On the real GBA: get to the trade counter (not the union room) and search for a partner. Watch
 * for: ROOM lines showing setup_role 0x3c/0x3f and broadcast_activity 0x04 (confirms the GBA is
 * asking for the trade counter and we're decoding it correctly); the existing MON line's cmds/
 * logins/resets exactly as before (confirms the relay didn't regress the proven link); and SLOT
 * lines when the GBA sends/receives trade data (confirms the round trip).
 */
#include "gba_spi.h"
#include "gba_wap.h"
#include "gba_relay.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "soc/gpio_reg.h"   /* REG_READ(GPIO_IN_REG) for the SI-readback self-check */

static const char *TAG = "gba_relay_st";

/* MON/TRACE/ANOMALY/CADENCE counters are declared in gba_spi.h — the anomaly-snapshot/trace/
 * cadence detection this session's Union-Room debugging built runs unconditionally inside
 * gba_spi_core1_entry() regardless of which gba_wap_io backend is wired in, so printing them here
 * gets the same rich telemetry "for free" instead of needing a second copy of the detection logic. */

/* The exact trade-counter beacon already used by ldn_brain.c (activity byte 0x04, word3 low byte)
 * and, before that, proven against a real cartridge in the union-room fake-peer capture (only the
 * activity nibble differs: 0x40 there, 0x04 here). Peer id matches the value the earlier proven
 * capture recorded the GBA carrying back in its Connect command, for direct comparability. */
static const uint32_t kSyntheticBeacon[6] = {
    0x0f820002, 0x0000c979, 0x00000000, 0xe3000004, 0xffc2cdbb, 0x00000000,
};
#define SYNTHETIC_PEER_ID 0x00002abcu

void gba_spi_trade_relay_selftest(void)
{
    gba_spi_init();
    gba_relay_init();

    static gba_wap_io io;
    gba_relay_fill_io(&io);
    gba_spi_set_core1_io(&io);

    /* Seed the synthetic peer immediately — BroadcastReadPoll/End can find it from the very first
     * poll, no need to wait for the GBA to reach any particular state first. */
    gba_relay_set_peer(SYNTHETIC_PEER_ID, kSyntheticBeacon);

    ESP_LOGI(TAG, "PHASE 1 SELFTEST: bare core1 + REAL gba_relay backend (synthetic peer, no Wi-Fi "
                  "yet). Get the GBA into the TRADE counter (not the union room) and search.");
    ESP_LOGI(TAG, "synthetic peer_id=0x%08x beacon activity=0x%02x",
             SYNTHETIC_PEER_ID, (unsigned)(kSyntheticBeacon[3] & 0xFFu));

    gba_core1_start();

    /* SI-readback self-check (2026-09-11): core 1's first instruction sets SI idle-high through
     * its dedicated-GPIO out channel; SI's pad has input-enable on (gba_spi_init), so GPIO_IN_REG
     * bit 13 shows the ACTUAL pad level. If this reads 0, the dedic TX path is not driving the pin
     * (e.g. the block's CPU-peripheral clock gate — the first-flash bug) and a GBA would see a dead
     * adapter; catch it here instead of wasting a hardware run. */
    vTaskDelay(pdMS_TO_TICKS(50));
    {
        int si = (int)((REG_READ(GPIO_IN_REG) >> 13) & 1);
        if (si) ESP_LOGI(TAG, "SI readback OK (pad=1): dedic-GPIO TX path verified");
        else    ESP_LOGE(TAG, "SI readback FAILED (pad=0): dedic TX not driving the pin — GBA will NOT detect the adapter");
    }

    uint32_t p_clk = 0, p_logins = 0, p_cmds = 0, p_resets = 0, p_last_cmd = 0, p_last_reset = 0;
    uint32_t p_skips = 0, p_resyncs = 0;
    uint32_t p_login_n = 0;   /* LOGIN_RX print throttle (see below) */
    uint32_t p_setup_seen = 0, p_bcast_seen = 0;
    uint32_t p_alive = 0, alive_ticks = 0;
    uint8_t  p_anomaly_seq = 0;
    /* THROTTLE the MON line to at most once/second (mon_ticks), instead of once per counter
     * change. During a fast poll loop (e.g. BroadcastReadPoll every ~15-20ms) that was up to
     * 60+ UART writes/second — each one a blocking call on core0 that could plausibly stall the
     * shared bus fabric long enough to blow the REAL GBA's own (much tighter than ours) SIO
     * timing budget, even though our own generous timeouts would tolerate the same delay fine.
     * This is the architectural hypothesis for the "zero corruption, GBA comm-errors anyway"
     * failures seen in both the Union Room and here: our own diagnostic logging may be the stall
     * source. The dropped periodic TRACE dump (moved to ANOMALY-only, which is rare) removes the
     * single heaviest print (~2700 chars) from the hot path entirely. */
    uint32_t mon_ticks = 0;
    bool     mon_dirty = false;
    uint32_t gba_slot_buf[GBA_RELAY_SLOT_WORDS];

    /* CORE1 WATCHDOG — 2026-09-10 finding: an exception handler now installed on core1 (see
     * gba_core1.c) proved a freeze reproduction never faulted (no Guru Meditation) — so the
     * "Communication errors" wall this whole session chased is a HANG, not a crash. Root cause:
     * xfer32()/xfer_word()'s mid-word wait for the SC rising edge is deliberately an unbounded
     * `while (!CLK_HIGH()) {}` — see gba_spi.c's CLK_GIVEUP comment: a counter in THAT exact loop
     * was already tried once and shifted the sample point enough to corrupt a bit (0x494E ->
     * 0x929C), so it can't be timeout-guarded the same way the other wait loops are without
     * risking that same regression. If the real GBA ever stops toggling SC mid-word (for whatever
     * reason — possibly its own internal retry timeout expiring against our occasional ~150-
     * 400ms-late replies), core1 spins on that one instruction forever with no way out.
     * Can't fix the sample loop itself safely, so recover from the OUTSIDE instead: if none of
     * the progress counters (cmds/resets — a reset+resync IS progress, it's the link recovering)
     * move for WD_STALL_MS while a GBA has been seen at all, core1 is genuinely stuck, not just patiently
     * idle at a word-start boundary (WORD_START_GIVEUP already covers legitimate pauses up to
     * ~10s — e.g. room-select animations — so this threshold sits comfortably above that to avoid
     * false-triggering on a real long pause). Reboot core1 exactly like the first bring-up
     * (gba_core1_start() is idempotent: unstall + reset + fresh boot address) — the GBA's own
     * login_resync/cmd_resync are already designed to ride out exactly this kind of adapter
     * hiccup, so this should look like a brief glitch rather than a dead link. */
    #define WD_STALL_MS 3000u   /* shortened for faster checkpoint-diagnostic iteration, not for recovery */
    #define WD_STALL_TICKS (WD_STALL_MS / 200u)
    uint32_t wd_p_cmds = 0, wd_p_resets = 0, wd_stall_ticks = 0;
    uint32_t wd_fires = 0;
    bool death_dumped = false;   /* one TRACE dump per death (re-armed when progress resumes) */

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(200));

        if (g_gba_cmds != wd_p_cmds || g_gba_resets != wd_p_resets) {
            wd_p_cmds = g_gba_cmds; wd_p_resets = g_gba_resets; wd_stall_ticks = 0;
            death_dumped = false;
        } else if (g_gba_clock_seen && ++wd_stall_ticks >= WD_STALL_TICKS) {
            wd_stall_ticks = 0;
            wd_fires++;
            /* DEATH DUMP (2026-09-11): the 2026-09-11 A/B runs died with ZERO adapter-side error
             * counters — flawless cadence, then the GBA ran its own librfu retry ladder (130ms /
             * 467ms gaps) and latched the fatal error. Whatever went wrong is therefore only
             * visible in the CONTENT of the last exchanges, which core1 already records in the
             * cmd_trace ring but this monitor never printed. Print the last GBA_SPI_CMD_TRACE_N
             * tx/rx pairs ONCE per death, on the FIRST watchdog fire after real progress (>100
             * cmds — skips menu-phase idle fires), BEFORE the restart below can append anything.
             * Tags: H=hdr-read h=hdr-retry D=data-read R=resp-hdr o=resp-data a/i=async-ack
             * X=cmd_resync-recovered. */
            if (!death_dumped && g_gba_cmds > 100) {
                death_dumped = true;
                uint32_t n = g_cmd_trace_n;
                uint32_t start = (n > GBA_SPI_CMD_TRACE_N) ? (n - GBA_SPI_CMD_TRACE_N) : 0;
                char line[1600]; int off = 0;
                off += snprintf(line, sizeof(line), "DEATH TRACE n=%u (oldest first):", (unsigned)n);
                for (uint32_t k = start; k < n && off < (int)sizeof(line) - 24; k++) {
                    uint32_t i = k & (GBA_SPI_CMD_TRACE_N - 1);
                    off += snprintf(line + off, sizeof(line) - off, " %c[%08x/%08x]",
                                     g_cmd_trace_tag[i], (unsigned)g_cmd_trace_tx[i],
                                     (unsigned)g_cmd_trace_rx[i]);
                }
                ESP_LOGW(TAG, "%s", line);
            }
            ESP_LOGW(TAG, "CORE1 WATCHDOG #%u: no progress for %ums (cmds=%u resets=%u last_cmd=0x%02x "
                          "last_checkpoint=0x%03x) — forcing a restart",
                     (unsigned)wd_fires, (unsigned)WD_STALL_MS, (unsigned)g_gba_cmds,
                     (unsigned)g_gba_resets, (unsigned)g_gba_last_cmd, (unsigned)g_cp);
            g_core1_alive = 0;
            gba_core1_start();
        }

        if (++alive_ticks >= 5) {
            alive_ticks = 0;
            ESP_LOGI(TAG, "core1_alive=%u (%s)", (unsigned)g_core1_alive,
                     g_core1_alive != p_alive ? "RUNNING \xe2\x9c\x93" : "stalled \xe2\x9c\x97");
            p_alive = g_core1_alive;
            /* CADENCE: worst gap seen so far between two whole commands, in core1's OWN cycle
             * counter (ccount — private per-CPU, not shared with core0) converted to microseconds.
             * Measured safely BETWEEN whole commands (never inside a bit-sampling loop), so it
             * can't itself perturb timing. Answers directly: did core1 ever get held up for an
             * anomalously long stretch (e.g. a shared-bus stall from core0's PSRAM/flash activity)
             * right before a freeze? Printed every ~1s regardless of GBA activity — the frozen max
             * survives and is still visible even after the link has already died. Normal cadence
             * is VBlank-paced (~16.7ms = ~4000000 cyc @240MHz); anything wildly larger is the tell. */
            ESP_LOGI(TAG, "CADENCE max=%u us at cmd#%u (n=%u samples)",
                     (unsigned)(g_cmd_cadence_max_cyc / 240), (unsigned)g_cmd_cadence_max_idx,
                     (unsigned)g_cmd_cadence_n);
            /* Last GBA_SPI_CMD_CADENCE_N raw gaps (oldest first) — the trend right up to "now",
             * which after a freeze IS the trend right up to the last command that ever completed. */
            {
                uint32_t n = g_cmd_cadence_n;
                uint32_t start = (n > GBA_SPI_CMD_CADENCE_N) ? (n - GBA_SPI_CMD_CADENCE_N) : 0;
                char line[700]; int off = 0;
                off += snprintf(line, sizeof(line), "CADENCE last (us):");
                for (uint32_t k = start; k < n && off < (int)sizeof(line) - 8; k++)
                    off += snprintf(line + off, sizeof(line) - off, " %u",
                                     (unsigned)(g_cmd_cadence_cyc[k & (GBA_SPI_CMD_CADENCE_N - 1)] / 240));
                ESP_LOGI(TAG, "%s", line);
            }
        }

        /* LOGIN_RX (2026-09-11 dedic-GPIO diagnostic): the exact challenge words core1 READS during
         * login, straight from the recorder ring gba_spi.c already keeps. Decisive discriminator
         * for the "clk=1 but logins=0" failure: a clean table sequence (ffff494e b6b1494e b6b1544e
         * abb1544e ... b0bb8001) = RX perfect, TX is what the GBA rejects; garbled/shifted words =
         * the dedic input path is seeing edge noise the old APB-synchronizer read filtered out.
         * Printed at most once per 200ms tick and only when new words arrived. */
        {
            extern volatile uint32_t g_login_rx[16], g_login_n;
            if (g_login_n != p_login_n) {
                p_login_n = g_login_n;
                ESP_LOGI(TAG, "LOGIN_RX n=%u last16: %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x",
                         (unsigned)g_login_n,
                         (unsigned)g_login_rx[0], (unsigned)g_login_rx[1], (unsigned)g_login_rx[2], (unsigned)g_login_rx[3],
                         (unsigned)g_login_rx[4], (unsigned)g_login_rx[5], (unsigned)g_login_rx[6], (unsigned)g_login_rx[7],
                         (unsigned)g_login_rx[8], (unsigned)g_login_rx[9], (unsigned)g_login_rx[10], (unsigned)g_login_rx[11],
                         (unsigned)g_login_rx[12], (unsigned)g_login_rx[13], (unsigned)g_login_rx[14], (unsigned)g_login_rx[15]);
            }
        }

        /* Room-select bytes — confirms the GBA is asking for the TRADE counter and that our
         * passive on_raw_command decode is reading the right bytes. */
        gba_relay_room_info_t room;
        gba_relay_get_room_info(&room);
        if (room.setup_seen != p_setup_seen) {
            p_setup_seen = room.setup_seen;
            ESP_LOGI(TAG, "ROOM setup_role=0x%02x (%s) raw0=0x%08x", room.setup_role,
                     room.setup_role == 0x3f ? "HOST" : room.setup_role == 0x3c ? "JOIN" : "?",
                     (unsigned)room.setup_raw0);
        }
        if (room.broadcast_seen != p_bcast_seen) {
            p_bcast_seen = room.broadcast_seen;
            ESP_LOGI(TAG, "ROOM broadcast_activity=0x%02x (%s) raw=%08x %08x %08x %08x %08x %08x",
                     room.broadcast_activity,
                     room.broadcast_activity == 0x04 ? "TRADE" :
                     room.broadcast_activity == 0x40 ? "UNION" : "?",
                     (unsigned)room.broadcast_raw[0], (unsigned)room.broadcast_raw[1],
                     (unsigned)room.broadcast_raw[2], (unsigned)room.broadcast_raw[3],
                     (unsigned)room.broadcast_raw[4], (unsigned)room.broadcast_raw[5]);
        }

        /* GBA outgoing slot -> loopback straight back as the "Switch" slot, so ReceiveData has
         * something to return. This is the round-trip proof for Phase 1 (real content comes in
         * Phase 3, once a real Switch is on the other end instead of this loopback). */
        int n = gba_relay_take_gba_slot(gba_slot_buf, GBA_RELAY_SLOT_WORDS);
        if (n > 0) {
            char line[256]; int off = 0;
            off += snprintf(line + off, sizeof(line) - off, "SLOT gba->us (%d words):", n);
            for (int i = 0; i < n && off < (int)sizeof(line) - 12; i++)
                off += snprintf(line + off, sizeof(line) - off, " %08x", (unsigned)gba_slot_buf[i]);
            ESP_LOGI(TAG, "%s", line);
            gba_relay_put_switch_slot(gba_slot_buf, n);
            ESP_LOGI(TAG, "SLOT looped back as the Switch's slot (%d words)", n);
        }

        /* ANOMALY: a magic-valid, non-echo command byte we don't recognize — frozen the instant it
         * happens (see gba_spi.c's anomaly_snapshot()), independent of this loop's 200ms cadence,
         * so it can't get pushed out of the trace window by whatever runs before we get here. */
        if (g_anomaly_seq != p_anomaly_seq) {
            p_anomaly_seq = g_anomaly_seq;
            ESP_LOGI(TAG, "ANOMALY #%u at cmd_idx=%u hdr=0x%08x (cmd=0x%02x not in known whitelist)",
                     g_anomaly_seq, (unsigned)g_anomaly_cmd_idx, (unsigned)g_anomaly_hdr,
                     (unsigned)(uint8_t)g_anomaly_hdr);
            char line[1600]; int off = 0;
            off += snprintf(line, sizeof(line), "ANOMALY snapshot (oldest first):");
            for (int k = 0; k < GBA_SPI_ANOMALY_SNAP_N && off < (int)sizeof(line) - 24; k++)
                off += snprintf(line + off, sizeof(line) - off, " %c[%08x/%08x]",
                                 g_anomaly_snap_tag[k], (unsigned)g_anomaly_snap_tx[k],
                                 (unsigned)g_anomaly_snap_rx[k]);
            ESP_LOGI(TAG, "%s", line);
        }

        static uint32_t p_word_resyncs = 0;
        if (g_gba_clock_seen != p_clk || g_gba_logins != p_logins || g_gba_cmds != p_cmds ||
            g_gba_resets != p_resets || g_gba_last_cmd != p_last_cmd ||
            g_gba_last_reset != p_last_reset || g_gba_skips != p_skips || g_cmd_resyncs != p_resyncs ||
            g_word_resyncs != p_word_resyncs)
            mon_dirty = true;

        if (!mon_dirty || ++mon_ticks < 5) continue;   /* throttle: at most 1 MON line/second */
        mon_ticks = 0; mon_dirty = false;
        p_clk = g_gba_clock_seen; p_logins = g_gba_logins; p_cmds = g_gba_cmds;
        p_resets = g_gba_resets; p_last_cmd = g_gba_last_cmd; p_last_reset = g_gba_last_reset;
        p_skips = g_gba_skips; p_resyncs = g_cmd_resyncs; p_word_resyncs = g_word_resyncs;
        ESP_LOGI(TAG, "MON clk=%u logins=%u cmds=%u last_cmd=0x%02x skips=%u resyncs=%u word_resyncs=%u resets=%u last=0x%08x",
                 (unsigned)g_gba_clock_seen, (unsigned)g_gba_logins, (unsigned)g_gba_cmds,
                 (unsigned)g_gba_last_cmd, (unsigned)g_gba_skips, (unsigned)g_cmd_resyncs, (unsigned)g_word_resyncs,
                 (unsigned)g_gba_resets, (unsigned)g_gba_last_reset);
    }
}
