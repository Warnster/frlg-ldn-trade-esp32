/* gba_spi.c — GPIO BIT-BANG transport for the GBA link (the S3 emulating the Wireless Adapter).
 *
 * Rewritten from the hardware-SPI-slave attempt, which is unusable here: the ESP32 spi_slave frames
 * each word by the CS pin toggling, but the GBA link has NO chip-select — so words never complete.
 * Every project that talks to a GBA as clock-slave bit-bangs (our RP2040 spi.rs; shinyquagsire23/
 * ESP32-GBA-SIO) or uses a bit-count state machine (RP2040 PIO — which the S3 lacks). So we bit-bang,
 * mirroring the proven RP2040 `spi.rs`/`gpi.rs`: the GBA drives SC, we spin on its edges and count 32
 * bits, no CS. Pinned to core1; Wi-Fi/LDN stay on core0. See GBA_SPI_SLAVE_DESIGN.md. */
#include "gba_spi.h"
#include "gba_wap.h"
#include "gba_relay.h"   /* parent-frame builder for the clock-master data push */
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "esp_attr.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

static const char *TAG = "gba_spi";

/* ---- pins (no CS; drop the old GPIO10 wire) ---- */
#define GBA_SC_PIN  12   /* clock  — GBA drives it (input) */
#define GBA_SO_PIN  11   /* GBA->us data (input)  */
#define GBA_SI_PIN  13   /* us->GBA data (output) */
#define GBA_SD_PIN  14   /* reset/ready (GPIO)    */

#define SC_MASK (1u << GBA_SC_PIN)
#define SO_MASK (1u << GBA_SO_PIN)
#define SI_MASK (1u << GBA_SI_PIN)

/* ---- link-pin access layer ----
 * 2026-09-11 (step 4, docs/17 §7 follow-up): switched from GPIO_IN_REG/OUT_W1TS APB register access
 * to the ESP32-S3's DEDICATED GPIO — per-core CPU instructions (ee.get_gpio_in / ee.wr_mask_gpio_out,
 * hal/dedic_gpio_cpu_ll.h) wired straight into the GPIO matrix, bypassing the APB bus entirely.
 * WHY: every GPIO_IN_REG read/OUT write is an APB round-trip over the bus matrix SHARED with core 0
 * (UART logging, flash cache ops, PSRAM traffic) — tens of ns each, with contention-dependent
 * variance. That cost the receive path 50-200ns of detect->sample skew (fixed separately by
 * single-read sampling) and — the remaining suspect after the 2026-09-11 A/B runs showed ZERO
 * receive-side errors — it delayed OUR OWN SI bit (posted APB write issued a poll-granularity-late
 * ~50-200ns into the GBA's 250ns SC-low half-period), eating the GBA's setup time on the bits IT
 * samples from us: a failure mode entirely invisible to our counters, matching the observed
 * "flawless link, then the GBA runs its 130/467ms librfu retry ladder and dies" signature.
 * Dedicated GPIO reads/writes are ~1 CPU cycle, uncontended, per-core — the S3's equivalent of the
 * RP2040's single-cycle SIO, which is what the proven reference implementation uses.
 * Channel map (core 1): IN ch0=SC ch1=SO ch2=SD; OUT ch0=SI. Matrix routing in gba_spi_init().
 * CAVEAT: these instructions address the EXECUTING core's own channels — only bare-metal core 1
 * may call CLK_HIGH/SO_HIGH/SI_HIGH/SI_LOW/LINK_IN once dedic is on. The legacy core-0 paths
 * (gba_spi_run_adapter / gba_spi_task wrappers) are NOT dedic-aware — they're unused by both
 * selftests; do not revive them without routing CORE0_GPIO_* channels too. Core-0 diagnostics that
 * read GPIO_IN_REG directly (cap_task, pin snapshots) still work: the matrix keeps feeding the
 * plain input register in parallel. Set GBA_SPI_DEDIC=0 to fall back to the old APB access. */
#define GBA_SPI_DEDIC 1
#if GBA_SPI_DEDIC
#include "hal/dedic_gpio_cpu_ll.h"
#include "hal/dedic_gpio_ll.h"
#include "esp_private/periph_ctrl.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"   /* GPIO.func_out_sel_cfg — SI oen_sel override below */
#define DG_SC_OUT_BIT 0x2u                     /* core1 dedic OUT channel 1 -> SC (clock-master drive) */
#define DG_SC_BIT   0x1u                       /* core1 dedic IN  channel 0 */
#define DG_SO_BIT   0x2u                       /* core1 dedic IN  channel 1 */
#define DG_SD_BIT   0x4u                       /* core1 dedic IN  channel 2 (unused, routed anyway) */
#define DG_SI_BIT   0x1u                       /* core1 dedic OUT channel 0 */
#define LINK_IN()   dedic_gpio_cpu_ll_read_in()          /* one ee.get_gpio_in — ~1 cycle */
#define LINK_SC(v)  ((v) & DG_SC_BIT)
#define LINK_SO(v)  ((v) & DG_SO_BIT)
#define CLK_HIGH()  (dedic_gpio_cpu_ll_read_in() & DG_SC_BIT)
#define SO_HIGH()   (dedic_gpio_cpu_ll_read_in() & DG_SO_BIT)
#define SI_HIGH()   dedic_gpio_cpu_ll_write_mask(DG_SI_BIT, DG_SI_BIT)   /* one ee.wr_mask_gpio_out */
#define SI_LOW()    dedic_gpio_cpu_ll_write_mask(DG_SI_BIT, 0)
/* Clock-master drive of SC (see gba_spi_master_xfer_word). The pad's output-enable is toggled via
 * GPIO_ENABLE_W1TS/W1TC — plain register writes, IRAM-safe (ESP-IDF's gpio_set_direction is
 * flash-resident and must never be called from bare-metal core 1). */
#define SC_DRIVE_HIGH() dedic_gpio_cpu_ll_write_mask(DG_SC_OUT_BIT, DG_SC_OUT_BIT)
#define SC_DRIVE_LOW()  dedic_gpio_cpu_ll_write_mask(DG_SC_OUT_BIT, 0)
#define SC_OUT_ENABLE()  REG_WRITE(GPIO_ENABLE_W1TS_REG, SC_MASK)
#define SC_OUT_DISABLE() REG_WRITE(GPIO_ENABLE_W1TC_REG, SC_MASK)
/* DEBOUNCE (2026-09-11, found on hardware): raw ee.get_gpio_in is ASYNC and ~4ns-granular — during
 * SC's real 5-20ns breadboard rise/fall the loop reads the transition bouncing 0/1/0/1 and counts
 * phantom edges. First dedic login attempt read HUNDREDS of garbled challenges, all bit-SHIFTED
 * 0x494E fragments (929c=494E<<1 etc.) = duplicated/missed edges, not wrong data. The old APB path
 * was implicitly protected by its 80MHz two-stage input synchronizer; the S3 pads have no Schmitt
 * trigger to help (unlike the RP2040's, on by default). So debounce IN SOFTWARE: a state change is
 * accepted only after LINK_DEBOUNCE consecutive agreeing reads (~12-20ns window) — a software
 * Schmitt trigger, still 3-5x faster than a single old APB read. */
#define LINK_DEBOUNCE 3u
#else
/* Direct-register GPIO for the tight loop (pins <32 → the low GPIO bank). gpio_get_level is too slow. */
#define LINK_IN()   REG_READ(GPIO_IN_REG)
#define LINK_SC(v)  ((v) & SC_MASK)
#define LINK_SO(v)  ((v) & SO_MASK)
#define CLK_HIGH()  (REG_READ(GPIO_IN_REG) & SC_MASK)
#define SO_HIGH()   (REG_READ(GPIO_IN_REG) & SO_MASK)
#define SI_HIGH()   REG_WRITE(GPIO_OUT_W1TS_REG, SI_MASK)
#define SI_LOW()    REG_WRITE(GPIO_OUT_W1TC_REG, SI_MASK)
#define LINK_DEBOUNCE 1u   /* APB path: single-read accept = the proven original semantics */
#endif

/* Debounced SC waits (see LINK_DEBOUNCE's comment). A state is accepted only after LINK_DEBOUNCE
 * consecutive agreeing reads; a disagreeing read resets the run. The give-up counter counts ONLY
 * wrong-state reads (like the originals). link_wait_sc_high returns the LAST snapshot, so the
 * caller samples SO from the very read that confirmed the edge — the single-read atomicity fix is
 * preserved. giveup==0 means wait forever (the tight, pure hot-path wait). */
static inline IRAM_ATTR bool link_wait_sc_low(uint32_t giveup)
{
    uint32_t run = 0, g = 0;
    for (;;) {
        if (!LINK_SC(LINK_IN())) { if (++run >= LINK_DEBOUNCE) return true; }
        else { run = 0; if (giveup && ++g > giveup) return false; }
    }
}
static inline IRAM_ATTR uint32_t link_wait_sc_high(uint32_t giveup, bool *okp)
{
    uint32_t run = 0, g = 0, v;
    for (;;) {
        v = LINK_IN();
        if (LINK_SC(v)) { if (++run >= LINK_DEBOUNCE) { if (okp) *okp = true;  return v; } }
        else { run = 0;   if (giveup && ++g > giveup) { if (okp) *okp = false; return v; } }
    }
}

/* Loop-count give-up for a stopped clock (GBA off/disconnected). Placed ONLY in the wait-for-LOW
 * (inter-bit) loop, never in the wait-for-HIGH+sample path — the RP2040 learned that a counter in the
 * sample path shifts the sample point by a bit (0x494E->0x929C). ~a few ms at 240MHz. */
#define CLK_GIVEUP 300000u
/* Inter-WORD wait budget for the command phase. The gap before the in-room commands is INDETERMINATE
 * and can be SECONDS (the game saves, waits for the player to pick a room, plays the walk-in animation),
 * so we must wait patiently. Safe to be this large because CPU1's interrupt+idle watchdogs are disabled
 * (sdkconfig) — core 1 spins for the clock like the RP2040's bare loop and simply resumes when it
 * returns. ~10s @240MHz; only bails if the GBA is genuinely gone. */
#define WORD_START_GIVEUP 480000000u
#define IDLE_WORD  0x80000000u

static bool s_inited;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* Progress counters — updated by the SPI core with plain writes (NO logging in the hot path, matching
 * the RP2040). A monitor task on the OTHER core prints them, so tracing never delays the bit-bang. */
volatile uint32_t g_gba_clock_seen, g_gba_logins, g_gba_cmds, g_gba_resets, g_gba_last_cmd, g_gba_last_reset;
volatile uint32_t g_gba_skips;   /* idle/non-0x9966 frames skipped while waiting for a real command */
volatile uint32_t g_cmd_resyncs; /* times cmd_resync() recovered a single-bit slip without a full re-login */
volatile uint32_t g_word_resyncs; /* times a bad header self-corrected on the VERY NEXT aligned word —
                                    * no bit-shifting, no relogin, just one extra transfer (round 5 fix) */
volatile uint32_t g_clock_master_swaps;
volatile uint32_t g_parent_frames;        /* parent UNI frames pushed to the GBA as clock master */
volatile uint32_t g_parent_acks;          /* of those, how many the GBA acked with 0x996600A8 */
volatile uint32_t g_parent_ack_last;      /* raw word read back in the ack slot — the diagnosis */
volatile uint32_t g_parent_hdr_rx;        /* raw word the GBA emitted while we clocked the 0x28 header */  /* times we took the clock on a role-change ack */
volatile uint32_t g_login_rx[16], g_login_n;   /* rolling record of login challenge words core1 reads */

/* ---- command-loop trace (temporary) — every xfer_word tx/rx pair in the command phase, rolling.
 * Cheap (plain volatile writes, no logging), so it can't perturb timing — same technique as g_login_rx.
 * Lets us see EXACTLY which exchange the 0x996600bd-echo corruption enters at, instead of guessing. */
#define CMD_TRACE_N 128
volatile uint8_t  g_cmd_trace_tag[CMD_TRACE_N];   /* 'H'=header read, 'D'=data read, 'R'=response hdr write,
                                                    * 'o'=response data write, 'a'/'i'=async-ack write */
volatile uint32_t g_cmd_trace_tx[CMD_TRACE_N], g_cmd_trace_rx[CMD_TRACE_N];
volatile uint32_t g_cmd_trace_n;
static IRAM_ATTR void cmd_trace_push(uint8_t tag, uint32_t tx, uint32_t rx)
{
    uint32_t i = g_cmd_trace_n & (CMD_TRACE_N - 1);
    g_cmd_trace_tag[i] = tag; g_cmd_trace_tx[i] = tx; g_cmd_trace_rx[i] = rx;
    g_cmd_trace_n++;
}

/* Whitelist of every command byte we've actually seen the GBA send in the lobby (login-table-adjacent
 * 0x00, plus every Command enum value from routes.rs). Purely OBSERVATIONAL — matching this list does
 * NOT change how a command is handled, we still answer everything the same way as before. It only lets
 * us tell, after the fact, whether a header that passed the 0x9966-magic check carried a command byte
 * we actually recognize, or one that looks like noise/an uncatalogued command. */
static IRAM_ATTR bool cmd_is_known(uint8_t c)
{
    switch (c) {
    case 0x00: case 0x10: case 0x11: case 0x13: case 0x16: case 0x17: case 0x19: case 0x1a:
    case 0x1b: case 0x1c: case 0x1d: case 0x1e: case 0x1f: case 0x20: case 0x21:
    case 0x24: case 0x25: case 0x26: case 0x27: case 0x28: case 0x3d:
    case 0x35: case 0x36: case 0x37:   /* agbrfu "_AND_CHANGE" family — see gba_wap.h/.c */
        return true;
    default:
        return false;
    }
}

/* ANOMALY SNAPSHOT (temporary diagnostic): the moment a magic-valid, non-echo header carries an
 * unrecognized command byte, FREEZE a copy of the trailing trace window into a dedicated buffer that
 * nothing else can overwrite. Without this, the anomaly gets pushed out of the small rolling cmd_trace
 * ring by however many more (normal) exchanges happen before the core-0 monitor's next ~200ms poll —
 * which is exactly what happened the first time (we only ever saw the lagging last_cmd=0x6a counter,
 * never the actual header word or its surrounding context). Recording this does NOT alter control flow
 * — the command below is still answered exactly as it would be without this block. */
#define ANOMALY_SNAP_N 64
volatile uint8_t  g_anomaly_seq;                          /* bumped once per captured anomaly */
volatile uint32_t g_anomaly_cmd_idx;                      /* g_gba_cmds at the moment of capture */
volatile uint32_t g_anomaly_hdr;                          /* the full offending header word */
volatile uint8_t  g_anomaly_snap_tag[ANOMALY_SNAP_N];
volatile uint32_t g_anomaly_snap_tx[ANOMALY_SNAP_N], g_anomaly_snap_rx[ANOMALY_SNAP_N];
static IRAM_ATTR void anomaly_snapshot(uint32_t hdr)
{
    uint32_t n = g_cmd_trace_n;
    uint32_t start = (n >= ANOMALY_SNAP_N) ? (n - ANOMALY_SNAP_N) : 0;
    for (uint32_t k = 0; start + k < n; k++) {
        uint32_t i = (start + k) & (CMD_TRACE_N - 1);
        g_anomaly_snap_tag[k] = g_cmd_trace_tag[i];
        g_anomaly_snap_tx[k]  = g_cmd_trace_tx[i];
        g_anomaly_snap_rx[k]  = g_cmd_trace_rx[i];
    }
    g_anomaly_hdr = hdr;
    g_anomaly_cmd_idx = g_gba_cmds;
    g_anomaly_seq++;
}

/* ---- jitter detector (temporary) ---- time each command word's 32-bit transfer window (first rising
 * edge → last). Normal @2MHz ≈ 15.5us (~3720 cyc @240MHz). A high-priority ISR (cache/IPC) firing
 * mid-word — which portENTER_CRITICAL does NOT mask — stretches it. We record the worst window, a count
 * of over-long transfers, the cmd index of the last over-long one, and the cmd index at reset, so we can
 * see if the dropout coincides with an interrupt. */
volatile uint32_t g_jit_max_cyc, g_jit_cnt, g_jit_lastcmd, g_reset_cmd;

/* CHECKPOINT (2026-09-10 diagnostic): every identified loop in gba_spi.c/gba_wap.c/gba_relay.c has
 * now been reviewed and is either bounded or provably loop-free — yet core1 still goes silent
 * (frozen SI, frozen counters) while the sniffer shows the GBA keeps clocking real edges afterward,
 * and the installed exception handler (gba_core1.c) proved a repro never faults. That combination
 * means core1 is not spinning on the clock pin at all when it dies — something stops it cold
 * between two of these markers. Written ONLY in the outer per-command loop, around/between whole
 * xfer_word() calls — same safe location as g_gba_cmds, never inside the per-bit sample loop — so
 * it can't perturb sample timing. When frozen, its last value pinpoints which call site core1 was
 * about to enter. Values: 1=before header xfer_word, 0x100+i=before data word i, 2=before
 * gba_wap_respond, 3=before ASYNC_ACK word0, 4=before ASYNC_ACK word1, 5=before response header,
 * 0x300+i=before response word i. */
volatile uint32_t g_cp;
#define JIT_THRESH_CYC 6000u             /* ~25us: >1.5x a clean transfer → an ISR landed mid-word */
static inline IRAM_ATTR uint32_t ccount(void){ uint32_t c; __asm__ __volatile__("rsr %0,ccount":"=r"(c)); return c; }

/* Per-command CADENCE timing (temporary diagnostic): the GBA's own comm-error dialog ("turn off and
 * on the console") means ITS link stack detected a timing/protocol violation and gracefully dropped —
 * not a hang. Measured ONLY between whole commands (ccount() taken right after one command's response
 * is fully sent, and again right after the next), never inside a bit-sampling wait loop — so unlike the
 * earlier jitter detector (which lived in the per-BIT sample path and caused the RP2040-style 1-bit
 * shift), this cannot perturb any sample instant. Normal cadence is VBlank-paced (~16ms); this rolling
 * buffer lets us see whether cadence drifts long right before the comm error instead of guessing. */
#define CMD_CADENCE_N 64
volatile uint32_t g_cmd_cadence_cyc[CMD_CADENCE_N];   /* cycles since the previous command completed */
volatile uint32_t g_cmd_cadence_n;
volatile uint32_t g_cmd_cadence_max_cyc, g_cmd_cadence_max_idx;

/* bare-metal core 1 bring-up (gba_core1.c) */
extern volatile uint32_t g_core1_alive;   /* Stage 1: bumped by the bare core-1 loop = proof of life */
void gba_core1_start(void);

/* ---- two-core firmware logic analyzer (temporary), RE-ARMABLE ----
 * core 1 runs the REAL adapter untouched. core 1 sets cap_arm=1 both after login AND on every drop-back
 * to login (room-entry re-login) — so we can see the RE-LOGIN stream, not just the first login. A core-0
 * task oversamples SC/SO/SI purely by READING the pins (never drives → can't perturb core 1), edge-
 * triggers on the first SC rising edge, reconstructs words from the TRUE edges (framing-agnostic), and
 * scans every bit offset for BOTH 0x9966 (a real command) and 0x494E (a login challenge) so we can tell
 * what the GBA is actually streaming during room entry. */
#define CAP_RAW 8192
static uint8_t cap_raw[CAP_RAW];
static uint8_t cap_bits[2048];
static portMUX_TYPE cap_mux = portMUX_INITIALIZER_UNLOCKED;   /* separate from s_mux → no cross-core block */
volatile int cap_arm, cap_seq, cap_edges, cap_nbits;
volatile int cap_off9966, cap_off494e;                        /* first bit offset of each, or -1 */
volatile uint32_t cap_w9966, cap_w494e, cap_word0;            /* 32-bit word at those offsets / at offset 0 */
volatile uint32_t cap_words[10];                              /* first 10 reconstructed words @0-offset */

void gba_spi_init(void)
{
    if (s_inited) return;
    /* 2026-09-11 fix (pull-downs): the proven RP2040 reference runs PULL-DOWN on every link pin
     * (spi.rs: `type Input<P> = Pin<P, FunctionSio<SioInput>, PullDown>`) — this port had them
     * FLOATING. A floating CMOS input across a micro-flicker (cracked splice, breadboard/socket
     * contact) holds its previous charge for us-to-ms, so a momentary contact-open reads back the
     * PREVIOUS bit's value — which is exactly the one wire-level corruption ever captured
     * (0x9966001a -> 0x9d66001a: the flipped bit's misread value equals the bit transmitted just
     * before it). The RP2040 additionally has Schmitt-trigger hysteresis on by default; the S3 has
     * none at all (not available on this silicon), so a defined disconnect level matters MORE here,
     * not less. ~45k internal pull — negligible load on the GBA's drive. */
    gpio_config_t in = {
        .pin_bit_mask = SC_MASK | SO_MASK | (1ULL << GBA_SD_PIN),
        .mode = GPIO_MODE_INPUT, .pull_up_en = 0, .pull_down_en = 1, .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in);
    gpio_config_t out = {
        /* INPUT_OUTPUT (2026-09-11): input-enable on SI too, so GPIO_IN_REG reflects the actual
         * pad level — lets core 0 verify the dedic TX path really drives the pin (readback
         * self-check in gba_relay_selftest) without a GBA attached. No effect on drive. */
        .pin_bit_mask = SI_MASK, .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = 0, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out);
#if GBA_SPI_DEDIC
    /* Route the link pins into CORE 1's dedicated-GPIO channels (see the access-layer comment at
     * the top). Input routing is additive — GPIO_IN_REG keeps reflecting the pads in parallel, so
     * the core-0 sniffer/pin-snapshot diagnostics are unaffected. OUTPUT routing REPLACES the pad's
     * out mux: after connect_out_signal, plain GPIO_OUT writes to SI from core 0 no longer reach
     * the pad — only core 1's ee.wr_mask_gpio_out does. Must run AFTER gpio_config (which resets
     * the out mux to simple-GPIO). SI's initial idle-high is set by core 1 itself first thing in
     * gba_spi_core1_entry (core 0 physically cannot write core 1's channels). */
    /* FIRST TRY (2026-09-11) MISSED THIS: on the S3 the dedicated-GPIO block sits behind a CPU-
     * peripheral clock gate (SYSTEM.cpu_peri_clk_en.clk_en_dedicated_gpio) which the IDF driver
     * enables in dedic_gpio_build_platform() — without it the OUT channels never drive the pad
     * (observed on hardware: RX matched a login challenge but the GBA never saw our SI responses
     * -> "adapter not detected", clk=1/logins=0 forever). Mirror the driver's exact sequence. */
    PERIPH_RCC_ATOMIC() {
        dedic_gpio_ll_enable_bus_clock(true);
        dedic_gpio_ll_reset_register();
    }
    esp_rom_gpio_connect_in_signal(GBA_SC_PIN, CORE1_GPIO_IN0_IDX, false);
    esp_rom_gpio_connect_in_signal(GBA_SO_PIN, CORE1_GPIO_IN1_IDX, false);
    esp_rom_gpio_connect_in_signal(GBA_SD_PIN, CORE1_GPIO_IN2_IDX, false);
    esp_rom_gpio_connect_out_signal(GBA_SI_PIN, CORE1_GPIO_OUT0_IDX, false, false);
    /* SC also gets a core-1 dedicated OUT channel so we can DRIVE the clock during the trade data
     * phase (the RFU clock role swaps — see gba_spi_master_xfer_word). Routing the out signal is
     * harmless while the pad's output-enable stays CLEAR: the GBA keeps driving SC and we only
     * listen. gpio_config() above left SC as INPUT, i.e. GPIO_ENABLE bit clear — we explicitly
     * clear it again here so this can never come up driving and fight the GBA. */
    esp_rom_gpio_connect_out_signal(GBA_SC_PIN, CORE1_GPIO_OUT1_IDX, false, false);
    GPIO.func_out_sel_cfg[GBA_SC_PIN].oen_sel = 1;   /* OE from GPIO_ENABLE, which we control */
    SC_OUT_DISABLE();                                 /* stay an input until we take the clock */
    /* Force SI's pad output-enable to come from GPIO_ENABLE (set by gpio_config above) instead of
     * the routed peripheral signal's OE. SI is a PERMANENTLY-driven output in the adapter role
     * (the RP2040 reference also drives p_tx unconditionally), so constant OE is exactly right —
     * and it removes any dependence on whether the dedic out channel asserts its own OE. */
    GPIO.func_out_sel_cfg[GBA_SI_PIN].oen_sel = 1;
    ESP_LOGI(TAG, "DEDICATED GPIO active: core1 ee.get_gpio_in/ee.wr_mask_gpio_out (1-cycle, APB-bypass)");
    /* SAFETY: SC is bidirectional now (clock-master path). Confirm at boot that our driver is OFF,
     * i.e. the GBA still owns the clock — if this ever reads driving, we'd be fighting the GBA on a
     * shared push-pull line. Must read 0. */
    ESP_LOGI(TAG, "SC clock-master driver: %s (must be 'off' at boot; GBA owns SC until the data phase)",
             (REG_READ(GPIO_ENABLE_REG) & SC_MASK) ? "ON *** UNEXPECTED ***" : "off");
#else
    SI_HIGH();                 /* SI idles high between words */
#endif
    s_inited = true;
    ESP_LOGI(TAG, "bit-bang up (SC=%d in, SO=%d in, SI=%d out, SD=%d) — NO CS wire. GBA drives clock.",
             GBA_SC_PIN, GBA_SO_PIN, GBA_SI_PIN, GBA_SD_PIN);
}

/* One 32-bit exchange, MSB-first, the GBA clocking SC. Mirrors RP2040 spi.rs::transfer_u32:
 * wait SC low → drive SI bit → wait SC high (rising edge) → sample SO. LOCK-FREE: interrupts are
 * assumed already OFF (run_adapter holds one session-wide critical section = the hard-real-time bare
 * loop). IRAM so flash misses can't stall it. Sets *ok=false and bails if the clock stalls (GBA gone). */
static IRAM_ATTR uint32_t xfer32(uint32_t tx, bool *ok)
{
    uint32_t rx = 0;
    *ok = true;
    for (int i = 31; i >= 0; i--) {
        if (!link_wait_sc_low(CLK_GIVEUP)) { *ok = false; return rx; }   /* wait SC low (debounced) */
        if ((tx >> i) & 1u) SI_HIGH(); else SI_LOW();   /* present our bit while SC is low */
        /* 2026-09-11 fix (single-read sampling): the old code did TWO separate GPIO_IN_REG reads —
         * one to see the rising edge, a SECOND to sample SO. On the RP2040 that structure is
         * harmless (SIO reads are 1-cycle, from the core's private port, uncontended). On the S3
         * every GPIO_IN_REG read is an APB round-trip across the bus matrix SHARED with core 0
         * (UART log bursts, Wi-Fi, cache/PSRAM traffic) — so the second read landed a variable
         * 50-200ns after edge detect, against a 250ns half-bit budget @2MHz, and a bus-arbitration
         * stall could push it past the next falling edge (one wrong bit, framing intact — the
         * observed failure signature). Fix: ONE read; the same value that shows SC high supplies
         * the SO bit, sampled in the SAME bus transaction — detect->sample skew is zero by
         * construction, immune to stalls. This REMOVES an instruction+read from the sample path
         * (the per-bit-timeout lesson forbids ADDING work there; taking work out is strictly
         * safer). Wait loop stays TIGHT, no counter (pure), as before. */
        uint32_t v = link_wait_sc_high(0, NULL);               /* rising edge — tight, debounced */
        rx = (rx << 1) | (LINK_SO(v) ? 1u : 0u);               /* SO from the SAME read as the edge */
    }
    return rx;
}

/* Bit-level login re-sync (bare core). After power-on OR a reset, our 32-bit framing sits at a VARIABLE
 * bit offset from the GBA's words (the post-login/transition boundary offset is NOT a fixed constant — a
 * hardcoded consume can't track it). So clock bits ONE AT A TIME, presenting SI=0 (which IS the first
 * login response, tx=0x00000000, so the GBA stays happy), and lock onto the GBA's word boundary the
 * instant a login challenge appears aligned (low 16 bits == 0x494E — every early challenge ends in it).
 * Returns that first aligned challenge in *first_rx; false if none within the budget (clock dead/other). */
static IRAM_ATTR bool login_resync(uint32_t *first_rx)
{
    uint32_t sr = 0, total = 0;
    for (;;) {
        /* 2026-09-10 fix: this per-BIT "wait SC low" was using WORD_START_GIVEUP (~10s, sized for
         * the OUTER inter-command gap) instead of CLK_GIVEUP (~ms, sized for a single bit's gap
         * within an active clock stream). If the clock has any ordinary pause while this search is
         * mid-flight, that made a SINGLE bit iteration able to block for up to ~10 REAL SECONDS —
         * while the real GBA's own patience (confirmed from the decompiled pokefirered/librfu
         * source, librfu_stwi.c's STWI retry timers) is only ~50-470ms before it permanently gives
         * up ("Communication error", unrecoverable short of a power cycle). A core1 diagnostic
         * checkpoint (g_cp) proved this exact line was where core1 sat, every single time, across
         * 10 reproductions. CLK_GIVEUP makes this fail fast enough to actually have a chance of
         * beating the GBA's own deadline instead of guaranteeing we lose the race. The overall
         * "how long will we wait for a GBA to show up at all" patience is unaffected — that's the
         * `total > 40000` bit budget below, not this per-bit wait. */
        if (!link_wait_sc_low(CLK_GIVEUP)) return false;             /* wait SC low (debounced) */
        SI_LOW();                                                          /* present 0 = first login tx */
        /* 2026-09-10 fix, round 2: this "wait for rising edge" was ALSO unguarded, and turned out to
         * be the ACTUAL stuck point (checkpoint stayed at cmd_resync's identical line even after the
         * wait-SC-low fix above — confirmed the stall was here, not there). Unlike xfer32's hot path
         * (where a counter in the sample loop once shifted a real data bit by one position — see
         * CLK_GIVEUP's declaration comment — this is a SEARCH, not a trusted data sample: we're
         * hunting for an alignment over a large bounded budget (total>40000 below) and can afford to
         * fail one bit attempt and keep going, so a guard here is safe unlike in xfer32. */
        /* 2026-09-11: single-read sampling here too (see xfer32) — SO comes from the same
         * read that confirmed the rising edge, so a bus stall can't skew the sample. */
        bool hok;
        uint32_t v = link_wait_sc_high(CLK_GIVEUP, &hok);                  /* rising edge (debounced) */
        if (!hok) return false;
        sr = (sr << 1) | (LINK_SO(v) ? 1u : 0u);
        if ((sr & 0xFFFFu) == 0x494Eu) { *first_rx = sr; return true; }    /* aligned to a login challenge */
        /* 2026-09-10 fix, round 4: 40000 bits (up to ~100 REAL SECONDS worst case, now that both
         * inner waits are correctly CLK_GIVEUP-bounded per-bit) made this the new confirmed hang
         * site (checkpoint g_cp=0x20) once cmd_resync's own budget was fixed and correctly fell
         * back to a full relogin. But the caller (gba_spi_core1_entry) already retries a failed
         * login_resync() forever in its own loop — so shrinking THIS budget costs nothing in total
         * patience, it just returns control to that retry loop far more often. Matches cmd_resync's
         * budget for consistency; a real challenge stream realigns within ~32 bits regardless. */
        if (++total > 128u) return false;
    }
}

/* Fast LOCAL bit-level resync for the COMMAND phase (as opposed to login_resync's full-login search).
 * Hunts for a real command header (top 16 bits == 0x9966) one bit at a time, presenting SI=0 (matches
 * IDLE_WORD's low bits — the only 1 bit in IDLE_WORD is its very top bit, negligible over a short hunt).
 * Bounded to a SMALL budget (a few words' worth of bits): this is for recovering a single-bit slip
 * (e.g. observed immediately after discarding a GBA-echoed response: 0x996600bd echo -> discarded ->
 * next real header misread 1 bit late, 0xfffe929c = 0x494E<<1 with noise) — NOT for a genuine drop back
 * to login (that still falls through to a full re-login via login_resync, which hunts for 0x494E over a
 * much larger budget). Cheap+local beats "always pay for a full re-login", since re-login apparently
 * doesn't recover fast enough to keep the GBA from giving up ("can't find the adapter"). */
/* 2026-09-10 fix, round 3: 3000 bits is WAY beyond this function's own documented intent ("a few
 * words' worth of bits" ~ a few x 32). With both inner waits now correctly bounded at CLK_GIVEUP
 * (~1.25ms each), a genuinely dead clock makes EVERY one of those 3000 attempts time out, and the
 * worst case (3000 x up to ~2.5ms) is up to ~7.5 REAL SECONDS before this function admits defeat and
 * lets the outer loop fall back to a full relogin — far past the GBA's own ~470ms patience, and long
 * enough to keep re-tripping the core1 watchdog instead of ever reaching a clean recovery attempt.
 * 128 bits (~4 words) matches the doc comment and caps the worst case near ~320ms. */
#define CMD_RESYNC_BUDGET 128u
static IRAM_ATTR bool cmd_resync(uint32_t *first_hdr)
{
    uint32_t sr = 0, total = 0;
    for (;;) {
        /* 2026-09-10 fix: same WORD_START_GIVEUP -> CLK_GIVEUP fix as login_resync above — see its
         * comment. This was the confirmed hang site (checkpoint g_cp=0x10, 10/10 reproductions). */
        if (!link_wait_sc_low(CLK_GIVEUP)) return false;   /* wait SC low (debounced) */
        SI_LOW();
        /* 2026-09-10 fix, round 2 — see login_resync's matching comment: the unguarded "wait high"
         * was the actual confirmed stuck point, not the "wait low" fixed first. Safe to guard here
         * (search, not a trusted data sample). 2026-09-11: single-read sampling + debounce. */
        bool hok;
        uint32_t v = link_wait_sc_high(CLK_GIVEUP, &hok);
        if (!hok) return false;
        sr = (sr << 1) | (LINK_SO(v) ? 1u : 0u);
        if ((sr >> 16) == GBA_WAP_MAGIC) { *first_hdr = sr; return true; }
        if (++total > CMD_RESYNC_BUDGET) return false;
    }
}

/* External wrapper (used by the provider-shim gba_spi_task): xfer32 is lock-free, so take the lock
 * here. run_adapter does NOT use this — it calls xfer32 directly under its own session-wide lock. */
IRAM_ATTR bool gba_spi_exchange_raw(uint32_t tx_word, uint32_t *rx_word)
{
    if (!s_inited) return false;
    portENTER_CRITICAL(&s_mux);
    bool ok = false;
    uint32_t rx = xfer32(tx_word, &ok);
    portEXIT_CRITICAL(&s_mux);
    if (ok && rx_word) *rx_word = rx;
    return ok;
}

/* One full COMMAND-phase word in a SINGLE core-1 critical section. IRAM-resident + self-contained
 * (only register macros and esp_rom_delay_us, which is ROM/IRAM-safe) so NO flash-cache miss — e.g.
 * core-0 logging churning the shared cache — can stall us mid-word. Mirrors gpi.rs::send_ack:
 *   ack_ready_send (SI low) → 32-bit transfer (GBA clocks) → ack_recv (SI low, settle, SI high, wait
 *   the GBA's SO ack).
 * Crucially ack_ready_send lives INSIDE the critical section: previously SI_LOW ran before xfer32
 * took the lock, leaving a window where a FreeRTOS tick (1kHz) could land between "ready" and the
 * first sampled bit and slip our framing by a few bits. Returns false if the clock/ack stalls. */
/* One full COMMAND-phase word (gpi.rs::send_ack): ack_ready_send (SI low) → 32-bit transfer → ack_recv
 * (SI low, settle, SI high, wait the GBA's SO ack). LOCK-FREE (interrupts assumed already off; the
 * session-wide critical section is held by run_adapter). Ends with SI HIGH = idle between words, so the
 * next word's SI-low is a clean "ready" edge. wait-low budget spans a VBlank gap. */
static IRAM_ATTR bool xfer_word(uint32_t tx, uint32_t *rx_word)
{
    uint32_t rx = 0;
    SI_LOW();                                     /* ack_ready_send: "ready to exchange" */
    for (int i = 31; i >= 0; i--) {
        if (!link_wait_sc_low(WORD_START_GIVEUP)) { SI_HIGH(); return false; }  /* spans VBlank gap */
        if ((tx >> i) & 1u) SI_HIGH(); else SI_LOW();   /* present our bit while SC is low */
        /* 2026-09-11: single-read sampling — see xfer32's comment. SO is taken from the SAME
         * read that confirmed the rising edge (zero detect->sample skew, stall-proof). */
        uint32_t v = link_wait_sc_high(0, NULL);              /* rising edge — tight, debounced */
        rx = (rx << 1) | (LINK_SO(v) ? 1u : 0u);              /* sample from the same read */
    }
    /* ack_recv (gpi.rs::ack_recv). RP2040 does SI-low, delay(100 cyc)~0.8us @125MHz, THEN THREE SI-high
     * writes (not one) before polling — the repeated writes cost a little extra time on real silicon.
     * Widened 1us->2us as a cheap experiment against a suspected settle-time race at the S3's faster
     * clock (see esp32-s3-direct-gba-spi memory, 2026-09-10 reply-boundary entry). */
    SI_LOW();
    esp_rom_delay_us(2);
    SI_HIGH();
    /* 2026-09-11: debounced like the SC waits — a single async bounce on SO must not fake the ack. */
    uint32_t ag = 0, arun = 0;
    for (;;) {
        if (LINK_SO(LINK_IN())) { if (++arun >= LINK_DEBOUNCE) break; }
        else { arun = 0; if (++ag > WORD_START_GIVEUP) { return false; } }
    }
    if (rx_word) *rx_word = rx;
    return true;                                   /* SI left HIGH (idle) */
}

/* ---- CLOCK-MASTER path (trade data phase) --------------------------------------------------
 * WHY THIS EXISTS: the RFU clock role SWAPS. For discovery/connect the GBA drives SC and we are the
 * clock slave (everything above). But the GBA's link manager, on entering the data phase, issues a
 * clock-role change (0x25/0x27/0x35/0x37 + their A5/A7/B5/B7 acks) after which — per the decompiled
 * pokefirered librfu (librfu_intr.c sets msMode = AGB_CLK_SLAVE; librfu_stwi.c then returns
 * ERR_REQ_CMD_CLOCK_SLAVE from STWI_init without touching SIOCNT) — the GBA STOPS DRIVING SC
 * ENTIRELY and waits for the ADAPTER to clock the link. Without this path every trade data exchange
 * deadlocks: the GBA is silent forever and our slave loop waits for edges that never come.
 *
 * STATUS: implemented and compiled, but NOT yet wired into the command loop — it must land together
 * with the ASYNC_ACK fix (see gba_wap.c's ASYNC_ACK comment), because it is precisely those correct
 * acks that put the GBA into clock-slave mode. Enabling one without the other breaks the link. The
 * electrical direction handling below is the part that is safe to prove independently.
 *
 * DIRECTION SAFETY: SC is bidirectional now. We only assert our driver between acquire/release, and
 * we release in every exit path, so the GBA and this board can never both drive for longer than the
 * handoff instant (the series resistor on SC covers that window — see docs/18).
 *
 * TIMING: mirror-image of the slave loop's sampling contract (data presented while SC is low, both
 * sides sample on the rising edge). ~2 MHz link => 250 ns per half-period = 60 cycles @240 MHz. */
#ifndef GBA_SC_HALF_CYCLES
#define GBA_SC_HALF_CYCLES 60
#endif

static inline IRAM_ATTR void sc_delay(uint32_t cycles)
{
    uint32_t t0 = ccount();
    while ((uint32_t)(ccount() - t0) < cycles) { }
}

/* Take/release the clock. Idle level is HIGH (matching the GBA's own idle SC). */
IRAM_ATTR void gba_spi_clock_master_acquire(void)
{
    SC_DRIVE_HIGH();      /* preset the level BEFORE enabling the driver — no glitch on the line */
    SC_OUT_ENABLE();
}
IRAM_ATTR void gba_spi_clock_master_release(void)
{
    SC_DRIVE_HIGH();      /* leave it idle-high as we hand back */
    SC_OUT_DISABLE();     /* pad returns to input; the GBA owns SC again */
}

/* One 32-bit exchange with US driving SC. MSB-first, same bit order/sampling edge as the slave path:
 * SC low + present our SI bit -> settle -> SC high (both sides sample) -> hold. Returns the word the
 * GBA presented on SO. Caller must hold the clock (acquire/release around a whole transaction). */
IRAM_ATTR uint32_t gba_spi_master_xfer_word(uint32_t tx)
{
    uint32_t rx = 0;
    for (int i = 31; i >= 0; i--) {
        SC_DRIVE_LOW();
        if ((tx >> i) & 1u) SI_HIGH(); else SI_LOW();   /* present our bit while SC is low */
        sc_delay(GBA_SC_HALF_CYCLES);
        SC_DRIVE_HIGH();                                /* rising edge: the GBA samples SI here */
        uint32_t v = LINK_IN();                         /* ...and presents SO; sample it now */
        rx = (rx << 1) | (LINK_SO(v) ? 1u : 0u);
        sc_delay(GBA_SC_HALF_CYCLES);
    }
    SI_HIGH();                                          /* idle-high between words, as in slave mode */
    return rx;
}

/* One master word WITH the GBA's slave-side inter-word handshake (librfu_intr.c IntrSIO32, slave
 * path). The GBA's per-word ISR gates on OUR SI line: it enters with handshake_wait(0) — spins
 * until the GBA's SI *input* (our SI output) reads LOW — processes the word + reloads SIODATA32,
 * then handshake_wait(1) — spins until it reads HIGH — and only THEN re-enables its SIO for the
 * next transfer. So after every clocked word we must swing SI low, hold long enough for the ISR to
 * enter (worst-case interrupt latency, tens of us), then high, then settle before clocking again.
 * Clocking back-to-back words without this leaves the GBA mid-ISR while bits fly — the immediate
 * hard-crash observed on hardware. Delays are deliberately generous; the GBA self-paces inside
 * handshake_wait, so too-slow is safe and too-fast is fatal. */
#ifndef GBA_MASTER_HS_LOW_US
#define GBA_MASTER_HS_LOW_US  80
#endif
#ifndef GBA_MASTER_HS_HIGH_US
#define GBA_MASTER_HS_HIGH_US 30
#endif
static IRAM_ATTR uint32_t master_word_handshaken(uint32_t tx)
{
    uint32_t rx = gba_spi_master_xfer_word(tx);
    SI_LOW();  esp_rom_delay_us(GBA_MASTER_HS_LOW_US);   /* ISR entry gate: handshake_wait(0) */
    SI_HIGH(); esp_rom_delay_us(GBA_MASTER_HS_HIGH_US);  /* ISR exit gate:  handshake_wait(1) */
    return rx;
}

/* External wrapper (provider-shim). run_adapter calls xfer_word directly under its session-wide lock. */
IRAM_ATTR bool gba_spi_exchange_word(uint32_t tx_word, uint32_t *rx_word)
{
    if (!s_inited) return false;
    portENTER_CRITICAL(&s_mux);
    bool ok = xfer_word(tx_word, rx_word);
    portEXIT_CRITICAL(&s_mux);
    SI_HIGH();                        /* idle-high between words */
    return ok;
}

/* ---- full adapter: login (bare exchanges) then the WAP command loop via gba_wap ----
 * IRAM-resident (whole path) so no flash-cache miss can stall core1 between/within transactions —
 * this is the "hard-timing bare-loop" fix: the only flash call left is vTaskDelay on the idle (no-clock)
 * path, which is not timing-critical. */
IRAM_ATTR void gba_spi_run_adapter(struct gba_wap_io *io)
{
    gba_spi_init();
    /* Per-WORD critical sections (via the exchange_* wrappers) + the whole path in IRAM = the proven
     * build that reached the Union Room (~700 commands). We can NOT hold interrupts off across the whole
     * session (ESP-IDF needs core1 to service inter-processor/cache interrupts — a forever-off core1
     * wedges the system). So each word is interrupts-off (jitter-free sample), interrupts on between
     * words. The residual between-word tick jitter is addressed separately (see gba_spi_selftest). */
    for (;;) {
        /* ---- login: bare exchanges, walk the challenge/response to 0xB0BB8001 ---- */
        uint32_t tx = 0x00000000;
        bool done = false;
        while (!done) {
            uint32_t rx = 0;
            if (!gba_spi_exchange_raw(tx, &rx)) { vTaskDelay(1); continue; }
            g_gba_clock_seen = 1;
            g_login_rx[g_login_n & 15] = rx; g_login_n++;    /* record challenge stream (like RP2040 LOGIN_RX) */
            tx = gba_login_next(rx, &done);
        }
        g_gba_logins++;
        if (cap_seq == 0) cap_arm = 1;   /* capture ONCE: the window right after the FIRST login (CAP#1) */

        /* ---- command loop: CLEAN RP2040 logic — any non-0x9966 header = the GBA dropped back to login
         * (streams 0x494E challenges) → drop to login. A clock stall (GBA gone) also breaks to login. ---- */
        for (;;) {
            uint32_t hdr = 0;
            if (!gba_spi_exchange_word(IDLE_WORD, &hdr)) { g_gba_resets++; g_gba_last_reset = 1; g_reset_cmd = g_gba_cmds; break; }
            if (!gba_wap_header_valid(hdr)) { g_gba_resets++; g_gba_last_reset = hdr; g_reset_cmd = g_gba_cmds; break; }
            if (gba_wap_is_response(gba_wap_cmd(hdr))) { g_gba_skips++; continue; }   /* echoed reply — discard */
            uint8_t cmd = gba_wap_cmd(hdr), size = gba_wap_size(hdr);
            uint32_t data[64];
            if (size > 64) size = 64;
            for (uint8_t i = 0; i < size; i++) { uint32_t w = IDLE_WORD; gba_spi_exchange_word(IDLE_WORD, &w); data[i] = w; }

            uint32_t out[8]; gba_wap_action act;
            int rn = gba_wap_respond((gba_wap_io *)io, cmd, data, size, out, &act);
            g_gba_cmds++; g_gba_last_cmd = cmd;

            uint32_t dummy;
            if (act == GBA_WAP_ASYNC_ACK) {
                gba_spi_exchange_word(gba_wap_header(0xa8, 0), &dummy);
                gba_spi_exchange_word(IDLE_WORD, &dummy);
            } else {
                gba_spi_exchange_word(gba_wap_response_header(gba_wap_header(cmd, (uint8_t)rn)), &dummy);
                for (int i = 0; i < rn; i++) gba_spi_exchange_word(out[i], &dummy);
            }
        }
    }
}

/* ---- BARE-METAL core-1 adapter (Stage 2) ----
 * Runs on core 1 as a bare, non-preemptible loop (no FreeRTOS, no tick, no ISR — see gba_core1.c). So:
 *   - NO gba_spi_init here (done on core 0 before core 1 starts),
 *   - NO vTaskDelay (illegal without a scheduler) — we SPIN,
 *   - NO critical sections (core 1 has no interrupts to mask) → call the lock-free xfer32/xfer_word
 *     directly. This is the true hard-real-time path: zero jitter, ever.
 * The io (providers) is copied in from core 0 via gba_spi_set_core1_io() before start. */
static gba_wap_io s_core1_io;
void gba_spi_set_core1_io(const gba_wap_io *io) { s_core1_io = *io; }

void IRAM_ATTR gba_spi_core1_entry(void)
{
    gba_wap_io *io = &s_core1_io;
    SI_HIGH();   /* dedic build: core 1 owns SI's out channel — set the idle-high state here, since
                  * gba_spi_init (core 0) can no longer reach the pad. Harmless duplicate otherwise. */
    /* Hold SI high ~150ms before entering the adapter loop, so core 0's SI-readback self-check
     * (gba_relay_selftest) can observe the pad actually driven high. Without this hold the check
     * is invalid: with no GBA clocking, login_resync pulls SI LOW within microseconds of entry, so
     * pad=0 would be indistinguishable from a dead TX path. One-time (and per watchdog-restart)
     * ~150ms boot delay — the GBA retries detection continuously, so this costs nothing real. */
    { uint32_t t0 = ccount(); while ((uint32_t)(ccount() - t0) < 36000000u) { } }
    for (;;) {
        /* login (bare, lock-free) — START with a BIT-LEVEL RE-SYNC so we're word-aligned to the GBA no
         * matter what bit offset the (re)login starts at, then walk the challenge table aligned. */
        uint32_t rx = 0;
        g_cp = 0x20;
        if (!login_resync(&rx)) { g_core1_alive++; continue; }   /* no challenge yet — spin/retry */
        g_gba_clock_seen = 1;
        g_login_rx[g_login_n & 15] = rx; g_login_n++;
        bool done = false;
        uint32_t tx = gba_login_next(rx, &done);
        while (!done) {
            g_cp = 0x21;
            bool ok = false;
            rx = xfer32(tx, &ok);
            if (!ok) continue;                               /* no clock — spin (bare loop) */
            g_login_rx[g_login_n & 15] = rx; g_login_n++;
            tx = gba_login_next(rx, &done);
        }
        g_gba_logins++;
        if (cap_seq == 0) cap_arm = 1;

        /* command loop: a bad header means the GBA dropped back to login (post-login transition or a real
         * re-login) → break; the outer loop's login_resync re-aligns to the fresh login. No fixed consume/
         * skip — the resync handles the variable boundary offset. Clock stall also breaks to login. */
        uint32_t t_prev_cmd_end = ccount();   /* cadence baseline, reset fresh after each (re)login */
        for (;;) {
            uint32_t hdr = 0;
            g_cp = 1;
            bool hok = xfer_word(IDLE_WORD, &hdr);
            cmd_trace_push('H', IDLE_WORD, hdr);
            if (!hok) { g_gba_resets++; g_gba_last_reset = 1; g_reset_cmd = g_gba_cmds; break; }
            if (!gba_wap_header_valid(hdr)) {
                /* 2026-09-10 fix, round 5: a sniffer capture caught the ACTUAL failure mode live —
                 * a single bit misread on the wire (0x9966001a -> 0x9d66001a, ONE bit different),
                 * with the very NEXT natural 32-bit-aligned word already reading back correctly
                 * (0x9966001a). Framing was never lost, only one word's VALUE was wrong — but the
                 * old code treated every invalid header as a framing loss and jumped straight to
                 * cmd_resync's destructive bit-shifting search, which (confirmed in that same
                 * capture, twice) escalated all the way to a full relogin even though the link was
                 * fine one word later. A full relogin wipes the GBA's established session state
                 * (Setup/Connect/room membership) even though the bits were perfect — which is
                 * likely WHY the game shows "Communication error" despite the low-level link
                 * recovering flawlessly every time: from the game's perspective a relogin looks
                 * identical to the adapter being unplugged and replugged mid-session. Fix: try ONE
                 * more plain aligned word first (cheap, non-destructive, no bit-shifting) — if
                 * THAT'S a valid header, we've recovered from a single bad word for the cost of one
                 * extra transfer and nothing else. Only escalate to cmd_resync (and beyond that,
                 * full relogin) if the very next aligned word is ALSO invalid — i.e. framing is
                 * genuinely lost, not just one value. */
                uint32_t hdr2 = 0;
                g_cp = 0x08;
                bool hok2 = xfer_word(IDLE_WORD, &hdr2);
                cmd_trace_push('h', IDLE_WORD, hdr2);
                if (hok2 && gba_wap_header_valid(hdr2)) {
                    g_word_resyncs++;
                    hdr = hdr2;
                } else {
                    /* try a cheap LOCAL resync (single-bit-slip recovery) before paying for a full
                     * re-login — see cmd_resync()'s comment. Only escalate to a full re-login if
                     * that fails too (a genuine drop back to login, or the clock is actually gone). */
                    uint32_t resynced = 0;
                    g_cp = 0x10;
                    if (cmd_resync(&resynced)) { g_cmd_resyncs++; hdr = resynced; cmd_trace_push('X', 0, hdr); }
                    else { g_gba_resets++; g_gba_last_reset = hdr; g_reset_cmd = g_gba_cmds; break; }
                }
            }
            if (gba_wap_is_response(gba_wap_cmd(hdr))) {
                /* The GBA ECHOES our own reply header back as the very next word it clocks — this is
                 * real link behaviour (RP2040 routes.rs::handle_req: `if req.is_response() { return; }`),
                 * NOT a fresh command. Silently discard it and read the next word; do NOT reply, do NOT
                 * count it as a command, do NOT reset. Missing this check was the root cause of the
                 * reply->next-command desync (we were re-replying to our own echo). */
                g_gba_skips++;
                continue;
            }
            uint8_t cmd = gba_wap_cmd(hdr), size = gba_wap_size(hdr);
            if (!cmd_is_known(cmd)) anomaly_snapshot(hdr);   /* observe-only, see anomaly_snapshot() comment */
            uint32_t data[64];
            if (size > 64) size = 64;
            for (uint8_t i = 0; i < size; i++) {
                g_cp = 0x100 + i;
                uint32_t w = IDLE_WORD; xfer_word(IDLE_WORD, &w); data[i] = w;
                cmd_trace_push('D', IDLE_WORD, w);
            }

            g_cp = 2;
            uint32_t out[8]; gba_wap_action act;
            int rn = gba_wap_respond(io, cmd, data, size, out, &act);
            g_gba_cmds++; g_gba_last_cmd = cmd;

            uint32_t dummy;
            if (act == GBA_WAP_ASYNC_ACK) {
#if CONFIG_GBA_SPI_CLOCK_MASTER
                /* CORRECT protocol (decomp: librfu_intr.c) — ack the command the GBA actually sent
                 * (cmd|0x80 => A5/A7/B5/B7) with NO trailing word, then TAKE THE CLOCK: that ack is
                 * exactly what sets the GBA's msMode = AGB_CLK_SLAVE, after which it stops driving
                 * SC and waits for us. Enabled together, because either alone breaks the link. */
                uint32_t tx1 = gba_wap_response_header(gba_wap_header(cmd, 0));
                g_cp = 3;
                xfer_word(tx1, &dummy); cmd_trace_push('a', tx1, dummy);
                g_clock_master_swaps++;
#if CONFIG_GBA_SPI_PARENT_PUSH
                /* EXPERIMENTAL master data-phase push. DISPROVEN on hardware 2026-09-11: when we
                 * take the clock after the 0xA7 ack and clock our frame in, the GBA drives SO all-1s
                 * (hdrrx=0xffc00000, ackrx=0xffffffff) — i.e. it is NOT in a slave-receive-data
                 * exchange at that instant, so the frame lands on a dead line and the extra clocking
                 * crashes the GBA *faster* than doing nothing. Kept behind a flag (default OFF) as a
                 * record; do not enable without new evidence about WHEN the GBA actually listens. */
                uint32_t pf[20];
                int pfn = gba_relay_build_parent_frame(pf, (int)(sizeof(pf) / sizeof(pf[0])));
                gba_spi_clock_master_acquire();
                g_cp = 4;
                esp_rom_delay_us(150);
                uint32_t mrx = master_word_handshaken(gba_wap_header(0x28, (uint8_t)pfn));
                g_parent_hdr_rx = mrx;
                cmd_trace_push('M', (uint32_t)pfn, mrx);
                for (int i = 0; i < pfn; i++) (void)master_word_handshaken(pf[i]);
                uint32_t ack = master_word_handshaken(0x80000000u);
                g_parent_ack_last = ack;
                cmd_trace_push('K', 0x80000000u, ack);
                gba_spi_clock_master_release();
                g_parent_frames++;
                if ((ack & 0xFFFFFFFFu) == 0x996600A8u) g_parent_acks++;
#else
                /* BEST-KNOWN behaviour (got furthest on hardware: GBA played the trade-room entry
                 * animation before dropping). After the correct 0xA7 ack the GBA is clock-slave;
                 * clock exactly ONE idle word so its slave-side DMA/handshake completes, then hand
                 * the clock straight back and let the GBA drive again. Touch the clock as little as
                 * possible — every extra master word we push destabilises it. */
                gba_spi_clock_master_acquire();
                g_cp = 4;
                uint32_t mrx = gba_spi_master_xfer_word(IDLE_WORD);
                g_parent_hdr_rx = mrx;
                cmd_trace_push('i', IDLE_WORD, mrx);
                gba_spi_clock_master_release();
#endif
#else
                uint32_t tx1 = gba_wap_header(0xa8, 0);
                g_cp = 3;
                xfer_word(tx1, &dummy); cmd_trace_push('a', tx1, dummy);
                g_cp = 4;
                xfer_word(IDLE_WORD, &dummy); cmd_trace_push('i', IDLE_WORD, dummy);
#endif
            } else {
                uint32_t tx1 = gba_wap_response_header(gba_wap_header(cmd, (uint8_t)rn));
                g_cp = 5;
                xfer_word(tx1, &dummy); cmd_trace_push('R', tx1, dummy);
                for (int i = 0; i < rn; i++) {
                    g_cp = 0x300 + i;
                    xfer_word(out[i], &dummy); cmd_trace_push('o', out[i], dummy);
                }
            }
            g_cp = 6;
            /* CADENCE: cycles since the previous command's response finished — measured only here,
             * between whole commands, so it can't perturb any bit-sampling instant (see comment at the
             * declaration). Lets us see whether the gap before the comm-error was abnormally long. */
            uint32_t t_now = ccount();
            uint32_t delta = t_now - t_prev_cmd_end;
            t_prev_cmd_end = t_now;
            g_cmd_cadence_cyc[g_cmd_cadence_n & (CMD_CADENCE_N - 1)] = delta;
            g_cmd_cadence_n++;
            if (delta > g_cmd_cadence_max_cyc) { g_cmd_cadence_max_cyc = delta; g_cmd_cadence_max_idx = g_gba_cmds; }
        }
    }
}

/* ---- unused provider-task shim kept for the header API ---- */
static gba_spi_provider_t s_provider;
static void *s_provider_ctx;
static void gba_spi_task(void *arg)
{
    (void)arg;
    uint32_t tx = IDLE_WORD;
    for (;;) {
        uint32_t rx = 0;
        if (gba_spi_exchange_word(tx, &rx)) tx = s_provider ? s_provider(rx, s_provider_ctx) : IDLE_WORD;
        else vTaskDelay(1);
    }
}
void gba_spi_start(gba_spi_provider_t provider, void *ctx)
{
    gba_spi_init();
    s_provider = provider; s_provider_ctx = ctx;
    xTaskCreatePinnedToCore(gba_spi_task, "gba_spi", 4096, NULL, configMAX_PRIORITIES - 2, NULL, 1);
}

/* Stub LDN-side providers: no peer, no slots — the adapter answers every command locally so the GBA
 * enters the Union Room SOLO and stays (the RP2040's proven "no relayed peer = no peers" behaviour). */
static IRAM_ATTR int      stub_get_peer(uint32_t o[7], void *c) { (void)o; (void)c; return 0; }
static IRAM_ATTR uint32_t stub_peer_id(void *c)                 { (void)c; return 0; }
static IRAM_ATTR int      stub_take_slot(uint32_t *o, int m, void *c) { (void)o; (void)m; (void)c; return 0; }
static IRAM_ATTR void     stub_put_slot(const uint32_t *d, int l, void *c) { (void)d; (void)l; (void)c; }

static void adapter_task(void *arg) { gba_spi_run_adapter((struct gba_wap_io *)arg); }

/* core-0 passive sniffer: wait until core 1 signals login done (cap_arm), then oversample the raw bus
 * (READ-ONLY — drives nothing) while core 1 actively runs the command handshake, so we capture the real
 * post-login window. Reconstruct words from true SC rising edges and scan for 0x9966. One-shot. */
static void cap_task(void *arg)
{
    (void)arg;
    for (;;) {
        while (!cap_arm) vTaskDelay(1);
        cap_arm = 0;
        /* EDGE TRIGGER (interrupts ON so a long wait can't trip a watchdog): wait for the first SC rising
         * edge, else we capture a quiet gap. Frames are VBlank-paced so activity can be delayed. */
        int tlast = (REG_READ(GPIO_IN_REG) >> GBA_SC_PIN) & 1;
        for (uint32_t guard = 0; guard < 240000000u; guard++) {
            int sc = (REG_READ(GPIO_IN_REG) >> GBA_SC_PIN) & 1;
            if (sc && !tlast) break;                        /* first rising edge → trigger */
            tlast = sc;
        }
        portENTER_CRITICAL(&cap_mux);                       /* interrupts off on CORE 0 only (own lock) */
        for (int i = 0; i < CAP_RAW; i++) cap_raw[i] = (uint8_t)(REG_READ(GPIO_IN_REG) >> 8);
        portEXIT_CRITICAL(&cap_mux);
        /* reconstruct bits at true framing: SC=GPIO12 -> (>>8)>>4 ; SO=GPIO11 -> (>>8)>>3 */
        int nb = 0, last = 0, edges = 0;
        for (int i = 0; i < CAP_RAW && nb < (int)sizeof(cap_bits); i++) {
            int sc = (cap_raw[i] >> 4) & 1, so = (cap_raw[i] >> 3) & 1;
            if (sc && !last) { cap_bits[nb++] = (uint8_t)so; edges++; }
            last = sc;
        }
        cap_nbits = nb; cap_edges = edges;
        int o9966 = -1, o494e = -1; uint32_t w9966 = 0, w494e = 0;
        for (int p = 0; p + 16 <= nb && (o9966 < 0 || o494e < 0); p++) {
            uint32_t v = 0; for (int k = 0; k < 16; k++) v = (v << 1) | cap_bits[p + k];
            if (v == 0x9966u && o9966 < 0) { o9966 = p; for (int k = 0; k < 32 && p+k < nb; k++) w9966 = (w9966<<1)|cap_bits[p+k]; }
            if (v == 0x494eu && o494e < 0) { o494e = p; for (int k = 0; k < 32 && p+k < nb; k++) w494e = (w494e<<1)|cap_bits[p+k]; }
        }
        uint32_t w0 = 0; for (int k = 0; k < 32 && k < nb; k++) w0 = (w0 << 1) | cap_bits[k];
        for (int wi = 0; wi < 10; wi++) {                   /* first 10 words at 0-offset framing */
            uint32_t w = 0; for (int k = 0; k < 32; k++) { int b = wi*32+k; w = (w<<1) | (b<nb ? cap_bits[b] : 0); }
            cap_words[wi] = w;
        }
        cap_off9966 = o9966; cap_w9966 = w9966; cap_off494e = o494e; cap_w494e = w494e; cap_word0 = w0;
        cap_seq++;                                          /* signal the monitor a new capture is ready */
    }
}

/* GATE 2→4 bring-up self-test: the FULL adapter (login + command loop) with stub providers, running
 * on CORE 1 with ZERO logging in the loop (RP2040 design). A monitor here (core 0) prints counters,
 * so tracing never delays the bit-bang — the fix for the "login complete" log desyncing the 1st word. */
void gba_spi_selftest(void)
{
    gba_spi_init();
    static gba_wap_io io;
    memset(&io, 0, sizeof(io));
    io.get_peer = stub_get_peer; io.peer_id = stub_peer_id;
    io.take_slot = stub_take_slot; io.put_slot = stub_put_slot;
    (void)adapter_task;
    /* STAGE 2: the GBA adapter runs on the BARE core 1 (unicore) — true hard real-time, no FreeRTOS/tick/
     * ISR jitter. Copy the (stub) providers in for core 1, then wake it into gba_spi_core1_entry. */
    gba_spi_set_core1_io(&io);
    ESP_LOGI(TAG, "SELFTEST STAGE 2: GBA adapter on BARE core 1 (unicore, hard real-time). Enter the "
                  "Union Room; watch cmds= climb and STAY (no jitter kicks).");
    gba_core1_start();
    /* core-0 passive sniffer: captures the post-login bus at TRUE framing (armed by core 1 after login)
     * so we can see exactly what the GBA clocks at the login→command boundary on the bare core. */
    xTaskCreatePinnedToCore(cap_task, "gba_cap", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 0);
    /* core-0 monitor: print ONLY when a counter changes, so the console (flash/UART) stays quiet while
     * the GBA is actively clocking commands — no cache churn to race core-1's between-word flash code. */
    uint32_t p_clk = 0, p_logins = 0, p_cmds = 0, p_resets = 0, p_last_cmd = 0, p_last_reset = 0, p_skips = 0, p_resyncs = 0;
    int p_cap_seq = 0;
    uint8_t p_anomaly_seq = 0;
    uint32_t p_alive = 0, alive_ticks = 0;
    /* WIRING/SIGNAL-INTEGRITY HEALTH (objective bit-error-rate style metric): every ~5s, compare how
     * many resets/resyncs/anomalies happened against how many commands were actually served. A "clean"
     * link should show resyncs/anomalies rarely-to-never; if the rate stays high (or gets worse) across
     * a rewiring attempt, that's direct evidence the wiring is still the problem — no GBA test needed to
     * see the trend, and no logic analyzer needed to get SOME quantified signal beyond "it crashed". */
    uint32_t health_ticks = 0;
    uint32_t h_prev_cmds = 0, h_prev_resets = 0, h_prev_resyncs = 0, h_prev_anom = 0, h_prev_skips = 0;
    int64_t h_start_us = esp_timer_get_time();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(200));
        /* STAGE 1: report the bare core-1 proof-of-life every ~1s (climbing = bare core runs). */
        if (++alive_ticks >= 5) {
            alive_ticks = 0;
            /* Live SC/SO/SI pin snapshot — read-only from CORE 0, can NEVER perturb core1's tight
             * loop (unlike a counter inside the loop itself, which the login lesson taught us shifts
             * the sample point). If core1 ever wedges post-room-entry, this tells us in one glance
             * whether the GBA's clock genuinely went static (real GBA-side crash/freeze — SC frozen
             * high or low) or is still toggling (meaning OUR code is stuck despite a live clock). */
            uint32_t pins = REG_READ(GPIO_IN_REG);
            ESP_LOGI(TAG, "core1_alive=%u (%s)  PINS sc=%d so=%d si=%d",
                     (unsigned)g_core1_alive, g_core1_alive != p_alive ? "RUNNING ✓" : "stalled ✗",
                     (int)((pins >> GBA_SC_PIN) & 1), (int)((pins >> GBA_SO_PIN) & 1),
                     (int)((pins >> GBA_SI_PIN) & 1));
            p_alive = g_core1_alive;
        }
        if (++health_ticks >= 25) {   /* ~5s at the 200ms loop period */
            health_ticks = 0;
            uint32_t cmds = g_gba_cmds, resets = g_gba_resets, resyncs = g_cmd_resyncs;
            uint32_t anom = g_anomaly_seq, skips = g_gba_skips;
            uint32_t d_cmds = cmds - h_prev_cmds, d_resets = resets - h_prev_resets;
            uint32_t d_resyncs = resyncs - h_prev_resyncs, d_anom = anom - h_prev_anom;
            uint32_t d_skips = skips - h_prev_skips;
            int64_t elapsed_us = esp_timer_get_time() - h_start_us;
            /* per-1000-commands rates (cumulative since boot) — the number to compare across a rewiring
             * attempt. 0 across a long run with real traffic = clean link. Climbing = still noisy. */
            uint32_t resets_per_1k   = cmds ? (uint32_t)((uint64_t)resets * 1000 / cmds) : 0;
            uint32_t resyncs_per_1k  = cmds ? (uint32_t)((uint64_t)resyncs * 1000 / cmds) : 0;
            uint32_t anom_per_1k     = cmds ? (uint32_t)((uint64_t)anom * 1000 / cmds) : 0;
            ESP_LOGI(TAG, "HEALTH t=%llds cmds=%u(+%u/5s) | cumulative per-1000-cmds: resets=%u resyncs=%u anomalies=%u | last-5s: resets=%u resyncs=%u anomalies=%u skips=%u",
                     (long long)(elapsed_us / 1000000), (unsigned)cmds, (unsigned)d_cmds,
                     (unsigned)resets_per_1k, (unsigned)resyncs_per_1k, (unsigned)anom_per_1k,
                     (unsigned)d_resets, (unsigned)d_resyncs, (unsigned)d_anom, (unsigned)d_skips);
            h_prev_cmds = cmds; h_prev_resets = resets; h_prev_resyncs = resyncs;
            h_prev_anom = anom; h_prev_skips = skips;
        }
        if (cap_seq != p_cap_seq) {                     /* a new sniffer capture is ready — print it */
            p_cap_seq = cap_seq;
            ESP_LOGI(TAG, "CAP#%d edges=%d nbits=%d  9966@%d(0x%08x)  494E@%d(0x%08x)",
                     cap_seq, cap_edges, cap_nbits,
                     cap_off9966, (unsigned)cap_w9966, cap_off494e, (unsigned)cap_w494e);
            ESP_LOGI(TAG, "CAP#%d words: %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x", cap_seq,
                     (unsigned)cap_words[0],(unsigned)cap_words[1],(unsigned)cap_words[2],(unsigned)cap_words[3],
                     (unsigned)cap_words[4],(unsigned)cap_words[5],(unsigned)cap_words[6],(unsigned)cap_words[7],
                     (unsigned)cap_words[8],(unsigned)cap_words[9]);
        }
        if (g_anomaly_seq != p_anomaly_seq) {   /* an unrecognized-but-magic-valid command was seen — print
                                                  * the FROZEN snapshot now, independent of the lagging MON
                                                  * counter comparison below (that's what lost the context
                                                  * last time: by the next poll, dozens more normal
                                                  * exchanges had already pushed it out of cmd_trace). */
            p_anomaly_seq = g_anomaly_seq;
            ESP_LOGI(TAG, "ANOMALY #%u at cmd_idx=%u hdr=0x%08x (cmd=0x%02x not in known whitelist)",
                     g_anomaly_seq, (unsigned)g_anomaly_cmd_idx, (unsigned)g_anomaly_hdr,
                     (unsigned)gba_wap_cmd(g_anomaly_hdr));
            char line[1600]; int off = 0;
            off += snprintf(line + off, sizeof(line) - off, "ANOMALY snapshot (oldest first):");
            for (int k = 0; k < ANOMALY_SNAP_N && off < (int)sizeof(line) - 24; k++) {
                off += snprintf(line + off, sizeof(line) - off, " %c[%08x/%08x]",
                                 g_anomaly_snap_tag[k], (unsigned)g_anomaly_snap_tx[k],
                                 (unsigned)g_anomaly_snap_rx[k]);
            }
            ESP_LOGI(TAG, "%s", line);
        }
        if (g_gba_clock_seen == p_clk && g_gba_logins == p_logins && g_gba_cmds == p_cmds &&
            g_gba_resets == p_resets && g_gba_last_cmd == p_last_cmd && g_gba_last_reset == p_last_reset &&
            g_gba_skips == p_skips && g_cmd_resyncs == p_resyncs)
            continue;
        p_clk = g_gba_clock_seen; p_logins = g_gba_logins; p_cmds = g_gba_cmds;
        p_resets = g_gba_resets; p_last_cmd = g_gba_last_cmd; p_last_reset = g_gba_last_reset;
        p_skips = g_gba_skips; p_resyncs = g_cmd_resyncs;
        ESP_LOGI(TAG, "MON clk=%u logins=%u cmds=%u last_cmd=0x%02x skips=%u resyncs=%u resets=%u last=0x%08x",
                 (unsigned)g_gba_clock_seen, (unsigned)g_gba_logins, (unsigned)g_gba_cmds,
                 (unsigned)g_gba_last_cmd, (unsigned)g_gba_skips, (unsigned)g_cmd_resyncs,
                 (unsigned)g_gba_resets, (unsigned)g_gba_last_reset);
        ESP_LOGI(TAG, "JITTER max=%u cyc  over%u=%u  jit_lastcmd=%u  reset_cmd=%u  (dropout at reset_cmd; if it == jit_lastcmd, an ISR caused it)",
                 (unsigned)g_jit_max_cyc, (unsigned)JIT_THRESH_CYC, (unsigned)g_jit_cnt,
                 (unsigned)g_jit_lastcmd, (unsigned)g_reset_cmd);
        /* CADENCE: worst gap between two whole commands seen so far (frozen/monotonic, so it survives
         * even if the GBA drops the link right after) + the most recent raw gaps to see the trend. */
        {
            uint32_t n = g_cmd_cadence_n;
            uint32_t start = (n > CMD_CADENCE_N) ? (n - CMD_CADENCE_N) : 0;
            char cline[700]; int coff = 0;
            coff += snprintf(cline + coff, sizeof(cline) - coff,
                              "CADENCE max=%u cyc (%u us) at cmd#%u  last:",
                              (unsigned)g_cmd_cadence_max_cyc, (unsigned)(g_cmd_cadence_max_cyc / 240),
                              (unsigned)g_cmd_cadence_max_idx);
            for (uint32_t k = start; k < n && coff < (int)sizeof(cline) - 16; k++) {
                coff += snprintf(cline + coff, sizeof(cline) - coff, " %u",
                                  (unsigned)(g_cmd_cadence_cyc[k & (CMD_CADENCE_N - 1)] / 240));
            }
            ESP_LOGI(TAG, "%s us", cline);
        }
        ESP_LOGI(TAG, "LOGIN_RX n=%u last16: %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x",
                 (unsigned)g_login_n,
                 (unsigned)g_login_rx[0],(unsigned)g_login_rx[1],(unsigned)g_login_rx[2],(unsigned)g_login_rx[3],
                 (unsigned)g_login_rx[4],(unsigned)g_login_rx[5],(unsigned)g_login_rx[6],(unsigned)g_login_rx[7],
                 (unsigned)g_login_rx[8],(unsigned)g_login_rx[9],(unsigned)g_login_rx[10],(unsigned)g_login_rx[11]);
        /* CMD_TRACE: every xfer_word tx/rx pair since boot, in order — shows EXACTLY which exchange the
         * reply<->next-header corruption enters at (tag: H=hdr-read D=data-read R=resp-hdr-write
         * o=resp-data-write a/i=async-ack). Print the last CMD_TRACE_N entries oldest-first. */
        {
            uint32_t n = g_cmd_trace_n;
            uint32_t start = (n > CMD_TRACE_N) ? (n - CMD_TRACE_N) : 0;
            char line[1600]; int off = 0;
            off += snprintf(line + off, sizeof(line) - off, "TRACE n=%u:", (unsigned)n);
            for (uint32_t k = start; k < n && off < (int)sizeof(line) - 24; k++) {
                uint32_t i = k & (CMD_TRACE_N - 1);
                off += snprintf(line + off, sizeof(line) - off, " %c[%08x/%08x]",
                                 g_cmd_trace_tag[i], (unsigned)g_cmd_trace_tx[i], (unsigned)g_cmd_trace_rx[i]);
            }
            ESP_LOGI(TAG, "%s", line);
        }
    }
}
