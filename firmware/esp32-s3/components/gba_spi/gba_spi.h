/* gba_spi — ESP32-S3 hardware-SPI-slave front-end that emulates the GBA Wireless Adapter, so a real
 * GBA link port can be wired STRAIGHT into the S3 (no RP2040, no UART bridge). The GBA is the SPI
 * master (drives SC), so the S3 SPI-slave silicon shifts each 32-bit word immune to Wi-Fi jitter;
 * only the between-word ready-handshake (§3 of GBA_SPI_SLAVE_DESIGN.md) is software-timed.
 *
 * STATUS: skeleton for the hardware go/no-go test (GBA_SPI_SLAVE_DESIGN.md §8). The between-word
 * handshake + the SI GPIO<->peripheral handoff are the parts that need real-GBA validation/tuning.
 *
 * Transport is GPIO BIT-BANG (the GBA drives the clock; there is no chip-select on a GBA link, so a
 * hardware SPI-slave can't frame words — see the design doc). Pins: SC=GPIO12 (clock in), SO=GPIO11
 * (GBA->us, in), SI=GPIO13 (us->GBA, out), SD=GPIO14 (reset/ready). NO CS wire — GPIO10 is unused. */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One exchanged 32-bit word: rx = what the GBA sent us, and we must have supplied tx (what the GBA
 * reads). Returned by the provider callback below. */
typedef uint32_t (*gba_spi_provider_t)(uint32_t rx_from_gba, void *ctx);

/* Configure the SPI-slave peripheral + the handshake GPIOs. Safe to call once at boot. */
void gba_spi_init(void);

/* Perform ONE full adapter-side word cycle in the GBA-initiated direction: run the between-word
 * ready-handshake, then let the SPI-slave shift 32 bits (we present *tx_word, we capture the GBA's
 * word into *rx_word). Returns true on success, false if the ~800us handshake window elapsed. */
bool gba_spi_exchange_word(uint32_t tx_word, uint32_t *rx_word);

/* ONE 32-bit exchange with NO handshake — for the login phase (GBA clocks back-to-back). */
bool gba_spi_exchange_raw(uint32_t tx_word, uint32_t *rx_word);

/* Full adapter: login, then the WAP command loop driven by gba_wap (peer/slot providers in `io`).
 * Never returns. The complete intended structure; per-word handshake timing needs HW validation. */
struct gba_wap_io;
void gba_spi_run_adapter(struct gba_wap_io *io);

/* Bare-metal core-1 bring-up (gba_core1.c / gba_spi.c). Call gba_spi_set_core1_io() with the real
 * providers BEFORE gba_core1_start() — core1 copies the struct by value at that point and never
 * looks at the original again (see gba_relay.h for why backend state must live behind `ctx`, not
 * struct fields). Both were previously file-static-only in gba_spi.c; exposed here so a separate
 * module (e.g. gba_relay_selftest.c) can wire a real provider without living inside gba_spi.c. */
void gba_spi_set_core1_io(const struct gba_wap_io *io);
void gba_core1_start(void);

/* Start the core1-pinned service task. `provider` is called with each received word and returns the
 * next word to present (e.g. wired to the WAP/RFU adapter FSM). ctx is passed through. */
void gba_spi_start(gba_spi_provider_t provider, void *ctx);

/* GATE 1/2 bring-up self-test (design doc §8): logs the first words the GBA clocks in, so SPI mode
 * (Mode 3 vs 0) and wiring can be confirmed before the full protocol port. Never returns. */
void gba_spi_selftest(void);

/* Phase 1 of docs/16-single-chip-trade-plan.md (gba-switch-bridge repo): the bare-metal core1 GBA
 * adapter wired to the REAL gba_relay backend (gba_relay.c) instead of the always-empty stub, with
 * a synthetic Switch peer seeded so the discover->connect->slot-exchange flow can be validated
 * against a real cartridge in the TRADE counter — WITHOUT real Wi-Fi yet (that's Phase 2/3). Never
 * returns. Leave CONFIG_GBA_SPI_TRADE_RELAY_SELFTEST off for normal builds. */
void gba_spi_trade_relay_selftest(void);

/* GPIO bus-stall probe (gba_busprobe.c, 2026-09-11): core 1 times back-to-back GPIO_IN_REG reads
 * while core 0 cycles load phases (quiet/uart/flash/psram/all) — measures whether shared-bus
 * arbitration can stall a core-1 GPIO read past the 250ns half-bit budget (docs/17 §7). Runs
 * instead of the LDN app when CONFIG_GBA_SPI_BUS_PROBE is set. Never returns. */
void gba_spi_bus_probe(void);

/* ---- diagnostic globals (gba_spi.c) shared with any monitor that wants the same rich telemetry
 * this session's Union-Room debugging already built — the anomaly-snapshot/trace/cadence machinery
 * runs unconditionally inside gba_spi_core1_entry() regardless of which gba_wap_io backend is
 * wired in, so gba_relay_trade_relay_selftest()'s monitor gets it "for free" just by printing
 * these, rather than needing its own copy of the detection logic. See gba_spi.c for how each is
 * populated; sizes must match the #defines there exactly (checked at compile time by array size on
 * both sides — a mismatch here would be a real bug, not just a cosmetic one). */
#define GBA_SPI_CMD_TRACE_N   128
#define GBA_SPI_ANOMALY_SNAP_N 64
#define GBA_SPI_CMD_CADENCE_N  64

extern volatile uint32_t g_gba_clock_seen, g_gba_logins, g_gba_cmds, g_gba_resets;
extern volatile uint32_t g_gba_last_cmd, g_gba_last_reset, g_gba_skips, g_cmd_resyncs;
extern volatile uint32_t g_word_resyncs;
extern volatile uint32_t g_core1_alive;
extern volatile uint32_t g_cp;   /* checkpoint — see gba_spi.c's declaration comment */

extern volatile uint8_t  g_cmd_trace_tag[GBA_SPI_CMD_TRACE_N];
extern volatile uint32_t g_cmd_trace_tx[GBA_SPI_CMD_TRACE_N], g_cmd_trace_rx[GBA_SPI_CMD_TRACE_N];
extern volatile uint32_t g_cmd_trace_n;

extern volatile uint8_t  g_anomaly_seq;
extern volatile uint32_t g_anomaly_cmd_idx, g_anomaly_hdr;
extern volatile uint8_t  g_anomaly_snap_tag[GBA_SPI_ANOMALY_SNAP_N];
extern volatile uint32_t g_anomaly_snap_tx[GBA_SPI_ANOMALY_SNAP_N], g_anomaly_snap_rx[GBA_SPI_ANOMALY_SNAP_N];

extern volatile uint32_t g_cmd_cadence_cyc[GBA_SPI_CMD_CADENCE_N];
extern volatile uint32_t g_cmd_cadence_n, g_cmd_cadence_max_cyc, g_cmd_cadence_max_idx;

#ifdef __cplusplus
}
#endif
