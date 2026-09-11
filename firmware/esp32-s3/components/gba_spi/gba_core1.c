/* gba_core1.c — bring up core 1 (APP CPU) as a BARE-METAL, non-preemptible loop, outside FreeRTOS.
 *
 * Why: the GBA link bit-bang needs hard-real-time timing. As a FreeRTOS task (even pinned + IRAM), a
 * rare high-priority ISR (cache/IPC coordination) fires mid-word and corrupts a sample (~1-in-700).
 * ESP-IDF can't mask those without wedging. The fix (r/rust "bare-metal on ESP32-S3 second core" +
 * tingouw.com/blog): build with CONFIG_FREERTOS_UNICORE=y so FreeRTOS/tick/scheduler/IPC live ONLY on
 * core 0, leaving core 1 empty; then manually wake core 1 and run our own bare loop on it. Because
 * core 0 doesn't know core 1 exists, it never IPC-stalls it during flash ops, and our loop is 100% IRAM
 * so a disabled cache can't stall it. Core 1 becomes the RP2040's bare core.
 *
 * STAGE 1 (this file, for bring-up validation): core 1 just increments a shared counter forever. If the
 * core-0 monitor sees g_core1_alive climbing, the bare-core bring-up works and we move the GBA adapter
 * onto it (Stage 2). Inter-core comms are plain shared internal SRAM (not per-core cached on the S3).
 */
#include <stdint.h>
#include "esp_attr.h"
#include "esp_cpu.h"
#include "rom/ets_sys.h"          /* ets_set_appcpu_boot_addr */
#include "hal/cpu_utility_ll.h"   /* cpu_utility_ll_enable_clock_and_reset_app_cpu */

/* Shared state — both cores touch this (internal SRAM, volatile; add barriers/atomics for real data). */
volatile uint32_t g_core1_alive;   /* Stage 1: core 1 bumps this every loop → proof of life */

/* Core 1's own stack (global so the .S trampoline can reference it; in .bss so the heap never claims it).
 * 8 KB is ample for the bare loop. `core1_stack_top` is used by the trampoline as the initial SP. */
uint8_t core1_stack[8192] __attribute__((aligned(16)));

/* The bare loop that runs on core 1. NOReturn, IRAM (must not depend on the cache).
 * Called (window-call) from the .S trampoline gba_core1_boot after it sets SP to core1_stack top.
 * Stage 2: run the GBA adapter bit-bang bare (gba_spi_core1_entry, in gba_spi.c) — it bumps
 * g_core1_alive on idle spins so the core-0 monitor still sees proof-of-life. */
extern void gba_spi_core1_entry(void);   /* the bare adapter loop (never returns) */

/* 2026-09-11: selectable core-1 body. Defaults to the real adapter loop; the GPIO bus-stall probe
 * (gba_busprobe.c) repoints it BEFORE gba_core1_start() so the same proven trampoline/vector-table
 * bring-up can host a different bare workload. Read exactly once at core-1 boot — no hot-path cost. */
void (*g_core1_entry)(void) = gba_spi_core1_entry;

/* docs/16, 2026-09-10: core 1 boots via our own trampoline (gba_core1_boot.S), never through
 * ESP-IDF's call_start_cpu1 — which means it NEVER runs ESP-IDF's per-core init_cpu() and, in
 * particular, never gets esp_cpu_intr_set_ivt_addr(&_vector_table) called for it. Core 1's
 * VECBASE is left at whatever the ROM app-cpu bring-up path set (a ROM vector table, not the
 * app's). Windowed calls still work (window overflow/underflow ARE serviced, or nothing would
 * run at all), but any OTHER exception (bad load/store, illegal instruction, etc.) vectors into
 * ROM code with no knowledge of our app, and silently halts core 1 with no report — the leading
 * theory for the sniffer-confirmed deterministic crash. Fix: install the SAME vector table
 * ESP-IDF's own app-cpu bring-up would have installed, so a real fault instead lands in the real
 * ESP-IDF panic handler (Guru Meditation dump: EXCCAUSE/EXCVADDR/EPC1/backtrace over the console
 * UART) before halting. This is a one-line mirror of cpu_start.c's own init_cpu(), nothing new
 * or hand-rolled. */
extern int _vector_table;

void IRAM_ATTR __attribute__((noreturn)) gba_core1_main(void)
{
    esp_cpu_intr_set_ivt_addr(&_vector_table);  /* MUST be first: real exception handling from here on */
    g_core1_alive = 1;                    /* mark reached-C before entering the adapter */
    g_core1_entry();                      /* adapter loop by default; probe body if repointed */
    for (;;) { }                          /* unreachable */
}

/* The APP-CPU boot entry (Xtensa asm trampoline). Defined in gba_core1_boot.S. */
extern void gba_core1_boot(void);

/* Called from core 0 (in UNICORE build) to wake core 1 into the bare loop. Mirrors ESP-IDF's own
 * start_other_core(): unstall → enable clock+reset the APP CPU → set its boot address. */
void gba_core1_start(void)
{
    esp_cpu_unstall(1);
    cpu_utility_ll_enable_clock_and_reset_app_cpu();
    ets_set_appcpu_boot_addr((uint32_t)gba_core1_boot);
}
