/* gba_busprobe.c — GPIO-read bus-stall probe (2026-09-11, docs/17 §7 follow-up).
 *
 * QUESTION UNDER TEST: on the S3, every GPIO_IN_REG read from bare-metal core 1 is an APB
 * round-trip across the bus matrix SHARED with core 0 (UART logging, flash/cache ops, PSRAM
 * traffic). The bit-bang has a 250ns half-bit budget @2MHz. Do core-0 activities ever stall a
 * core-1 GPIO read long enough (>~60 CPU cycles @240MHz) to push a sample past that budget?
 * If yes, that is a corruption mechanism the RP2040 (1-cycle private SIO reads) physically
 * cannot have — a direct, measured answer to docs/17 §7's open question, on-device, no logic
 * analyzer needed.
 *
 * METHOD: core 1 runs a bare IRAM loop timing every consecutive GPIO_IN_REG read with ccount
 * (min/max delta + absolute-threshold buckets). Core 0 cycles through 5s load phases:
 *   quiet -> uart (log spam) -> flash (esp_flash_read churn) -> psram (memcpy churn) -> all
 * printing the per-phase stats each second. Attribution falls straight out: whichever phase's
 * max/bucket counts jump is the bus master that steals core 1's margin.
 *
 * Baseline note: the loop body itself (read + ccount + a few DRAM bookkeeping writes) costs
 * ~25-45 cycles/iteration — that's what `min` will read, and it's the honest per-iteration floor
 * (the real bit-bang loop also does a read + a couple of ALU ops). What matters is the SPREAD:
 * max and the >=60/120/240-cycle bucket counts versus `min`, per phase.
 *
 * Not covered: Wi-Fi-driver bus load (the probe build skips LDN/Wi-Fi init). If the quiet phase
 * is clean but real sessions still corrupt, re-run this with the Wi-Fi stack brought up. */
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_reg.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "sdkconfig.h"

static const char *TAG = "busprobe";

/* From gba_core1.c / gba_spi.c */
extern void (*g_core1_entry)(void);
void gba_core1_start(void);
void gba_spi_init(void);
extern volatile uint32_t g_core1_alive;

static inline uint32_t ccount(void) { uint32_t c; __asm__ __volatile__("rsr %0,ccount" : "=r"(c)); return c; }

/* Probe stats — core 1 writes, core 0 reads (and zeroes at phase boundaries; the race is benign
 * for a diagnostic: worst case one iteration lands in the wrong phase's bucket). */
#define PROBE_BUCKETS 5
static const uint32_t s_thresh[PROBE_BUCKETS] = { 60, 120, 240, 2400, 24000 };  /* cycles @240MHz:
                                                   250ns   500ns  1us   10us   100us */
volatile uint32_t g_probe_iters;                  /* total loop iterations (proof of life + rate) */
volatile uint32_t g_probe_min = 0xFFFFFFFFu;      /* per-phase floor = clean loop cost */
volatile uint32_t g_probe_max;                    /* per-phase worst consecutive-read delta */
volatile uint32_t g_probe_bucket[PROBE_BUCKETS];  /* per-phase counts of deltas >= s_thresh[i] */

/* Core-1 bare loop. IRAM; touches only DRAM globals + GPIO_IN_REG — same residency rules as the
 * real adapter loop. Never returns. */
static void IRAM_ATTR probe_core1_entry(void)
{
    uint32_t prev = ccount();
    for (;;) {
        (void)REG_READ(GPIO_IN_REG);              /* the read under test */
        uint32_t now = ccount();
        uint32_t d = now - prev;
        prev = now;
        g_probe_iters++;
        if (d < g_probe_min) g_probe_min = d;
        if (d > g_probe_max) g_probe_max = d;
        if (d >= s_thresh[0]) {                   /* rare path — outliers only */
            for (int i = PROBE_BUCKETS - 1; i >= 0; i--) {
                if (d >= s_thresh[i]) { g_probe_bucket[i]++; break; }
            }
        }
    }
}

/* ---- core-0 load generators (each a plain FreeRTOS task, gated by a phase flag) ---- */
static volatile bool s_load_uart, s_load_flash, s_load_psram;

static void load_uart_task(void *arg)
{
    (void)arg;
    /* Long line ~every 2ms — heavier than the real monitor's cadence, deliberately: an upper
     * bound on logging-induced APB traffic. */
    static const char pad[] = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
    uint32_t n = 0;
    for (;;) {
        if (s_load_uart) ESP_LOGI(TAG, "uart-load %u %s%s", (unsigned)n++, pad, pad);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

static void load_flash_task(void *arg)
{
    (void)arg;
    static uint8_t buf[4096];
    uint32_t addr = 0;
    for (;;) {
        if (s_load_flash) {
            /* Real flash reads = cache-suspend windows + SPI1 bus traffic, back to back. */
            esp_flash_read(NULL, buf, 0x10000 + addr, sizeof(buf));
            addr = (addr + sizeof(buf)) & 0x3FFFF;
            taskYIELD();
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

static void load_psram_task(void *arg)
{
    (void)arg;
    uint8_t *a = heap_caps_malloc(65536, MALLOC_CAP_SPIRAM);
    uint8_t *b = heap_caps_malloc(65536, MALLOC_CAP_SPIRAM);
    if (!a || !b) { ESP_LOGW(TAG, "psram alloc failed — psram phase will be a no-op"); vTaskDelete(NULL); }
    memset(a, 0xA5, 65536);
    for (;;) {
        if (s_load_psram) { memcpy(b, a, 65536); memcpy(a, b, 65536); taskYIELD(); }
        else vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* Entry: replaces the LDN app when CONFIG_GBA_SPI_BUS_PROBE is set. Never returns. */
void gba_spi_bus_probe(void)
{
    gba_spi_init();                        /* same pin config as the real adapter (incl. pulls) */
    g_core1_entry = probe_core1_entry;     /* repoint BEFORE waking core 1 */
    gba_core1_start();
    ESP_LOGI(TAG, "BUS PROBE up: core1 timing back-to-back GPIO_IN_REG reads; 5s load phases "
                  "quiet->uart->flash->psram->all. Watch max & >=60cyc counts per phase.");
    xTaskCreatePinnedToCore(load_uart_task,  "bp_uart",  4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(load_flash_task, "bp_flash", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(load_psram_task, "bp_psram", 4096, NULL, 5, NULL, 0);

    static const char *phase_name[] = { "quiet", "uart", "flash", "psram", "ALL" };
    int phase = 0;
    for (;;) {
        s_load_uart  = (phase == 1 || phase == 4);
        s_load_flash = (phase == 2 || phase == 4);
        s_load_psram = (phase == 3 || phase == 4);
        /* reset per-phase stats (benign race with core 1 — diagnostics only) */
        g_probe_min = 0xFFFFFFFFu; g_probe_max = 0;
        for (int i = 0; i < PROBE_BUCKETS; i++) g_probe_bucket[i] = 0;
        uint32_t it0 = g_probe_iters;
        for (int s = 0; s < 5; s++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP_LOGI(TAG, "PHASE %-5s t=%ds iters=%u min=%u max=%u cyc | >=60:%u >=120:%u >=240:%u >=2400:%u >=24000:%u",
                     phase_name[phase], s + 1, (unsigned)(g_probe_iters - it0),
                     (unsigned)g_probe_min, (unsigned)g_probe_max,
                     (unsigned)g_probe_bucket[0], (unsigned)g_probe_bucket[1],
                     (unsigned)g_probe_bucket[2], (unsigned)g_probe_bucket[3],
                     (unsigned)g_probe_bucket[4]);
        }
        /* verdict line per phase: 60 cyc = 250ns = one half-bit @2MHz — any hit is a would-be
         * corrupted sample under the OLD two-read code; under the new single-read code the same
         * stall would instead delay edge DETECTION (rarely, a recoverable frame slip). */
        ESP_LOGW(TAG, "PHASE %-5s done: worst=%u cyc (%.0f ns) — %s", phase_name[phase],
                 (unsigned)g_probe_max, (double)g_probe_max * (1000.0 / 240.0),
                 g_probe_max >= 60 ? "EXCEEDS the 250ns half-bit budget => bus stalls CAN corrupt samples"
                                   : "inside budget");
        phase = (phase + 1) % 5;
    }
}
