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
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "esp_attr.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
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

/* Direct-register GPIO for the tight loop (pins <32 → the low GPIO bank). gpio_get_level is too slow. */
#define CLK_HIGH()  (REG_READ(GPIO_IN_REG) & SC_MASK)
#define SO_HIGH()   (REG_READ(GPIO_IN_REG) & SO_MASK)
#define SI_HIGH()   REG_WRITE(GPIO_OUT_W1TS_REG, SI_MASK)
#define SI_LOW()    REG_WRITE(GPIO_OUT_W1TC_REG, SI_MASK)

/* Loop-count give-up for a stopped clock (GBA off/disconnected). Placed ONLY in the wait-for-LOW
 * (inter-bit) loop, never in the wait-for-HIGH+sample path — the RP2040 learned that a counter in the
 * sample path shifts the sample point by a bit (0x494E->0x929C). ~a few ms at 240MHz. */
#define CLK_GIVEUP 300000u
#define IDLE_WORD  0x80000000u

static bool s_inited;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* Progress counters — updated by the SPI core with plain writes (NO logging in the hot path, matching
 * the RP2040). A monitor task on the OTHER core prints them, so tracing never delays the bit-bang. */
volatile uint32_t g_gba_clock_seen, g_gba_logins, g_gba_cmds, g_gba_resets, g_gba_last_cmd, g_gba_last_reset;
#define GBA_CAP_N 48
volatile uint32_t g_cap[GBA_CAP_N];
volatile int g_cap_ready;

void gba_spi_init(void)
{
    if (s_inited) return;
    gpio_config_t in = {
        .pin_bit_mask = SC_MASK | SO_MASK | (1ULL << GBA_SD_PIN),
        .mode = GPIO_MODE_INPUT, .pull_up_en = 0, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in);
    gpio_config_t out = {
        .pin_bit_mask = SI_MASK, .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out);
    SI_HIGH();                 /* SI idles high between words */
    s_inited = true;
    ESP_LOGI(TAG, "bit-bang up (SC=%d in, SO=%d in, SI=%d out, SD=%d) — NO CS wire. GBA drives clock.",
             GBA_SC_PIN, GBA_SO_PIN, GBA_SI_PIN, GBA_SD_PIN);
}

/* One 32-bit exchange, MSB-first, the GBA clocking SC. Mirrors RP2040 spi.rs::transfer_u32:
 * wait SC low → drive SI bit → wait SC high (rising edge) → sample SO. Runs in a core1 critical
 * section (interrupts off) so nothing jitters the sample. IRAM so flash misses can't stall it.
 * Sets *ok=false and bails if the clock stalls (GBA gone). */
static IRAM_ATTR uint32_t xfer32(uint32_t tx, bool *ok)
{
    uint32_t rx = 0;
    *ok = true;
    portENTER_CRITICAL(&s_mux);
    for (int i = 31; i >= 0; i--) {
        uint32_t guard = 0;
        while (CLK_HIGH()) {                     /* wait for SC low (inter-bit gap) */
            if (++guard > CLK_GIVEUP) { *ok = false; portEXIT_CRITICAL(&s_mux); return rx; }
        }
        if ((tx >> i) & 1u) SI_HIGH(); else SI_LOW();   /* present our bit while SC is low */
        while (!CLK_HIGH()) { }                  /* wait for rising edge — TIGHT, no counter */
        rx = (rx << 1) | (SO_HIGH() ? 1u : 0u);  /* sample the GBA's bit */
    }
    portEXIT_CRITICAL(&s_mux);
    return rx;
}

bool gba_spi_exchange_raw(uint32_t tx_word, uint32_t *rx_word)
{
    if (!s_inited) return false;
    bool ok = false;
    uint32_t rx = xfer32(tx_word, &ok);
    if (ok && rx_word) *rx_word = rx;
    return ok;
}

/* Post-transfer ack, mirroring gpi.rs::ack_recv: SI low, brief settle, SI high, then wait for the GBA
 * to raise SO (its ack). Returns false if the GBA never acks (stalled). */
static bool ack_recv(void)
{
    SI_LOW();
    esp_rom_delay_us(1);   /* ~match RP2040 gpi.rs ack_recv (~0.8us); 2us was too wide */
    SI_HIGH();
    uint32_t guard = 0;
    while (!SO_HIGH()) {
        if (++guard > CLK_GIVEUP) return false;
    }
    return true;
}

/* Clock exactly 16 bits and discard them (present idle SI) — used once after login to eat the stray
 * half-word at the login→command boundary and re-align our 32-bit framing to the GBA's. */
static void consume_16(void)
{
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < 16; i++) {
        uint32_t g = 0;
        while (CLK_HIGH()) { if (++g > CLK_GIVEUP) { portEXIT_CRITICAL(&s_mux); return; } }
        SI_HIGH();                       /* idle high while re-aligning */
        while (!CLK_HIGH()) { }
    }
    portEXIT_CRITICAL(&s_mux);
}

bool gba_spi_exchange_word(uint32_t tx_word, uint32_t *rx_word)
{
    if (!s_inited) return false;
    SI_LOW();                         /* gpi.rs::ack_ready_send: SI low = "ready to send" */
    bool ok = false;
    uint32_t rx = xfer32(tx_word, &ok);
    if (!ok) { SI_HIGH(); return false; }
    bool acked = ack_recv();
    SI_HIGH();                        /* idle-high between words */
    if (rx_word) *rx_word = rx;
    return acked;
}

/* ---- full adapter: login (bare exchanges) then the WAP command loop via gba_wap ---- */
void gba_spi_run_adapter(struct gba_wap_io *io)
{
    gba_spi_init();
    /* Outer loop mirrors RP2040 routes.rs::run(): (re)login, then serve commands until the stream
     * resets — and the GBA RE-RUNS login on room entry, so we must be able to drop back to login.
     * ZERO logging in this loop — like the RP2040. Prints from the monitor task (other core). */
    for (;;) {
        /* ---- login: bare exchanges, walk the challenge/response to 0xB0BB8001 ---- */
        uint32_t tx = 0x00000000;
        bool done = false;
        while (!done) {
            uint32_t rx = 0;
            if (!gba_spi_exchange_raw(tx, &rx)) { vTaskDelay(1); continue; }
            g_gba_clock_seen = 1;
            tx = gba_login_next(rx, &done);
        }
        g_gba_logins++;

        /* DIAGNOSTIC: capture the RAW post-login word stream (no re-align) so we can search it offline
         * for the 0x9966 header at ANY bit offset and measure the true framing/bit error. */
        for (int i = 0; i < GBA_CAP_N; i++) { uint32_t w = 0; gba_spi_exchange_raw(IDLE_WORD, &w); g_cap[i] = w; }
        g_cap_ready = 1;

        /* ---- command loop: serve WAP commands until an invalid header (= reset / GBA re-login) ---- */
        int bad = 0;
        for (;;) {
            uint32_t hdr = 0;
            if (!gba_spi_exchange_word(IDLE_WORD, &hdr)) { if (++bad > 4) break; else continue; }
            if (!gba_wap_header_valid(hdr)) { g_gba_resets++; g_gba_last_reset = hdr; break; }
            bad = 0;
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
static int      stub_get_peer(uint32_t o[7], void *c) { (void)o; (void)c; return 0; }
static uint32_t stub_peer_id(void *c)                 { (void)c; return 0; }
static int      stub_take_slot(uint32_t *o, int m, void *c) { (void)o; (void)m; (void)c; return 0; }
static void     stub_put_slot(const uint32_t *d, int l, void *c) { (void)d; (void)l; (void)c; }

static void adapter_task(void *arg) { gba_spi_run_adapter((struct gba_wap_io *)arg); }

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
    ESP_LOGI(TAG, "SELFTEST: bit-bang adapter on CORE 1, stub providers. Put GBA in wireless mode + "
                  "enter the room; watch cmds= climb (should stay connected solo).");
    xTaskCreatePinnedToCore(adapter_task, "gba_adapter", 8192, &io, configMAX_PRIORITIES - 2, NULL, 1);
    bool dumped = false;
    for (;;) {   /* core-0 monitor: prints never touch core-1's timing */
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "MON clk=%u logins=%u cmds=%u last_cmd=0x%02x resets=%u last_reset=0x%08x",
                 (unsigned)g_gba_clock_seen, (unsigned)g_gba_logins, (unsigned)g_gba_cmds,
                 (unsigned)g_gba_last_cmd, (unsigned)g_gba_resets, (unsigned)g_gba_last_reset);
        if (g_cap_ready && !dumped) {
            dumped = true;
            ESP_LOGI(TAG, "CAP raw post-login words (find 0x9966 here):");
            for (int i = 0; i < GBA_CAP_N; i += 6)
                ESP_LOGI(TAG, "CAP[%02d] %08x %08x %08x %08x %08x %08x", i,
                         (unsigned)g_cap[i], (unsigned)g_cap[i+1], (unsigned)g_cap[i+2],
                         (unsigned)g_cap[i+3], (unsigned)g_cap[i+4], (unsigned)g_cap[i+5]);
        }
    }
}
