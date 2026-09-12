/* gba_spi_hwmaster — drive the GBA link as CLOCK MASTER using the GPSPI3 hardware SPI peripheral
 * instead of bit-banged CPU edges (2026-09-11).
 *
 * WHY: every attempt to clock the RFU data-phase wake by hand (core1 bit-bang) left the GBA silent
 * (SO=0xFFFFFFFF, armed=0). The GBA slave is a hardware shift register — tolerant of slow/irregular
 * clocks but desynced *permanently* by a single phantom edge, and a software takeover of SC is a
 * phantom-edge factory. Prior art proves the fix: Shyri/gba-bt-hid multiboots a real GBA from an
 * ESP32 using the hardware SPI master (mode 3, 500 kHz, 32-bit words, ordinary FreeRTOS timing) —
 * the peripheral's silicon-generated clock is what makes it work. This module is the S3 equivalent,
 * callable from BARE-METAL core1 (no FreeRTOS): all transaction control is inline spi_ll_* register
 * code (esp_hal_gpspi), configured once from core0 at boot.
 *
 * PIN HANDOVER (the part Shyri never needed — his SPI owned the pins forever, ours borrows them
 * mid-session from the bit-bang GPIO layer):
 *   - SO (GPIO11, GBA->us = our MISO): input selectors are per-peripheral, so SPI3_Q_IN is routed
 *     ONCE at boot and simply coexists with the dedicated-GPIO input bundle. No handover at all.
 *   - SC (GPIO12, our CLK out during the window): mode 3 idles the peripheral CLK signal HIGH
 *     (ck_idle_edge, programmed at boot + latched by a dummy transaction). The caller first drives
 *     SC high via the existing dedic-GPIO clock-master path, then we swap the pad's out_sel to
 *     SPI3_CLK_OUT — high -> high, a single atomic register write, NO EDGE. Handing back is the
 *     reverse while the line is again idle-high (mode 3 ends every transaction with CLK high).
 *   - SI (GPIO13, our MOSI): glitches during the swap are harmless while SC is idle (the GBA only
 *     samples on SC edges), so it needs no special discipline — and it swaps back between words so
 *     the SO/SI reverse-ack interlock can keep using the dedic-GPIO path untouched.
 */
#include "gba_spi_hwmaster.h"
#include <string.h>
#include "soc/spi_struct.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "hal/spi_ll.h"
#include "hal/spi_types.h"
#include "esp_rom_gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_private/periph_ctrl.h"   /* PERIPH_RCC_ATOMIC for the clock-gate writes */

/* Same pins as gba_spi.c (kept local: this module must not pull in that file's statics). */
#define HWM_SC_PIN 12   /* CLK out during the master window */
#define HWM_SO_PIN 11   /* GBA->us = MISO (SPI3_Q_IN)       */
#define HWM_SI_PIN 13   /* us->GBA = MOSI (SPI3_D_OUT)      */

/* Dedic-GPIO out signals the pads normally use (must match gba_spi.c's init routing). */
#define HWM_SI_GPIO_SIG  CORE1_GPIO_OUT0_IDX
#define HWM_SC_GPIO_SIG  CORE1_GPIO_OUT1_IDX

#define HWM_HZ 256000   /* GBATEK-recommended external-clock rate for GBA normal mode */

static const char *TAG = "gba_hwm";

/* ---- core0, boot ---------------------------------------------------------------------------- */
void gba_spi_hwmaster_init(void)
{
    spi_dev_t *hw = &GPSPI3;

    PERIPH_RCC_ATOMIC() {
        spi_ll_enable_bus_clock(SPI3_HOST, true);
        spi_ll_reset_register(SPI3_HOST);
        spi_ll_enable_clock(SPI3_HOST, true);
    }

    spi_ll_master_init(hw);
    spi_ll_master_set_clock(hw, 80 * 1000 * 1000, HWM_HZ, 128);
    spi_ll_master_set_mode(hw, 3);            /* CPOL=1 CPHA=1 — programs ck_idle_edge=HIGH */
    spi_ll_set_half_duplex(hw, false);        /* full duplex: MOSI out + MISO in same clocks */
    spi_ll_set_sio_mode(hw, 0);
    spi_ll_enable_mosi(hw, 1);
    spi_ll_enable_miso(hw, 1);
    spi_ll_set_command_bitlen(hw, 0);         /* raw 32-bit data phase only — no cmd/addr/dummy */
    spi_ll_set_addr_bitlen(hw, 0);
    spi_ll_set_dummy(hw, 0);
    spi_ll_master_set_cs_setup(hw, 0);
    spi_ll_master_set_cs_hold(hw, 0);
    spi_ll_master_keep_cs(hw, 0);             /* no CS pad is ever routed — GBA link has no CS */
    spi_ll_apply_config(hw);

    /* MISO: route the GBA's SO pad into SPI3 Q once, forever (coexists with dedic-GPIO input). */
    esp_rom_gpio_connect_in_signal(HWM_SO_PIN, SPI3_Q_IN_IDX, false);

    /* Latch the idle levels with one dummy transaction while CLK/MOSI are still UNROUTED (they
     * only reach pads after gba_spi_hwmaster_pins_take). Research note: the CLK idle level is not
     * presented on the internal signal until a transaction has run (arduino-esp32 #9221) — after
     * this, SPI3_CLK_OUT sits solid-high, making the later pad swap edge-free. */
    uint32_t dummy = 0;
    spi_ll_write_buffer(hw, (const uint8_t *)&dummy, 32);
    spi_ll_set_mosi_bitlen(hw, 32);
    spi_ll_set_miso_bitlen(hw, 32);
    spi_ll_apply_config(hw);
    spi_ll_clear_int_stat(hw);
    spi_ll_user_start(hw);
    while (!spi_ll_usr_is_done(hw)) { }
    spi_ll_clear_int_stat(hw);

    ESP_LOGI(TAG, "GPSPI3 hardware clock-master ready: mode 3, %d Hz, pins SC=%d SI=%d SO=%d "
                  "(CLK/MOSI unrouted until the data-phase window)", HWM_HZ, HWM_SC_PIN, HWM_SI_PIN, HWM_SO_PIN);
}

/* ---- core1, bare-metal (all IRAM, no allocation, no RTOS) ----------------------------------- */

/* Caller contract: SC must already be DRIVEN HIGH via the dedic-GPIO clock-master path
 * (gba_spi_clock_master_acquire) so the out_sel swap is high->high with no edge. */
/* CRITICAL (found on hardware 2026-09-11, run 3): esp_rom_gpio_connect_out_signal also flips the
 * pad's OE control to the PERIPHERAL (oen_sel=0) — and GPSPI only asserts its CLK output-enable
 * DURING a transaction. Left that way, SC tristates between words, the board pulldown drags it
 * LOW, and the GBA sees a phantom falling edge + a wrong idle level (readbacks: SO moving as
 * levels at odd moments, never shifting). Every mux switch must therefore restore oen_sel=1 (OE
 * from GPIO_ENABLE, which the caller holds set) so the pad is CONSTANTLY driven by the selected
 * signal's level — idle-high CLK included. */
static inline void hwm_mux_out(uint32_t pin, uint32_t sig)
{
    esp_rom_gpio_connect_out_signal(pin, sig, false, false);
    GPIO.func_out_sel_cfg[pin].oen_sel = 1;          /* keep OE ours: pad never floats */
}

IRAM_ATTR void gba_spi_hwmaster_pins_take(void)
{
    hwm_mux_out(HWM_SI_PIN, SPI3_D_OUT_IDX);
    hwm_mux_out(HWM_SC_PIN, SPI3_CLK_OUT_IDX);
}

IRAM_ATTR void gba_spi_hwmaster_pins_give(void)
{
    /* Mode 3 ends with CLK high and the dedic-GPIO SC latch is still high from the acquire, so
     * swapping back is also high->high. Caller then releases (output disable) as before. */
    hwm_mux_out(HWM_SC_PIN, HWM_SC_GPIO_SIG);
    hwm_mux_out(HWM_SI_PIN, HWM_SI_GPIO_SIG);
}

/* Route ONLY SI between the dedic-GPIO path (SI-level interlock) and the SPI peripheral (word
 * transfer) — SC stays on the SPI peripheral (constantly driven, idle high) for the whole window. */
IRAM_ATTR void gba_spi_hwmaster_si_gpio(void)
{
    hwm_mux_out(HWM_SI_PIN, HWM_SI_GPIO_SIG);
}
IRAM_ATTR void gba_spi_hwmaster_si_spi(void)
{
    hwm_mux_out(HWM_SI_PIN, SPI3_D_OUT_IDX);
}

/* One full-duplex 32-bit word, MSB-first, clock generated by GPSPI3 silicon (zero CPU jitter).
 * Returns the word the GBA presented on SO. ~125us at 256kHz; polls the done bit. */
IRAM_ATTR uint32_t gba_spi_hwmaster_xfer_word(uint32_t tx)
{
    spi_dev_t *hw = &GPSPI3;
    /* MSB-first byte order: byte0 is shifted first = bits 31:24 (same layout Shyri uses). */
    uint8_t txb[4] = { (uint8_t)(tx >> 24), (uint8_t)(tx >> 16), (uint8_t)(tx >> 8), (uint8_t)tx };
    spi_ll_write_buffer(hw, txb, 32);
    spi_ll_set_mosi_bitlen(hw, 32);
    spi_ll_set_miso_bitlen(hw, 32);
    spi_ll_apply_config(hw);
    spi_ll_clear_int_stat(hw);
    spi_ll_user_start(hw);
    uint32_t guard = 0;
    while (!spi_ll_usr_is_done(hw)) {
        if (++guard > 40u * 1000u * 1000u) return 0xDEADBEEFu;  /* never-hang backstop, ~>100ms */
    }
    spi_ll_clear_int_stat(hw);
    uint8_t rxb[4];
    spi_ll_read_buffer(hw, rxb, 32);
    return ((uint32_t)rxb[0] << 24) | ((uint32_t)rxb[1] << 16) | ((uint32_t)rxb[2] << 8) | rxb[3];
}
