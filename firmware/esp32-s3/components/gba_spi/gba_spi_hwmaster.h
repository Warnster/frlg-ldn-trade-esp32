/* gba_spi_hwmaster — GPSPI3 hardware SPI as the GBA-link CLOCK MASTER for the RFU data-phase
 * wake window. See gba_spi_hwmaster.c for the full rationale (bit-banged master edges left the
 * GBA silent; hardware-generated clock is the proven fix — Shyri/gba-bt-hid precedent). */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Core0, boot, BEFORE gba_core1_start(): configure GPSPI3 (mode 3, 256 kHz, full duplex, no CS),
 * route MISO once, and latch idle levels with a dummy transaction while CLK/MOSI are unrouted. */
void gba_spi_hwmaster_init(void);

/* Core1, IRAM. Caller must hold SC driven HIGH via gba_spi_clock_master_acquire() first:
 * take = swap SC+SI pads to the SPI signals (edge-free), give = swap back. */
void gba_spi_hwmaster_pins_take(void);
void gba_spi_hwmaster_pins_give(void);

/* Toggle ONLY the SI pad between SPI (word transfer) and dedic-GPIO (reverse-ack interlock);
 * SC stays on the SPI peripheral for the whole window. */
void gba_spi_hwmaster_si_gpio(void);
void gba_spi_hwmaster_si_spi(void);

/* One hardware-clocked full-duplex 32-bit word (MSB-first). Returns the GBA's word from SO;
 * 0xDEADBEEF only on the never-hang guard (peripheral wedged — should not happen). */
uint32_t gba_spi_hwmaster_xfer_word(uint32_t tx);

#ifdef __cplusplus
}
#endif
