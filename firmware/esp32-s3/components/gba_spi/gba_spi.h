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

/* Start the core1-pinned service task. `provider` is called with each received word and returns the
 * next word to present (e.g. wired to the WAP/RFU adapter FSM). ctx is passed through. */
void gba_spi_start(gba_spi_provider_t provider, void *ctx);

/* GATE 1/2 bring-up self-test (design doc §8): logs the first words the GBA clocks in, so SPI mode
 * (Mode 3 vs 0) and wiring can be confirmed before the full protocol port. Never returns. */
void gba_spi_selftest(void);

#ifdef __cplusplus
}
#endif
